#include "drv_ain_sensor.h"

#include "hal.h"

/*
 *  ST 风格：
 *   - TIM6 1kHz ISR → drv_ain_sensor_sample() → ADC 触发 + EMA
 *   - 100ms 线程 → drv_ain_sensor_poll() → 物理量转换
 */

/** ============================================================
   初始化单个传感器通道
   ============================================================ */
static void channel_init(struct drv_ain_sensor_channel *ch,
                         uint8_t adc_channel, uint32_t sample_time, uint8_t ema_shift)
{
    memset(ch, 0, sizeof(*ch));
    ch->adc_channel = adc_channel;
    ch->sample_time = sample_time;
    ema_init(&ch->ema, ema_shift);

    if (adc_channel != 0u)
    {
        hal_adc1_set_channel_sample_time(adc_channel, sample_time);
    }
}

/** ============================================================
   触发一次转换 + 读取 + EMA 滤波
   ============================================================ */
static void channel_sample(struct drv_ain_sensor_channel *ch)
{
    if (ch->adc_channel == 0u)
    {
        return;
    }

    /* 非阻塞两拍: 读上一拍触发的结果 + EMA, 再触发下一拍。
     * 规则组与 16kHz 注入组共享 ADC1, 阻塞等 EOC 会饿死调度器(跑飞),
     * 这里让转换在两次 1kHz 节拍之间(1ms)完成, 完全不阻塞。 */
    uint16_t raw = hal_adc1_reg_read_result();
    ch->filtered_adc = ema_update(&ch->ema, raw);
    hal_adc1_reg_trigger_channel(ch->adc_channel);
}

/** ============================================================
   初始化
   ============================================================ */
void drv_ain_sensor_init(struct drv_ain_sensor *self)
{
    if (self == NULL)
    {
        return;
    }

    memset(self, 0, sizeof(*self));

    channel_init(&self->bus_voltage.ch,
                 DRV_AIN_BUS_VOLTAGE_ADC_CHANNEL,
                 DRV_AIN_BUS_VOLTAGE_SAMPLE_TIME,
                 DRV_AIN_BUS_VOLTAGE_EMA_SHIFT);

    /* 触发首拍转换, 供第一个 TIM6 ISR 读取 */
    hal_adc1_reg_trigger_channel(self->bus_voltage.ch.adc_channel);

    hal_tim6_start();
}

/** ============================================================
   TIM6 ISR (1kHz): 逐个通道触发 ADC + EMA 滤波
   ============================================================ */
void drv_ain_sensor_tim_isr(struct drv_ain_sensor *self)
{
    if (self == NULL)
    {
        return;
    }

    channel_sample(&self->bus_voltage.ch);
}



/** ============================================================
   100ms 轮询: filtered_adc → 物理量
   ============================================================ */
void drv_ain_sensor_poll(struct drv_ain_sensor *self)
{
    if (self == NULL)
    {
        return;
    }

    self->bus_voltage.voltage_mv   = bus_voltage_get_voltage_mv(self->bus_voltage.ch.filtered_adc);
    self->bus_voltage.voltage_raw_d = self->bus_voltage.ch.filtered_adc;
}

/** ============================================================
   u16Volt → 实际电压 (仿 ST VBS_GetAvBusVoltage_V)
   Vbus_mV = raw_d * ConvFactor_mV / 65536
           = raw_d * 82500 >> 16
   ============================================================ */
uint32_t bus_voltage_get_voltage_mv(uint16_t raw_d)
{
    return (uint32_t)(((uint64_t)raw_d * 82500u) >> 16);
}