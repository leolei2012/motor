#ifndef DRV_AIN_SENSOR_H
#define DRV_AIN_SENSOR_H

#include "platform.h"
#include "bsp_config.h"

#include "ntc_temp.h"
#include "filter.h"

/*
 *  ADC 通道分配（根据实际硬件修改）
 *
 *  ADC1 (共享 ADC12 时钟):
 *    - 注入组 (TIM1 触发): CH7=PC1(电流U), CH8=PC2(电流V)
 *    - 规则组 (软件触发, poll 中阻塞读取): CH4=PA3(母线电压)
 *
 *  ADC2 (共享 ADC12 时钟):
 *    - 注入组 (TIM1 触发): CH8=PC2(电流V), CH9=PC3(电流W)
 *    - 规则组 (预留): 蒸发器温度 NTC, 湿度, 环境温度 NTC
 */

/** —— 母线电压传感器参数 ——
 * 分压比 1/25: Vbus = Vadc / 0.04
 * ADC 参考电压 3.3V, 12 位左对齐
 * u16Volt 格式 (ST 兼容): hVbus = ADC_raw * (65536 / ConversionFactor)
 *   ConversionFactor = ADC_REFERENCE_VOLTAGE / VBUS_PARTITIONING_FACTOR
 *                    = 3.3 / 0.04 = 82.5
 */
#define DRV_AIN_BUS_VOLTAGE_ADC_CHANNEL   4u
#define DRV_AIN_BUS_VOLTAGE_SAMPLE_TIME   LL_ADC_SAMPLINGTIME_47CYCLES_5
#define DRV_AIN_BUS_VOLTAGE_CONV_FACTOR   82U  /**< 3.3 / 0.04, 取整 */
#define DRV_AIN_BUS_VOLTAGE_EMA_SHIFT     8u  /**< alpha = 1/256 */

/** —— v_dc —— */
#define DRV_AIN_EVA_V_DC_ADC_CHANNEL      0u  /**< TODO: 填入实际通道 */
#define DRV_AIN_EVA_V_DC_SAMPLE_TIME      LL_ADC_SAMPLINGTIME_47CYCLES_5
#define DRV_AIN_EVA_V_DC_EMA_SHIFT        8u



/** ============================================================
   单个传感器通道：ADC 参数 + EMA 滤波器
   ============================================================ */
struct drv_ain_sensor_channel
{
    uint8_t                adc_channel;
    uint32_t               sample_time;
    struct ema_filter_t    ema;
    uint16_t               filtered_adc;
};

/** ============================================================
   ain_sensor 顶层结构
   ============================================================ */
struct drv_ain_sensor
{
    /** —— 母线电压传感器（ADC1 CH4, PA3, 1/25 分压）—— */
    struct
    {
        struct drv_ain_sensor_channel ch;
        uint32_t voltage_mv;  /**< 母线电压 (mV) */
        uint16_t voltage_raw_d;  /**< u16Volt 格式 (ST 兼容) */
    } bus_voltage;

};

/** ============================================================
   API
   ============================================================ */
void drv_ain_sensor_init(struct drv_ain_sensor *self);

/*
 * TIM6 ISR (1kHz) 调用：软件触发 ADC → 阻塞等 EOC → EMA 滤波
 * 采样节拍由硬件定时器保证，不受 RTOS 调度 jitter 影响。
 */
void drv_ain_sensor_tim_isr(struct drv_ain_sensor *self);

/*
 * 仿 ST VBS_GetAvBusVoltage_V：u16Volt → 实际电压
 * raw_d: 12-bit 左对齐 ADC 值 (0~65535)
 * 返回: 母线电压 (mV)
 */
uint32_t bus_voltage_get_voltage_mv(uint16_t raw_d);

#define DRV_AIN_SENSOR_TASK_PERIOD 100u

/*
 * 100ms 任务调用：将 filtered_adc 转换为物理量 (mV, °C, %RH)
 * 不做 ADC 操作，只做数值转换。
 */
void drv_ain_sensor_poll(struct drv_ain_sensor *self);

#endif /* DRV_AIN_SENSOR_H */