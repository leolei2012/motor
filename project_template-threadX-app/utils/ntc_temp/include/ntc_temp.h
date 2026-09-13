

#ifndef NTC_TEMP_H
#define NTC_TEMP_H

#include "platform.h"


#define NTC_OK (0)
#define NTC_ERR_NULL (-1)       /**< 空指针 */
#define NTC_ERR_ADC_RANGE (-2)  /**< ADC 超出量程（短路/断路） */
#define NTC_ERR_TEMP_RANGE (-3) /**< 温度超出查找表范围 */

/** 温度无效值（用于错误返回） */
#define NTC_TEMP_INVALID (0x7FFF)


struct ntc_table_entry_t
{
    int16_t temp_x10;    /**< 温度，单位 0.1°C，例如 -400 = -40.0°C */
    uint16_t ad;         /**< ADC 原始采样值 */
};


struct ntc_handle_t
{
    /** ── 硬件参数 ── */
    uint16_t adc_max;     /**< ADC 满量程值，例如 12 位 = 4095 */

    /** ── 查找表 ── */
    const struct ntc_table_entry_t *table; /**< 查找表指针（AD 单调排列，温度从低到高）*/
    uint16_t table_len;                    /**< 表条目数量 */


    uint8_t _initialized;
};



/**
 * @brief 初始化 NTC 句柄
 * @param h           句柄指针
 * @param adc_max     ADC 最大值（12位→4095，10位→1023）
 * @param table       查找表
 * @param table_len   查找表长度
 * @return NTC_OK 或错误码
 */
int8_t ntc_init(struct ntc_handle_t *h, uint16_t adc_max, const struct ntc_table_entry_t *table, uint16_t table_len);

/**
 * @brief  输入原始 ADC 值，返回温度
 * @param  h        句柄指针
 * @param  adc_raw  原始 ADC 采样值
 * @param  temp_x10 [out] 温度结果，单位 0.1°C
 * @return NTC_OK 或错误码
 *
 * @note 每个采样周期调用一次；带滤波时建议固定采样间隔
 */
int8_t ntc_get_temp(struct ntc_handle_t *h, uint16_t adc_raw, int16_t *temp_x10);

/**
 * @brief  仅做 AD→温度转换（不含采样和滤波，供调试用）
 * @param  h        句柄指针
 * @param  ad_raw   ADC 原始采样值
 * @param  temp_x10 [out] 温度结果，单位 0.1°C
 * @return NTC_OK 或错误码
 */
int8_t ntc_ad_to_temp(const struct ntc_handle_t *h, uint16_t ad_raw, int16_t *temp_x10);


/** 100kΩ @ 25°C，B=3950（常用红外/工业 NTC） */
extern const struct ntc_table_entry_t ntc_table_jxw3752[];
extern const uint16_t ntc_table_jxw3752_len;


#if 0
/** 10kΩ @ 25°C，B=3950（常见消费类 NTC） */
extern const struct ntc_table_entry_t ntc_table_10k_b3950[];
extern const uint16_t ntc_table_10k_b3950_len;

/** 10kΩ @ 25°C，B=3435（精密 NTC，如 Murata NCP15） */
extern const struct ntc_table_entry_t ntc_table_10k_b3435[];
extern const uint16_t ntc_table_10k_b3435_len;
#endif


#endif // NTC_TEMP_H