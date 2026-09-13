#ifndef RCM_H
#define RCM_H

#include <stdbool.h>
#include <stdint.h>

/*
 *  Regular Conversion Manager — 硬件无关的 ADC 通道轮询调度器
 *
 *  核心思想：RCM 只管"谁排到了、结果存哪"，不管 ADC 寄存器怎么操作。
 *  所有硬件操作通过 rcm_adc_ops_t 函数指针注入，移植时只需实现这组接口。
 *
 *  生产者-消费者模型：
 *    - 生产者 (中断):  rcm_read_ongoing() → rcm_exec_next()，不阻塞
 *    - 消费者 (线程):  rcm_get()，纯读缓存
 */

#define RCM_MAX_CONV  4

/** ADC 硬件操作接口 — 移植时需要实现的 5 个函数 */
typedef struct
{
    /*
     * 启动一次 ADC 转换（非阻塞）
     * @param channel  ADC 通道号（rcm_conv_t.channel）
     * @param user_data 用户自定义上下文
     */
    void (*start_conversion)(uint8_t channel, void *user_data);

    /*
     * 检查转换是否完成
     * @return true=已完成，false=仍在转换中
     */
    bool (*is_conversion_done)(void *user_data);

    /*
     * 读取转换结果
     * @return ADC 原始值
     */
    uint16_t (*read_result)(void *user_data);

    /*
     * 硬件初始化：校准、使能 ADC
     * 仅在该 ADC 被首次注册时调用一次
     */
    void (*init)(void *user_data);

    /*
     * 配置通道的采样时间
     */
    void (*set_sample_time)(uint8_t channel, uint32_t sample_time, void *user_data);

} rcm_adc_ops_t;

/*
 * 注册一个 ADC 转换
 *
 * 同一组 ADC（共享 ops + user_data）的多个通道共用一个索引空间。
 * 不同 ADC 各自独立注册，各自维护轮询队列。
 */
typedef struct
{
    uint8_t      channel;  /**< ADC 通道号 */
    uint32_t     sample_time;  /**< 采样周期（平台相关，透传给 set_sample_time） */
    uint16_t     data;  /**< 最新转换结果（RCM 写入，用户只读） */
    uint8_t      id;  /**< 内部索引，注册时自动分配 */
} rcm_conv_t;

typedef struct
{
    rcm_conv_t    *slots[RCM_MAX_CONV];  /**< 已注册的通道 */
    uint8_t        count;  /**< 实际注册数量 */
    uint8_t        index;  /**< 当前轮询位置 */
    rcm_adc_ops_t  ops;  /**< 硬件操作接口 */
    void          *user_data;  /**< 透传给 ops 的上下文 */
    bool           init_done;  /**< 首次注册时已调过 init */
} rcm_t;

/** 初始化 RCM 实例，绑定硬件操作 */
void      rcm_init(rcm_t *rcm, const rcm_adc_ops_t *ops, void *user_data);

/** 注册一个通道（返回 true 成功） */
bool      rcm_register(rcm_t *rcm, rcm_conv_t *conv);

/** 非阻塞：收上一个 + 启动下一个（中断里调用） */
void      rcm_read_ongoing(rcm_t *rcm);
void      rcm_exec_next(rcm_t *rcm);

/** 同步阻塞转换（仅用于初始化/调试） */
uint16_t  rcm_exec_blocking(rcm_t *rcm, rcm_conv_t *conv);

/** 等待当前转换完成 */
void      rcm_wait(rcm_t *rcm);

/** 读缓存（不触发硬件） */
static inline uint16_t rcm_get(const rcm_conv_t *conv)
{
    return (conv != NULL) ? conv->data : 0;
}

#endif /* RCM_H */