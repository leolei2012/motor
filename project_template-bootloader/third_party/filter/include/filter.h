#ifndef FILTER_H
#define FILTER_H

#include "platform.h"

#if 1
// 滤波器类型
typedef enum
{
    FILTER_NONE = 0,    // 无滤波
    FILTER_MOVING_AVG,  // 滑动平均滤波
    FILTER_EXPONENTIAL, // 一阶低通滤波
    FILTER_MEDIAN,      // 中值滤波
    FILTER_KALMAN       // 卡尔曼滤波
} filter_type_t;

// 滑动平均滤波器结构体
struct moving_avg_filter_t
{
    uint16_t buffer[16]; // 缓冲区（最大16个）
    uint8_t index;       // 当前索引
    uint8_t count;       // 有效数据个数
    uint8_t size;        // 窗口大小
    uint32_t sum;        // 累加和
    uint16_t output;     // 输出值
};

#endif

#if 1
// 一阶低通滤波器结构体
struct lowpass_filter_t
{
    uint16_t output;      // 上次输出
    uint16_t alpha;       // 滤波系数（0-255，对应0-1）
    uint16_t alpha_recip; // 1-alpha
} ;

#endif

#if 1
// 指数滑动平均（移位实现，ST MCSDK 风格）
struct ema_filter_t
{
    uint16_t output;
    uint8_t shift;
};

#endif

#if 0
// 中值滤波器结构体
typedef struct {
    uint16_t buffer[9];     // 缓冲区（奇数个）
    uint8_t size;           // 窗口大小（3,5,7,9）
    uint8_t count;          // 当前数据个数
    uint16_t output;        // 输出值
} median_filter_t;

// 卡尔曼滤波器结构体（简单版本）
typedef struct {
    float q;                // 过程噪声协方差
    float r;                // 测量噪声协方差
    float x;                // 估计值
    float p;                // 估计误差协方差
    float k;                // 卡尔曼增益
    uint8_t initialized;    // 初始化标志
} kalman_filter_t;

// 复合滤波器（组合多种滤波）
typedef struct {
    filter_type_t primary;           // 主滤波类型
    moving_avg_filter_t moving_avg;
    lowpass_filter_t lowpass;
    median_filter_t median;
    kalman_filter_t kalman;
    uint16_t raw_speed;              // 原始转速
    uint16_t filtered_speed;         // 滤波后转速
    uint8_t initialized;
} motor_filter_t;
#endif
#if 1

// 各滤波器独立API
void moving_avg_init(struct moving_avg_filter_t *maf, uint8_t window_size);
uint16_t moving_avg_update(struct moving_avg_filter_t *maf, uint16_t value);

#endif

#if 1
void lowpass_init(struct lowpass_filter_t *lpf, uint16_t alpha);
uint16_t lowpass_update(struct lowpass_filter_t *lpf, uint16_t value);
#endif

#if 1
void ema_init(struct ema_filter_t *ema, uint8_t shift);
uint16_t ema_update(struct ema_filter_t *ema, uint16_t value);
#endif

#if 0
void median_init(median_filter_t *mf, uint8_t window_size);
uint16_t median_update(median_filter_t *mf, uint16_t value);

void kalman_init(kalman_filter_t *kf, float q, float r);
uint16_t kalman_update(kalman_filter_t *kf, uint16_t value);
#endif
#endif // FILTER_H
