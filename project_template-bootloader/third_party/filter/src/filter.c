#include "filter.h"

#include "filter.h"
#include <string.h>

#if 1

/* ============================================================
   滑动平均滤波
   ============================================================ */

/**
 * @brief 初始化滑动平均滤波器
 * @param maf         滤波器指针
 * @param window_size 窗口大小（1-16）
 */
void moving_avg_init(struct moving_avg_filter_t *maf, uint8_t window_size)
{
    if (maf == NULL)
        return;

    if (window_size > 16)
        window_size = 16;
    if (window_size < 1)
        window_size = 1;

    memset(maf->buffer, 0, sizeof(maf->buffer));
    maf->index = 0;
    maf->count = 0;
    maf->size = window_size;
    maf->sum = 0;
    maf->output = 0;
}

/**
 * @brief 滑动平均滤波更新
 * @param maf   滤波器指针
 * @param value 原始值
 * @return 滤波后值
 */
uint16_t moving_avg_update(struct moving_avg_filter_t *maf, uint16_t value)
{
    uint32_t temp_sum;

    if (maf == NULL)
        return value;

    // 移除旧值
    if (maf->count >= maf->size)
    {
        maf->sum -= maf->buffer[maf->index];
    }
    else
    {
        maf->count++;
    }

    // 添加新值
    maf->buffer[maf->index] = value;
    maf->sum += value;

    // 更新索引
    maf->index++;
    if (maf->index >= maf->size)
    {
        maf->index = 0;
    }

    // 计算平均值
    temp_sum = maf->sum;
    if (maf->count > 0)
    {
        maf->output = (uint16_t)(temp_sum / maf->count);
    }

    return maf->output;
}

#endif

#if 1
/* ============================================================
   一阶低通滤波
   ============================================================ */

/**
 * @brief 初始化一阶低通滤波器
 * @param lpf   滤波器指针
 * @param alpha 滤波系数（0-255，对应0-1）
 *              alpha越大，响应越快，滤波效果越弱
 *              推荐值：10-50
 */
void lowpass_init(struct lowpass_filter_t *lpf, uint16_t alpha)
{
    if (lpf == NULL)
        return;

    if (alpha > 255)
        alpha = 255;

    lpf->alpha = alpha;
    lpf->alpha_recip = 256 - alpha; // 1-alpha（放大256倍）
    lpf->output = 0;
}

/**
 * @brief 一阶低通滤波更新
 * @param lpf   滤波器指针
 * @param value 原始值
 * @return 滤波后值
 */
uint16_t lowpass_update(struct lowpass_filter_t *lpf, uint16_t value)
{
    uint32_t temp;

    if (lpf == NULL)
        return value;

    // 一阶低通：output = output * (1-alpha) + value * alpha
    // 使用整数运算，放大256倍
    temp = (uint32_t)lpf->output * lpf->alpha_recip + (uint32_t)value * lpf->alpha;

    lpf->output = (uint16_t)(temp / 256);

    return lpf->output;
}

#endif

#if 1
/* ============================================================
   指数滑动平均（移位实现）
   等效于: output += (value - output) >> shift
          output = value * (1/2^shift) + output * (1 - 1/2^shift)
============================================================ */

/**
 *
 * ┌──────────────┬─────────────────────────┬──────────────────────────┐
 * │              │  ema_filter_t（新增）   │ lowpass_filter_t（已有） │
 * ├──────────────┼─────────────────────────┼──────────────────────────┤
 * │ 每周期运算   │ 1 减 + 1 移位 + 1 加    │ 2 乘 + 1 加 + 1 除       │
 * ├──────────────┼─────────────────────────┼──────────────────────────┤
 * │ alpha 范围   │ 固定 1/2^shift          │ 0~255 任意值             │
 * ├──────────────┼─────────────────────────┼──────────────────────────┤
 * │ 等效 ST 公式 │ out += (val - out) >> 8 │ out*255/256 + val*1/256  │
 * └──────────────┴─────────────────────────┴──────────────────────────┘
 */

void ema_init(struct ema_filter_t *ema, uint8_t shift)
{
    if (ema == NULL)
        return;

    ema->output = 0;
    ema->shift = shift;
}

uint16_t ema_update(struct ema_filter_t *ema, uint16_t value)
{
    int32_t diff;

    if (ema == NULL)
        return value;

    diff = (int32_t)value - (int32_t)ema->output;
    ema->output = (uint16_t)((int32_t)ema->output + (diff >> ema->shift));

    return ema->output;
}

#endif

#if 0

/* ============================================================
   中值滤波
   ============================================================ */

// 冒泡排序（针对小数组优化）
static void bubble_sort(uint16_t *arr, uint8_t n)
{
    uint8_t i, j;
    uint16_t temp;

    for (i = 0; i < n - 1; i++) {
        for (j = 0; j < n - i - 1; j++) {
            if (arr[j] > arr[j + 1]) {
                temp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;
            }
        }
    }
}

/**
 * @brief 初始化中值滤波器
 * @param mf          滤波器指针
 * @param window_size 窗口大小（3,5,7,9）
 */
void median_init(median_filter_t *mf, uint8_t window_size)
{
    if (mf == NULL) return;

    if (window_size > 9) window_size = 9;
    if (window_size < 3) window_size = 3;
    if (window_size % 2 == 0) window_size++;  // 确保奇数

    memset(mf->buffer, 0, sizeof(mf->buffer));
    mf->size = window_size;
    mf->count = 0;
    mf->output = 0;
}

/**
 * @brief 中值滤波更新
 * @param mf    滤波器指针
 * @param value 原始值
 * @return 滤波后值
 */
uint16_t median_update(median_filter_t *mf, uint16_t value)
{
    uint16_t temp[9];
    uint8_t i;

    if (mf == NULL) return value;

    // 移动窗口
    if (mf->count < mf->size) {
        mf->buffer[mf->count] = value;
        mf->count++;

        if (mf->count < mf->size) {
            return value;
        }
    } else {
        // 队列移动
        for (i = 0; i < mf->size - 1; i++) {
            mf->buffer[i] = mf->buffer[i + 1];
        }
        mf->buffer[mf->size - 1] = value;
    }

    // 复制并排序
    for (i = 0; i < mf->size; i++) {
        temp[i] = mf->buffer[i];
    }
    bubble_sort(temp, mf->size);

    // 取中值
    mf->output = temp[mf->size / 2];

    return mf->output;
}

/* ============================================================
   卡尔曼滤波（简单版本）
   ============================================================ */

/**
 * @brief 初始化卡尔曼滤波器
 * @param kf   滤波器指针
 * @param q    过程噪声协方差（推荐0.01-0.1）
 * @param r    测量噪声协方差（推荐0.1-1.0）
 */
void kalman_init(kalman_filter_t *kf, float q, float r)
{
    if (kf == NULL) return;

    kf->q = q;
    kf->r = r;
    kf->x = 0;
    kf->p = 1.0;
    kf->k = 0;
    kf->initialized = 0;
}

/**
 * @brief 卡尔曼滤波更新（定点数优化版本）
 * @param kf    滤波器指针
 * @param value 原始值
 * @return 滤波后值
 */
uint16_t kalman_update(kalman_filter_t *kf, uint16_t value)
{
    float z;  // 测量值

    if (kf == NULL) return value;

    z = (float)value;

    if (!kf->initialized) {
        kf->x = z;
        kf->initialized = 1;
        return value;
    }

    // 预测
    kf->p = kf->p + kf->q;

    // 更新
    kf->k = kf->p / (kf->p + kf->r);
    kf->x = kf->x + kf->k * (z - kf->x);
    kf->p = (1.0 - kf->k) * kf->p;

    return (uint16_t)kf->x;
}

/* ============================================================
   卡尔曼滤波（定点数优化版本，无浮点）
   ============================================================ */

// 定点数缩放因子
#define KALMAN_SCALE 1024

typedef struct {
    int32_t q;      // 过程噪声（放大1024倍）
    int32_t r;      // 测量噪声（放大1024倍）
    int32_t x;      // 估计值（放大1024倍）
    int32_t p;      // 估计误差协方差（放大1024倍）
    int32_t k;      // 卡尔曼增益（放大1024倍）
    uint8_t initialized;
} kalman_fixed_t;

/**
 * @brief 定点数卡尔曼滤波初始化
 * @param kf   滤波器指针
 * @param q    过程噪声（推荐10-100）
 * @param r    测量噪声（推荐100-1000）
 */
void kalman_fixed_init(kalman_fixed_t *kf, int32_t q, int32_t r)
{
    if (kf == NULL) return;

    kf->q = q;
    kf->r = r;
    kf->x = 0;
    kf->p = KALMAN_SCALE;  // 初始P=1
    kf->k = 0;
    kf->initialized = 0;
}

/**
 * @brief 定点数卡尔曼滤波更新（无浮点运算）
 * @param kf    滤波器指针
 * @param value 原始值
 * @return 滤波后值
 */
uint16_t kalman_fixed_update(kalman_fixed_t *kf, uint16_t value)
{
    int32_t z;

    if (kf == NULL) return value;

    z = (int32_t)value * KALMAN_SCALE;

    if (!kf->initialized) {
        kf->x = z;
        kf->initialized = 1;
        return value;
    }

    // 预测
    kf->p = kf->p + kf->q;

    // 更新
    kf->k = kf->p / (kf->p + kf->r);
    kf->x = kf->x + (kf->k * (z - kf->x)) / KALMAN_SCALE;
    kf->p = (kf->p * (KALMAN_SCALE - kf->k)) / KALMAN_SCALE;

    return (uint16_t)(kf->x / KALMAN_SCALE);
}

#endif
