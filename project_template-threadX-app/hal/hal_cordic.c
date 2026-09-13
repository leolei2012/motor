#include "hal_cordic.h"
#include "bsp_config.h"

/**
 * @brief  CORDIC COSINE 配置: 16-bit I/O, 6 周期精度, NBWRITE_1/NBREAD_1
 *
 * 写入: 32-bit 打包 (modulus 高 16 位 | angle 低 16 位)
 * 读取: 32-bit 打包 (cos 高 16 位 | sin 低 16 位)
 */
#define HAL_CORDIC_CSR_COSINE   (LL_CORDIC_FUNCTION_COSINE | LL_CORDIC_PRECISION_6CYCLES | LL_CORDIC_SCALE_0 |\
         LL_CORDIC_NBWRITE_1 | LL_CORDIC_NBREAD_1 |\
         LL_CORDIC_INSIZE_16BITS | LL_CORDIC_OUTSIZE_16BITS)

/** @brief  CORDIC SQRT 配置: 32-bit I/O, 6 周期精度, SCALE_1 */
#define HAL_CORDIC_CSR_SQRT     (LL_CORDIC_FUNCTION_SQUAREROOT | LL_CORDIC_PRECISION_6CYCLES | LL_CORDIC_SCALE_1 |\
         LL_CORDIC_NBWRITE_1 | LL_CORDIC_NBREAD_1 |\
         LL_CORDIC_INSIZE_32BITS | LL_CORDIC_OUTSIZE_32BITS)

/** @brief  CORDIC MODULUS 配置: 16-bit I/O, 6 周期精度 */
#define HAL_CORDIC_CSR_MODULUS  (LL_CORDIC_FUNCTION_MODULUS | LL_CORDIC_PRECISION_6CYCLES | LL_CORDIC_SCALE_0 |\
         LL_CORDIC_NBWRITE_1 | LL_CORDIC_NBREAD_1 |\
         LL_CORDIC_INSIZE_16BITS | LL_CORDIC_OUTSIZE_16BITS)

/** @brief  CORDIC PHASE 配置: 32-bit I/O, NBWRITE_2/NBREAD_1, 6 周期精度 */
#define HAL_CORDIC_CSR_PHASE    (LL_CORDIC_FUNCTION_PHASE | LL_CORDIC_PRECISION_6CYCLES | LL_CORDIC_SCALE_0 |\
         LL_CORDIC_NBWRITE_2 | LL_CORDIC_NBREAD_1 |\
         LL_CORDIC_INSIZE_32BITS | LL_CORDIC_OUTSIZE_32BITS)

/** ============================================================
   初始化
   ============================================================ */
void hal_cordic_init(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CORDIC);
}

/** ============================================================
   sincos: Cosine + Sine (Q1.15)
   ============================================================ */
void hal_cordic_sincos(int16_t angle, int16_t *p_cos, int16_t *p_sin)
{
    union {
        uint32_t rdata;
        struct {
            int16_t sin;
            int16_t cos;
        } comp;
    } result;

    WRITE_REG(CORDIC->CSR, HAL_CORDIC_CSR_COSINE);

    /** 高 16 位: modulus = 0x7FFF (Q1.15 = 1.0), 低 16 位: angle */
    LL_CORDIC_WriteData(CORDIC, ((uint32_t)0x7FFF0000) + ((uint32_t)angle));

    result.rdata = LL_CORDIC_ReadData(CORDIC);

    *p_cos = result.comp.cos;
    *p_sin = result.comp.sin;
}

/** ============================================================
   sqrt: 平方根 (Q1.31)
   ============================================================ */
int32_t hal_cordic_sqrt(int32_t x)
{
    int32_t result;

    if (x > 0)
    {
        uint32_t retVal;

        __disable_irq();
        WRITE_REG(CORDIC->CSR, HAL_CORDIC_CSR_SQRT);
        LL_CORDIC_WriteData(CORDIC, (uint32_t)x);
        retVal = LL_CORDIC_ReadData(CORDIC) >> 15;
        result = (int32_t)retVal;
        __enable_irq();
    }
    else
    {
        result = 0;
    }

    return result;
}

/** ============================================================
   modulus: sqrt(alpha^2 + beta^2) (Q1.15)
   ============================================================ */
int16_t hal_cordic_modulus(int16_t alpha, int16_t beta)
{
    uint32_t temp_val;

    __disable_irq();
    WRITE_REG(CORDIC->CSR, HAL_CORDIC_CSR_MODULUS);

    /** 高 16 位: beta, 低 16 位: alpha */
    LL_CORDIC_WriteData(CORDIC, (((uint32_t)beta << 16U) | ((uint32_t)alpha & 0x0000FFFFU)));

    /* 防跑飞: CORDIC 结果就绪标志偶发不置位会死循环, 加超时 */
    uint32_t rrdy_timeout = 1000u;
    while (LL_CORDIC_IsActiveFlag_RRDY(CORDIC) == 0U && rrdy_timeout > 0u)
    {
        rrdy_timeout--;
    }

    temp_val = (LL_CORDIC_ReadData(CORDIC) << 16U) >> 16U;
    __enable_irq();

    return (int16_t)temp_val;
}

/** ============================================================
   phase: atan2(beta, alpha) → 电角度 (Q1.15)
   ============================================================ */
int16_t hal_cordic_phase(int32_t alpha, int32_t beta)
{
    uint32_t result;

    WRITE_REG(CORDIC->CSR, HAL_CORDIC_CSR_PHASE);
    LL_CORDIC_WriteData(CORDIC, (uint32_t)alpha);
    LL_CORDIC_WriteData(CORDIC, (uint32_t)beta);
    result = LL_CORDIC_ReadData(CORDIC) >> 16U;

    return (int16_t)result;
}