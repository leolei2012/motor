/** Includes ------------------------------------------------------------------ */
#include "bsp_isr.h"
/** Private includes ---------------------------------------------------------- */
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_usart.h"
#include "hal_usart2.h"
#include "drv.h"
#include "tx_api.h"

/* ThreadX 内核 tick 中断处理（定义于 tx_timer_interrupt.S） */
extern VOID _tx_timer_interrupt(VOID);

volatile uint32_t g_tim6_isr_count = 0;

/* ================================================================
 * 故障诊断捕获 (排查"跑飞"用)
 *
 * 异常进入时 CPU 已把 R0-R3/R12/LR/PC/xPSR 压入当前栈(MSP/PSP),
 * 这里按 EXC_RETURN(LR) bit2 选栈, 把帧 + 各故障状态寄存器存到全局,
 * 供 J-Link 在 while(1) 停留期间读取 (地址见 .map 文件)。
 * ================================================================ */
#define FAULT_SCB_CFSR   (*(volatile uint32_t *)0xE000ED28u)
#define FAULT_SCB_HFSR   (*(volatile uint32_t *)0xE000ED2Cu)
#define FAULT_SCB_MMFAR  (*(volatile uint32_t *)0xE000ED34u)
#define FAULT_SCB_BFAR   (*(volatile uint32_t *)0xE000ED38u)

#define FAULT_ID_NONE     0u
#define FAULT_ID_MM       1u
#define FAULT_ID_BUS      2u
#define FAULT_ID_USAGE    3u
#define FAULT_ID_HARD     4u

volatile uint32_t g_fault_id;
volatile uint32_t g_fault_cfsr;
volatile uint32_t g_fault_hfsr;
volatile uint32_t g_fault_mmfar;
volatile uint32_t g_fault_bfar;
volatile uint32_t g_fault_pc;
volatile uint32_t g_fault_lr;
volatile uint32_t g_fault_xpsr;
volatile uint32_t g_fault_sp;

static uint32_t *fault_get_frame(void)
{
    uint32_t *frame;
    __asm volatile
    (
        "TST LR, #4\n\t"
        "ITE EQ\n\t"
        "MRSEQ %0, MSP\n\t"
        "MRSNE %0, PSP\n\t"
        : "=r" (frame)
        :
        : "cc"
    );
    return frame;
}

static void fault_capture(uint32_t id)
{
    uint32_t *frame = fault_get_frame();

    g_fault_id    = id;
    g_fault_cfsr  = FAULT_SCB_CFSR;
    g_fault_hfsr  = FAULT_SCB_HFSR;
    g_fault_mmfar = FAULT_SCB_MMFAR;
    g_fault_bfar  = FAULT_SCB_BFAR;
    g_fault_pc    = frame[6];
    g_fault_lr    = frame[5];
    g_fault_xpsr  = frame[7];
    g_fault_sp    = (uint32_t)frame;

    __disable_irq();
    while (1)
    {
    }
}

/******************************************************************************/
/** Cortex-M4 Processor Interruption and Exception Handlers */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
   while (1)
  {
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  fault_capture(FAULT_ID_MM);
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  fault_capture(FAULT_ID_BUS);
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  fault_capture(FAULT_ID_USAGE);
}

/**
  * @brief This function handles hard fault.
  */
void HardFault_Handler(void)
{
  fault_capture(FAULT_ID_HARD);
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief System tick timer —— 归 ThreadX 内核使用。
  *
  * 转发给 ThreadX 的 _tx_timer_interrupt 驱动内核 tick（默认 10ms）。
  * HAL 的 HAL_IncTick 改由 TIM6 ISR 提供（见 TIM6_DAC_IRQHandler）。
  */
void SysTick_Handler(void)
{
    _tx_timer_interrupt();
}

/******************************************************************************/
/** STM32G4xx Peripheral Interrupt Handlers */
/******************************************************************************/

/**
  * @brief TIM6 / DAC 共享中断
  *
  * TIM6 提供固定 1kHz 的传感器 ADC 采样节拍，
  * 在 ISR 中完成软件触发 + 非阻塞读取 + EMA 滤波。
  */
void TIM6_DAC_IRQHandler(void)
{
    if (LL_TIM_IsActiveFlag_UPDATE(TIM6) != 0U)
    {
        g_tim6_isr_count++;
        LL_TIM_ClearFlag_UPDATE(TIM6);

        /* HAL tick 时间基准：SysTick 已归 ThreadX，改由 TIM6（1kHz）驱动 */
        HAL_IncTick();

        drv_ain_sensor_tim_isr(g_drv.ain_sensor);
    }
}

/**
  * @brief ADC1/ADC2 共享中断
  *
  * 注入组 JEOS（TIM1 触发，三相电流采样完成）：
  * 清标志后驱动电机电流环（mcl_control_tick）。
  */
void ADC1_2_IRQHandler(void)
{
    if (LL_ADC_IsActiveFlag_JEOS(ADC1) != 0U)
    {
        LL_ADC_ClearFlag_JEOS(ADC1);

        /* 电流环控制节拍：mcl FOC 算法 */
        if (g_drv.motor != NULL)
        {
            drv_motor_control_isr(g_drv.motor);
        }
    }
    if (LL_ADC_IsActiveFlag_JEOS(ADC2) != 0U)
    {
        LL_ADC_ClearFlag_JEOS(ADC2);
    }
}

/**
  * @brief TIM1 UP / TIM16 共享中断
  *
  * TIM1 更新中断（每个 PWM 周期）。当前仅清标志，
  * ADC JSQR 扇区切换逻辑待后续移植电机驱动层时补充。
  */
void TIM1_UP_TIM16_IRQHandler(void)
{
    if (LL_TIM_IsActiveFlag_UPDATE(TIM1) != 0U)
    {
        LL_TIM_ClearFlag_UPDATE(TIM1);
    }
}

/**
  * @brief TIM1 BRK / TIM15 共享中断
  */
void TIM1_BRK_TIM15_IRQHandler(void)
{
    if (LL_TIM_IsActiveFlag_BRK(TIM1) != 0U)
    {
        LL_TIM_ClearFlag_BRK(TIM1);
    }
}

/**
  * @brief TIM7 / DAC 共享中断
  *
  * TIM7 空闲超时定时器：驱动 uart_control 的帧接收完成判定（1ms 节拍）。
  */
void TIM7_DAC_IRQHandler(void)
{
    if (LL_TIM_IsActiveFlag_UPDATE(TIM7) != 0U)
    {
        LL_TIM_ClearFlag_UPDATE(TIM7);

        if (g_drv.uart != NULL)
        {
            uart_control_timer_isr(&g_drv.uart->uart);
        }
    }
}

/**
  * @brief USART2 中断（RXNE 逐字节接收 + ORE 处理）
  */
void USART2_IRQHandler(void)
{
    hal_usart2_irq_handler();
}

/**
  * @brief DMA1 CH2 中断（USART2_TX 发送完成）
  */
void DMA1_Channel2_IRQHandler(void)
{
    hal_usart2_dma_tx_isr();

    if (g_drv.uart != NULL)
    {
        uart_control_tx_done_isr(&g_drv.uart->uart);
    }
}
