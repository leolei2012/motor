/** Includes ------------------------------------------------------------------ */
#include "bsp_isr.h"
/** Private includes ---------------------------------------------------------- */
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_tim.h"
#include "drv.h"

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
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  HAL_IncTick();
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
        drv_ain_sensor_tim_isr(g_drv.ain_sensor);
    }
}
