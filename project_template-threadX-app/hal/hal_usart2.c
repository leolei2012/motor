#include "hal_usart2.h"

#include "bsp_config.h"

void hal_usart2_init(void)
{
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    LL_USART_InitTypeDef USART_InitStruct = {0};

    LL_RCC_SetUSARTClockSource(LL_RCC_USART2_CLKSOURCE_PCLK1);

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOD);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMAMUX1);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

    GPIO_InitStruct.Pin = UART_TX_PIN;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
    LL_GPIO_Init(UART_TX_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = UART_RX_PIN;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
    LL_GPIO_Init(UART_RX_GPIO_PORT, &GPIO_InitStruct);

    /** USART2_TX DMA (CH2) */
    LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_2, LL_DMAMUX_REQ_USART2_TX);
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PRIORITY_LOW);
    LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MDATAALIGN_BYTE);

    /** USER CODE BEGIN USART2_Init 1 */

    /** USER CODE END USART2_Init 1 */
    USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
    USART_InitStruct.BaudRate = 9600;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
    LL_USART_Init(USART2, &USART_InitStruct);
    LL_USART_SetTXFIFOThreshold(USART2, LL_USART_FIFOTHRESHOLD_1_8);
    LL_USART_SetRXFIFOThreshold(USART2, LL_USART_FIFOTHRESHOLD_1_8);
    LL_USART_DisableFIFO(USART2);
    LL_USART_ConfigAsyncMode(USART2);

    /** USER CODE BEGIN WKUPType USART2 */

    /** USER CODE END WKUPType USART2 */

    LL_USART_Enable(USART2);

    /** Polling USART2 initialisation */
    while ((!(LL_USART_IsActiveFlag_TEACK(USART2))) || (!(LL_USART_IsActiveFlag_REACK(USART2))))
    {
    }
}

/** ============================================================
   DMA 发送
   ============================================================ */
void hal_usart2_dma_tx_start(const uint8_t *buf, uint16_t len)
{
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_2, (uint32_t)buf);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_2, (uint32_t)&USART2->TDR);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, len);
    LL_DMA_ClearFlag_TC2(DMA1);
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_2);
    LL_USART_EnableDMAReq_TX(USART2);
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);
}

void hal_usart2_dma_tx_isr(void)
{
    LL_DMA_ClearFlag_TC2(DMA1);
    LL_DMA_DisableIT_TC(DMA1, LL_DMA_CHANNEL_2);
    LL_USART_DisableDMAReq_TX(USART2);
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
}

/** ============================================================
   中断接收（逐字节）
   ============================================================ */
void hal_usart2_enable_it_rxne(void)
{
    LL_USART_EnableIT_RXNE(USART2);
}

void hal_usart2_disable_it_rxne(void)
{
    LL_USART_DisableIT_RXNE(USART2);
}

uint8_t hal_usart2_read_byte(void)
{
    return (uint8_t)LL_USART_ReceiveData8(USART2);
}

/* ============================================================
   uart_control 硬件 ops
   ============================================================ */

static struct uart_control *s_uart_instance;

/** 发送 ops：DMA 启动发送（成功启动返回 OK） */
static uint8_t usart2_ops_send(const uint8_t *buf, uint16_t len)
{
    if ((buf == NULL) || (len == 0u))
    {
        return UART_CONTROL_ERROR;
    }

    hal_usart2_dma_tx_start(buf, len);
    return UART_CONTROL_OK;
}

/** 启动逐字节接收：使能 RXNE 中断 */
static void usart2_ops_rx_start(void)
{
    hal_usart2_enable_it_rxne();
}

/** 停止接收：关闭 RXNE 中断 */
static void usart2_ops_rx_stop(void)
{
    hal_usart2_disable_it_rxne();
}

/** 读一个接收字节 */
static uint8_t usart2_ops_read_byte(void)
{
    return hal_usart2_read_byte();
}

static const struct uart_control_hal_ops s_usart2_ops =
{
    .send      = usart2_ops_send,
    .rx_start  = usart2_ops_rx_start,
    .rx_stop   = usart2_ops_rx_stop,
    .read_byte = usart2_ops_read_byte,
};

const struct uart_control_hal_ops *hal_usart2_get_ops(void)
{
    return &s_usart2_ops;
}

void hal_usart2_set_instance(struct uart_control *uart)
{
    s_uart_instance = uart;
}

/** USART2 中断：RXNE 逐字节接收 + ORE 处理 */
void hal_usart2_irq_handler(void)
{
    if (s_uart_instance == NULL)
    {
        return;
    }

    /* 处理上溢错误：ORE 未清除会阻止后续 RXNE 中断 */
    if (LL_USART_IsActiveFlag_ORE(USART2) != 0U)
    {
        LL_USART_ClearFlag_ORE(USART2);
    }

    if (LL_USART_IsActiveFlag_RXNE(USART2) != 0U)
    {
        uart_control_rx_isr(s_uart_instance);
    }
}
