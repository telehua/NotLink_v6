#include "bsp_debug.h"

#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_usart.h"

/**
 * @brief 初始化LED
 *
 */
void bsp_debug_init(void)
{
    LL_GPIO_InitTypeDef GPIO_InitStruct;
    LL_USART_InitTypeDef USART_InitStruct;
    LL_GPIO_StructInit(&GPIO_InitStruct);
    LL_USART_StructInit(&USART_InitStruct);

    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOA);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
    LL_USART_DeInit(USART1);

    GPIO_InitStruct.Pin = USART1_TXD_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
    LL_GPIO_Init(USART1_TXD_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = USART1_RXD_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
    LL_GPIO_Init(USART1_RXD_GPIO_Port, &GPIO_InitStruct);

    USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
    USART_InitStruct.BaudRate = 460800;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B; // 8N1
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
    LL_USART_Init(USART1, &USART_InitStruct);

    LL_USART_SetTXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_8_8);
    LL_USART_SetRXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
    LL_USART_EnableFIFO(USART1);
    LL_USART_ConfigAsyncMode(USART1);

    LL_USART_Enable(USART1);
    while (!(LL_USART_IsActiveFlag_TEACK(USART1)))
    {
    }
}

void bsp_debug_send_byte_poll(uint8_t data)
{
    while (LL_USART_IsActiveFlag_TXE_TXFNF(USART1) != 1)
    {
    }

    LL_USART_TransmitData8(USART1, data);
}

int bsp_debug_get_byte(uint8_t *data)
{
    if (LL_USART_IsActiveFlag_RXNE(USART1))
    {
        *data = LL_USART_ReceiveData8(USART1);
        return 0;
    }

    return -1;
}

void bsp_debug_deinit(void)
{
    // 等待发送缓冲区清空
    while (LL_USART_IsActiveFlag_TC(USART1) != 1)
    {
    }

    LL_USART_Disable(USART1);
    LL_USART_DeInit(USART1);
    LL_APB2_GRP1_DisableClock(LL_APB2_GRP1_PERIPH_USART1);
}
