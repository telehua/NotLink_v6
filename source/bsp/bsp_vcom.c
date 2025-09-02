#include "bsp_vcom.h"

#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_dma.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_usart.h"

static struct _bsp_vcom_config
{
    uint32_t baud_rate;
    uint8_t stop_bits;
    uint8_t parity;
    uint8_t data_bits;
} bsp_vcom_config;

/**
 * @brief 初始化LED
 *
 */
void bsp_vcom_init(void)
{
    bsp_vcom_config.baud_rate = 115200;
    bsp_vcom_config.stop_bits = 0;
    bsp_vcom_config.parity = 0;
    bsp_vcom_config.data_bits = 8;

    LL_USART_InitTypeDef USART_InitStruct;
    LL_USART_StructInit(&USART_InitStruct);

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART4);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

    LL_USART_DeInit(UART4);
    USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
    USART_InitStruct.BaudRate = 115200;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B; // 8N1
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
    LL_USART_Init(UART4, &USART_InitStruct);

    LL_USART_SetTXFIFOThreshold(UART4, LL_USART_FIFOTHRESHOLD_1_8);
    LL_USART_SetRXFIFOThreshold(UART4, LL_USART_FIFOTHRESHOLD_1_8);
    LL_USART_EnableFIFO(UART4);
    LL_USART_ConfigAsyncMode(UART4); // 异步模式
    LL_USART_DisableDMAReq_RX(UART4);
    LL_USART_DisableDMAReq_TX(UART4);

    /* UART4_TX Init */
    LL_DMA_DeInit(DMA1, LL_DMA_STREAM_0);
    LL_DMA_SetPeriphRequest(DMA1, LL_DMA_STREAM_0, LL_DMAMUX1_REQ_UART4_TX);
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_0, LL_DMA_DIRECTION_MEMORY_TO_PERIPH); // 内存到外设
    LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_0, LL_DMA_PRIORITY_LOW);
    LL_DMA_SetMode(DMA1, LL_DMA_STREAM_0, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_0, LL_DMA_PERIPH_NOINCREMENT); // 外设不递增
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_0, LL_DMA_MEMORY_INCREMENT);   // 内存递增
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_0, LL_DMA_PDATAALIGN_BYTE);       // 字节传输
    LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_0, LL_DMA_MDATAALIGN_BYTE);       // 字节传输
    LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_0);
    LL_DMA_ClearFlag_TC0(DMA1);
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_0);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_0,
                            LL_USART_DMA_GetRegAddr(UART4, LL_USART_DMA_REG_DATA_TRANSMIT));

    /* UART4_RX Init */
    LL_DMA_DeInit(DMA1, LL_DMA_STREAM_1);
    LL_DMA_SetPeriphRequest(DMA1, LL_DMA_STREAM_1, LL_DMAMUX1_REQ_UART4_RX);
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_1, LL_DMA_PRIORITY_LOW);
    LL_DMA_SetMode(DMA1, LL_DMA_STREAM_1, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_1, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_1, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_1, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_1, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_1);
    LL_DMA_ClearFlag_TC1(DMA1);
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_1);
    LL_DMA_EnableDoubleBufferMode(DMA1, LL_DMA_STREAM_1); // 双缓冲模式
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_1,
                            LL_USART_DMA_GetRegAddr(UART4, LL_USART_DMA_REG_DATA_RECEIVE));

    // DMA请求
    LL_USART_EnableDMAReq_TX(UART4);
    LL_USART_EnableDMAReq_RX(UART4);

    LL_USART_Enable(UART4);
    while (!(LL_USART_IsActiveFlag_TEACK(UART4)))
    {
    }

    // 空闲中断
    LL_USART_ClearFlag_IDLE(UART4);
    LL_USART_EnableIT_IDLE(UART4);

    // TX
    NVIC_SetPriority(DMA1_Stream0_IRQn, 10);
    NVIC_EnableIRQ(DMA1_Stream0_IRQn);

    // RX
    NVIC_SetPriority(DMA1_Stream1_IRQn, 10);
    NVIC_EnableIRQ(DMA1_Stream1_IRQn);

    // UART4
    NVIC_SetPriority(UART4_IRQn, 10);
    NVIC_EnableIRQ(UART4_IRQn);
}

/**
 * @brief 设置串口配置
 *
 * @param baud_rate
 * @param stop_bits
 * @param parity
 * @param data_bits
 */
void bsp_vcom_set_config(uint8_t *baud_rate, uint8_t *stop_bits, uint8_t *parity, uint8_t *data_bits)
{
    /*******************************************************************************/
    /* Line Coding Structure */
    /*-----------------------------------------------------------------------------*/
    /* Offset | Field       | Size | Value  | Description */
    /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
    /* 4      | bCharFormat |   1  | Number | Stop bits */
    /*                                        0 - 1 Stop bit */
    /*                                        1 - 1.5 Stop bits */
    /*                                        2 - 2 Stop bits */
    /* 5      | bParityType |  1   | Number | Parity */
    /*                                        0 - None */
    /*                                        1 - Odd */
    /*                                        2 - Even */
    /*                                        3 - Mark */
    /*                                        4 - Space */
    /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).*/
    /*******************************************************************************/

    LL_USART_InitTypeDef USART_InitStruct;
    LL_USART_StructInit(&USART_InitStruct);

    LL_USART_Disable(UART4);

    LL_USART_DeInit(UART4);
    USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;

    uint32_t baud;
    ((uint8_t *)&baud)[0] = baud_rate[0];
    ((uint8_t *)&baud)[1] = baud_rate[1];
    ((uint8_t *)&baud)[2] = baud_rate[2];
    ((uint8_t *)&baud)[3] = baud_rate[3];

    USART_InitStruct.BaudRate = baud;
    bsp_vcom_config.baud_rate = baud;

    switch (*stop_bits)
    {
    case 0:
        bsp_vcom_config.stop_bits = 0;
        USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
        break;
    case 1:
        bsp_vcom_config.stop_bits = 1;
        USART_InitStruct.StopBits = LL_USART_STOPBITS_1_5;
        break;
    case 2:
        bsp_vcom_config.stop_bits = 2;
        USART_InitStruct.StopBits = LL_USART_STOPBITS_2;
        break;
    default:
        bsp_vcom_config.stop_bits = 0;
        USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
        break;
    }

    switch (*parity)
    {
    case 0:
        bsp_vcom_config.parity = 0;
        USART_InitStruct.Parity = LL_USART_PARITY_NONE;
        break;
    case 1:
        bsp_vcom_config.parity = 1;
        USART_InitStruct.Parity = LL_USART_PARITY_ODD;
        break;
    case 2:
        bsp_vcom_config.parity = 2;
        USART_InitStruct.Parity = LL_USART_PARITY_EVEN;
        break;
    default:
        bsp_vcom_config.parity = 0;
        USART_InitStruct.Parity = LL_USART_PARITY_NONE;
        break;
    }

    switch (*data_bits)
    {
    case 7:
        bsp_vcom_config.data_bits = 7;
        USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_7B;
        break;
    case 8:
        bsp_vcom_config.data_bits = 8;
        USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
        break;
    case 9:
        bsp_vcom_config.data_bits = 9;
        USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_9B;
        break;
    default:
        bsp_vcom_config.data_bits = 8;
        USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
        break;
    }

    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
    LL_USART_Init(UART4, &USART_InitStruct);

    LL_USART_SetTXFIFOThreshold(UART4, LL_USART_FIFOTHRESHOLD_1_8);
    LL_USART_SetRXFIFOThreshold(UART4, LL_USART_FIFOTHRESHOLD_1_8);
    LL_USART_EnableFIFO(UART4);
    LL_USART_ConfigAsyncMode(UART4); // 异步模式

    // DMA请求
    LL_USART_EnableDMAReq_TX(UART4);
    LL_USART_EnableDMAReq_RX(UART4);

    LL_USART_Enable(UART4);
    while (!(LL_USART_IsActiveFlag_TEACK(UART4)))
    {
    }

    // 空闲中断
    LL_USART_ClearFlag_IDLE(UART4);
    LL_USART_EnableIT_IDLE(UART4);
}

/**
 * @brief 获取串口配置
 *
 * @param baud_rate
 * @param stop_bits
 * @param parity
 * @param data_bits
 */
void bsp_vcom_get_config(uint8_t *baud_rate, uint8_t *stop_bits, uint8_t *parity, uint8_t *data_bits)
{
    baud_rate[0] = (uint8_t)(bsp_vcom_config.baud_rate);
    baud_rate[1] = (uint8_t)(bsp_vcom_config.baud_rate >> 8);
    baud_rate[2] = (uint8_t)(bsp_vcom_config.baud_rate >> 16);
    baud_rate[3] = (uint8_t)(bsp_vcom_config.baud_rate >> 24);
    *stop_bits = bsp_vcom_config.stop_bits;
    *parity = bsp_vcom_config.parity;
    *data_bits = bsp_vcom_config.data_bits;
}

/**
 * @brief 开始发送
 *
 * @param buff
 * @param size
 */
void bsp_vcom_start_tx_dma(uint8_t *buff, uint32_t size)
{
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_0, (uint32_t)buff);
    LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_0, size);
    LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_0);
}

/**
 * @brief 开始接收
 *
 * @param buff
 * @param size
 */
void bsp_vcom_start_rx_dma(uint8_t *buff, uint8_t *buff_next, uint32_t size)
{
    if (LL_DMA_GetCurrentTargetMem(DMA1, LL_DMA_STREAM_1) == LL_DMA_CURRENTTARGETMEM0)
    {
        LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_1, (uint32_t)buff);
        LL_DMA_SetMemory1Address(DMA1, LL_DMA_STREAM_1, (uint32_t)buff_next);
    }
    else
    {
        LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_1, (uint32_t)buff_next);
        LL_DMA_SetMemory1Address(DMA1, LL_DMA_STREAM_1, (uint32_t)buff);
    }

    LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_1, size);
    LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_1);
}

/**
 * @brief 更新下一个缓冲区地址
 *
 * @param buff_next
 */
void bsp_vcom_set_dma_next(uint8_t *buff_next)
{
    if (LL_DMA_GetCurrentTargetMem(DMA1, LL_DMA_STREAM_1) == LL_DMA_CURRENTTARGETMEM0)
    {
        LL_DMA_SetMemory1Address(DMA1, LL_DMA_STREAM_1, (uint32_t)buff_next);
    }
    else
    {
        LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_1, (uint32_t)buff_next);
    }
}

/**
 * @brief 获取已传输的字节数
 *
 * @param size
 * @return uint32_t
 */
uint32_t bsp_vcom_get_rx_dma_length(uint32_t size)
{
    return (size - (0xFFFF & LL_DMA_GetDataLength(DMA1, LL_DMA_STREAM_1)));
}

/**
 * @brief DMA是否在运行
 *
 * @return uint32_t
 */
uint32_t bsp_vcom_is_rx_dma_enable(void)
{
    return LL_DMA_IsEnabledStream(DMA1, LL_DMA_STREAM_1);
}
