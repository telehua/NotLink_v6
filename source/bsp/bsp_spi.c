#include "bsp_spi.h"

#include "stm32h7xx_ll_bus.h"

#ifdef USE_HARD_SPI
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_spi.h"
#endif

/**
 * @brief 初始化
 *
 */
void bsp_spi_init(void)
{
    LL_GPIO_InitTypeDef GPIO_InitStruct;
    LL_GPIO_StructInit(&GPIO_InitStruct);

#ifdef USE_HARD_SPI
    LL_SPI_InitTypeDef SPI_InitStruct;
    LL_SPI_StructInit(&SPI_InitStruct);
#endif

    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOA);
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOB);

    LL_GPIO_SetOutputPin(SPI_CS0_GPIO_Port, SPI_CS0_Pin);   // 拉高片选
    LL_GPIO_ResetOutputPin(SPI_CLK_GPIO_Port, SPI_CLK_Pin); // 复位时钟

    GPIO_InitStruct.Pin = SPI_CS0_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(SPI_CS0_GPIO_Port, &GPIO_InitStruct);

    /* SPI其他引脚 */
    GPIO_InitStruct.Pin = SPI_CLK_Pin;
#ifdef USE_HARD_SPI
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
#else
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
#endif
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(SPI_CLK_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SPI_MOSI_Pin;
#ifdef USE_HARD_SPI
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
#else
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
#endif
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(SPI_MOSI_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SPI_MISO_Pin;
#ifdef USE_HARD_SPI
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
#else
    GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
#endif
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    LL_GPIO_Init(SPI_MISO_GPIO_Port, &GPIO_InitStruct);

#ifdef USE_HARD_SPI
    LL_RCC_SetSPIClockSource(LL_RCC_SPI123_CLKSOURCE_PLL1Q); // 120Mhz
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);
    LL_SPI_DeInit(SPI2);

    SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
    SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
    SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
    SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
    SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
    SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
    SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV2; // 120/2=60MHz
    SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
    SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
    SPI_InitStruct.CRCPoly = 0x0;
    LL_SPI_Init(SPI2, &SPI_InitStruct);
    LL_SPI_SetStandard(SPI2, LL_SPI_PROTOCOL_MOTOROLA);
    LL_SPI_SetFIFOThreshold(SPI2, LL_SPI_FIFO_TH_01DATA);
    LL_SPI_DisableNSSPulseMgt(SPI2);
    LL_SPI_EnableGPIOControl(SPI2);

    LL_SPI_Enable(SPI2);              // 使能
    LL_SPI_StartMasterTransfer(SPI2); // 启动传输
#endif
}

/**
 * @brief 取消初始化，所有引脚进入高阻状态
 *
 */
void bsp_spi_deinit(void)
{
    LL_GPIO_SetPinMode(SPI_CLK_GPIO_Port, SPI_CLK_Pin, LL_GPIO_MODE_INPUT);
    LL_GPIO_SetPinMode(SPI_MOSI_GPIO_Port, SPI_MOSI_Pin, LL_GPIO_MODE_INPUT);
    LL_GPIO_SetPinMode(SPI_MISO_GPIO_Port, SPI_MISO_Pin, LL_GPIO_MODE_INPUT);
    LL_GPIO_SetPinMode(SPI_CS0_GPIO_Port, SPI_CS0_Pin, LL_GPIO_MODE_INPUT);

    LL_SPI_Disable(SPI2);
    LL_APB1_GRP1_DisableClock(LL_APB1_GRP1_PERIPH_SPI2);
}

/**
 * @brief 片选
 *
 * @param index     序号
 * @param status    1选中
 */
void bsp_spi_set_cs(uint32_t index, uint8_t status)
{
    switch (index)
    {
    case 0:
        (status > 0) ? LL_GPIO_ResetOutputPin(SPI_CS0_GPIO_Port, SPI_CS0_Pin) : //
            LL_GPIO_SetOutputPin(SPI_CS0_GPIO_Port, SPI_CS0_Pin);
        break;

    default:
        /* 参数错误拉高所有片选 */
        LL_GPIO_SetOutputPin(SPI_CS0_GPIO_Port, SPI_CS0_Pin);
        break;
    }
}

uint8_t bsp_spi_transfer_byte(uint8_t data)
{
#ifdef USE_HARD_SPI
    LL_SPI_TransmitData8(SPI2, data);
    while (LL_SPI_IsActiveFlag_RXP(SPI2) != 1)
    {
    }
    data = LL_SPI_ReceiveData8(SPI2);
#else
    for (uint32_t i = 0; i < 8; i++)
    {
        (data & 0x80) ? LL_GPIO_SetOutputPin(SPI_MOSI_GPIO_Port, SPI_MOSI_Pin) : //
            LL_GPIO_ResetOutputPin(SPI_MOSI_GPIO_Port, SPI_MOSI_Pin);
        LL_GPIO_SetOutputPin(SPI_CLK_GPIO_Port, SPI_CLK_Pin); // 上升沿
        data = data << 1;
        data = data | ((LL_GPIO_ReadInputPort(SPI_MISO_GPIO_Port) & SPI_MISO_Pin) ? 0x01 : 0x00);
        LL_GPIO_ResetOutputPin(SPI_CLK_GPIO_Port, SPI_CLK_Pin); // 下降沿
    }
#endif

    return data;
}
