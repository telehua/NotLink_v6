#include "bsp_usb.h"

#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_pwr.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_utils.h"

#include "bsp_tick.h"

static void usb_set_gpio_af_mode(GPIO_TypeDef *GPIOx, uint32_t Pin, uint32_t Alternate)
{
    if (Pin > LL_GPIO_PIN_7)
    {
        LL_GPIO_SetAFPin_8_15(GPIOx, Pin, Alternate);
        return;
    }

    LL_GPIO_SetAFPin_0_7(GPIOx, Pin, Alternate);
}

/**
 * @brief 初始化USB
 *
 */
void bsp_usb_init(void)
{
#if 0 // 仅VDD50USB输入时使用内部LDO
    LL_PWR_EnableUSBReg();
    while (LL_PWR_IsActiveFlag_USB() != 1)
    {
    }
#endif

    LL_RCC_SetUSBClockSource(LL_RCC_USB_CLKSOURCE_PLL1Q); // USB
    LL_PWR_EnableUSBVoltageDetector();

    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOA);
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOB);
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOC);

    /* USB RST */
    LL_GPIO_SetOutputPin(ULPI_RST_GPIO_Port, ULPI_RST_Pin);
    LL_GPIO_SetPinMode(ULPI_RST_GPIO_Port, ULPI_RST_Pin, LL_GPIO_MODE_OUTPUT);

    /* MCO2 USBPHY 24MHz */
    LL_GPIO_ResetOutputPin(MCO2_GPIO_Port, MCO2_Pin);
    LL_GPIO_SetPinSpeed(MCO2_GPIO_Port, MCO2_Pin, LL_GPIO_SPEED_FREQ_HIGH);
    usb_set_gpio_af_mode(MCO2_GPIO_Port, MCO2_Pin, LL_GPIO_AF_0);
    LL_GPIO_SetPinMode(MCO2_GPIO_Port, MCO2_Pin, LL_GPIO_MODE_ALTERNATE);

    usb_set_gpio_af_mode(ULPI_CLK_GPIO_Port, ULPI_CLK_Pin, LL_GPIO_AF_10);
    usb_set_gpio_af_mode(ULPI_NXT_GPIO_Port, ULPI_NXT_Pin, LL_GPIO_AF_10);
    usb_set_gpio_af_mode(ULPI_DIR_GPIO_Port, ULPI_DIR_Pin, LL_GPIO_AF_10);
    usb_set_gpio_af_mode(ULPI_STP_GPIO_Port, ULPI_STP_Pin, LL_GPIO_AF_10);
    usb_set_gpio_af_mode(ULPI_D7_GPIO_Port, ULPI_D7_Pin, LL_GPIO_AF_10);
    usb_set_gpio_af_mode(ULPI_D6_GPIO_Port, ULPI_D6_Pin, LL_GPIO_AF_10);
    usb_set_gpio_af_mode(ULPI_D5_GPIO_Port, ULPI_D5_Pin, LL_GPIO_AF_10);
    usb_set_gpio_af_mode(ULPI_D4_GPIO_Port, ULPI_D4_Pin, LL_GPIO_AF_10);
    usb_set_gpio_af_mode(ULPI_D3_GPIO_Port, ULPI_D3_Pin, LL_GPIO_AF_10);
    usb_set_gpio_af_mode(ULPI_D2_GPIO_Port, ULPI_D2_Pin, LL_GPIO_AF_10);
    usb_set_gpio_af_mode(ULPI_D1_GPIO_Port, ULPI_D1_Pin, LL_GPIO_AF_10);
    usb_set_gpio_af_mode(ULPI_D0_GPIO_Port, ULPI_D0_Pin, LL_GPIO_AF_10);

    LL_GPIO_SetPinOutputType(ULPI_CLK_GPIO_Port, ULPI_CLK_Pin, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinOutputType(ULPI_NXT_GPIO_Port, ULPI_NXT_Pin, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinOutputType(ULPI_DIR_GPIO_Port, ULPI_DIR_Pin, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinOutputType(ULPI_STP_GPIO_Port, ULPI_STP_Pin, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinOutputType(ULPI_D7_GPIO_Port, ULPI_D7_Pin, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinOutputType(ULPI_D6_GPIO_Port, ULPI_D6_Pin, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinOutputType(ULPI_D5_GPIO_Port, ULPI_D5_Pin, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinOutputType(ULPI_D4_GPIO_Port, ULPI_D4_Pin, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinOutputType(ULPI_D3_GPIO_Port, ULPI_D3_Pin, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinOutputType(ULPI_D2_GPIO_Port, ULPI_D2_Pin, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinOutputType(ULPI_D1_GPIO_Port, ULPI_D1_Pin, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinOutputType(ULPI_D0_GPIO_Port, ULPI_D0_Pin, LL_GPIO_OUTPUT_PUSHPULL);

    LL_GPIO_SetPinSpeed(ULPI_CLK_GPIO_Port, ULPI_CLK_Pin, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinSpeed(ULPI_NXT_GPIO_Port, ULPI_NXT_Pin, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinSpeed(ULPI_DIR_GPIO_Port, ULPI_DIR_Pin, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinSpeed(ULPI_STP_GPIO_Port, ULPI_STP_Pin, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinSpeed(ULPI_D7_GPIO_Port, ULPI_D7_Pin, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinSpeed(ULPI_D6_GPIO_Port, ULPI_D6_Pin, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinSpeed(ULPI_D5_GPIO_Port, ULPI_D5_Pin, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinSpeed(ULPI_D4_GPIO_Port, ULPI_D4_Pin, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinSpeed(ULPI_D3_GPIO_Port, ULPI_D3_Pin, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinSpeed(ULPI_D2_GPIO_Port, ULPI_D2_Pin, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinSpeed(ULPI_D1_GPIO_Port, ULPI_D1_Pin, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinSpeed(ULPI_D0_GPIO_Port, ULPI_D0_Pin, LL_GPIO_SPEED_FREQ_HIGH);

    LL_GPIO_SetPinMode(ULPI_CLK_GPIO_Port, ULPI_CLK_Pin, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinMode(ULPI_NXT_GPIO_Port, ULPI_NXT_Pin, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinMode(ULPI_DIR_GPIO_Port, ULPI_DIR_Pin, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinMode(ULPI_STP_GPIO_Port, ULPI_STP_Pin, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinMode(ULPI_D7_GPIO_Port, ULPI_D7_Pin, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinMode(ULPI_D6_GPIO_Port, ULPI_D6_Pin, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinMode(ULPI_D5_GPIO_Port, ULPI_D5_Pin, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinMode(ULPI_D4_GPIO_Port, ULPI_D4_Pin, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinMode(ULPI_D3_GPIO_Port, ULPI_D3_Pin, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinMode(ULPI_D2_GPIO_Port, ULPI_D2_Pin, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinMode(ULPI_D1_GPIO_Port, ULPI_D1_Pin, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinMode(ULPI_D0_GPIO_Port, ULPI_D0_Pin, LL_GPIO_MODE_ALTERNATE);

    LL_GPIO_ResetOutputPin(ULPI_RST_GPIO_Port, ULPI_RST_Pin);

    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_USB1OTGHSULPI); // 只有HS1可以走外部PHY
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_USB1OTGHS);

    NVIC_SetPriority(OTG_HS_IRQn, 10);
    NVIC_EnableIRQ(OTG_HS_IRQn);
}
