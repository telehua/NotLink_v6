#include "bsp_led.h"

#include "stm32h7xx_ll_bus.h"

/**
 * @brief 初始化LED
 *
 */
void bsp_led_init(void)
{
    LL_GPIO_InitTypeDef GPIO_InitStruct;
    LL_GPIO_StructInit(&GPIO_InitStruct);

    // LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOC);
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOD);

    // LL_GPIO_SetOutputPin(LED0_GPIO_Port, LED0_Pin);
    // LL_GPIO_SetOutputPin(LED1_GPIO_Port, LED1_Pin);
    LL_GPIO_ResetOutputPin(LED0_GPIO_Port, LED0_Pin);
    LL_GPIO_ResetOutputPin(LED1_GPIO_Port, LED1_Pin);

    GPIO_InitStruct.Pin = LED1_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LED0_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(LED0_GPIO_Port, &GPIO_InitStruct);
}

/**
 * @brief 点亮LED
 *
 * @param index LED_RUNNING
 *              LED_CONNECT
 */
void bsp_led_set(uint8_t index)
{
    switch (index)
    {
    case LED_RUNNING:
        LL_GPIO_SetOutputPin(LED0_GPIO_Port, LED0_Pin);
        break;
    case LED_CONNECT:
        LL_GPIO_SetOutputPin(LED1_GPIO_Port, LED1_Pin);
        break;

    default:
        break;
    }
}

/**
 * @brief 翻转LED
 *
 * @param index LED_RUNNING
 *              LED_CONNECT
 */
void bsp_led_toggle(uint8_t index)
{
    switch (index)
    {
    case LED_RUNNING:
        LL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
        break;
    case LED_CONNECT:
        LL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
        break;

    default:
        break;
    }
}

/**
 * @brief 熄灭LED
 *
 * @param index LED_RUNNING
 *              LED_CONNECT
 */
void bsp_led_reset(uint8_t index)
{
    switch (index)
    {
    case LED_RUNNING:
        LL_GPIO_ResetOutputPin(LED0_GPIO_Port, LED0_Pin);
        break;
    case LED_CONNECT:
        LL_GPIO_ResetOutputPin(LED1_GPIO_Port, LED1_Pin);
        break;

    default:
        break;
    }
}
