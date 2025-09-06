#ifndef __LED_H__
#define __LED_H__

#include <stdint.h>

#include "stm32h7xx_ll_gpio.h"

#define LED0_GPIO_Port GPIOD
#define LED0_Pin LL_GPIO_PIN_14

#define LED1_GPIO_Port GPIOD
#define LED1_Pin LL_GPIO_PIN_13

enum BSP_LED
{
    LED_CONNECT = 0,
    LED_RUNNING,
};

void bsp_led_init(void);
void bsp_led_on(uint8_t index);
void bsp_led_off(uint8_t index);
void bsp_led_toggle(uint8_t index);

#endif /* __LED_H__ */
