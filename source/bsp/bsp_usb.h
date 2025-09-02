#ifndef __USB_H__
#define __USB_H__

#include <stdint.h>

#include "stm32h7xx_ll_gpio.h"

#define MCO2_GPIO_Port GPIOC
#define MCO2_Pin LL_GPIO_PIN_9

#define ULPI_RST_GPIO_Port GPIOA
#define ULPI_RST_Pin LL_GPIO_PIN_8

#define ULPI_CLK_GPIO_Port GPIOA
#define ULPI_CLK_Pin LL_GPIO_PIN_5

#define ULPI_NXT_GPIO_Port GPIOC
#define ULPI_NXT_Pin LL_GPIO_PIN_3

#define ULPI_DIR_GPIO_Port GPIOC
#define ULPI_DIR_Pin LL_GPIO_PIN_2

#define ULPI_STP_GPIO_Port GPIOC
#define ULPI_STP_Pin LL_GPIO_PIN_0

#define ULPI_D7_GPIO_Port GPIOB
#define ULPI_D7_Pin LL_GPIO_PIN_5

#define ULPI_D6_GPIO_Port GPIOB
#define ULPI_D6_Pin LL_GPIO_PIN_13

#define ULPI_D5_GPIO_Port GPIOB
#define ULPI_D5_Pin LL_GPIO_PIN_12

#define ULPI_D4_GPIO_Port GPIOB
#define ULPI_D4_Pin LL_GPIO_PIN_11

#define ULPI_D3_GPIO_Port GPIOB
#define ULPI_D3_Pin LL_GPIO_PIN_10

#define ULPI_D2_GPIO_Port GPIOB
#define ULPI_D2_Pin LL_GPIO_PIN_1

#define ULPI_D1_GPIO_Port GPIOB
#define ULPI_D1_Pin LL_GPIO_PIN_0

#define ULPI_D0_GPIO_Port GPIOA
#define ULPI_D0_Pin LL_GPIO_PIN_3

void bsp_usb_init(void);

#endif /* __USB_H__ */
