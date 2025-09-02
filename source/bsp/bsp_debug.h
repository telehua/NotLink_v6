#ifndef __DEBUG_H__
#define __DEBUG_H__

#include <stdint.h>

#include "stm32h7xx_ll_gpio.h"

#define USART1_TXD_GPIO_Port GPIOA
#define USART1_TXD_Pin LL_GPIO_PIN_9

#define USART1_RXD_GPIO_Port GPIOA
#define USART1_RXD_Pin LL_GPIO_PIN_10

void bsp_debug_init(void);
void bsp_debug_deinit(void);
void bsp_debug_send_byte_poll(uint8_t data);
int bsp_debug_get_byte(uint8_t *data);

#endif /* __DEBUG_H__ */
