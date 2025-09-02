#ifndef __SPI_H__
#define __SPI_H__

#include <stdint.h>

#include "stm32h7xx_ll_gpio.h"

/* 使用硬件SPI */
#define USE_HARD_SPI

/* SPI2 CLK PA12 */
#define SPI_CLK_GPIO_Port GPIOA
#define SPI_CLK_Pin LL_GPIO_PIN_12

/* SPI2 MOSI PB15 */
#define SPI_MOSI_GPIO_Port GPIOB
#define SPI_MOSI_Pin LL_GPIO_PIN_15

/* SPI2 MISO PB14 */
#define SPI_MISO_GPIO_Port GPIOB
#define SPI_MISO_Pin LL_GPIO_PIN_14

/* SPI2 CS0 PA11 */
#define SPI_CS0_GPIO_Port GPIOA
#define SPI_CS0_Pin LL_GPIO_PIN_11

void bsp_spi_init(void);
void bsp_spi_deinit(void);
void bsp_spi_set_cs(uint32_t index, uint8_t status);
uint8_t bsp_spi_transfer_byte(uint8_t data);

#endif /* __SPI_H__ */
