#ifndef __VCOM_H__
#define __VCOM_H__

#include <stdint.h>

#include "stm32h7xx_ll_gpio.h"

void bsp_vcom_init(void);
void bsp_vcom_start_tx_dma(uint8_t *buff, uint32_t size);
void bsp_vcom_start_rx_dma(uint8_t *buff, uint8_t *buff_next, uint32_t size);
uint32_t bsp_vcom_get_rx_dma_length(uint32_t size);
void bsp_vcom_set_dma_next(uint8_t *buff_next);
uint32_t bsp_vcom_is_rx_dma_enable(void);
void bsp_vcom_set_config(uint8_t *baud_rate, uint8_t *stop_bits, uint8_t *parity, uint8_t *data_bits);
void bsp_vcom_get_config(uint8_t *baud_rate, uint8_t *stop_bits, uint8_t *parity, uint8_t *data_bits);

#endif /* __VCOM_H__ */
