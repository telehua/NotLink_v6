#ifndef __CRC_H__
#define __CRC_H__

#include <stdint.h>

void bsp_crc_init(void);
void bsp_crc_deinit(void);
uint32_t bsp_crc_get_crc32(uint8_t *buff, uint32_t size);

#endif /* __CRC_H__ */
