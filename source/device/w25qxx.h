#ifndef __W25QXX_H__
#define __W25QXX_H__

#include <stdint.h>

#define W25QXX_PAGE_SIZE 256U

#define W25QXX_CMD_WRITE_ENABLE 0x06    /* 写使能 */
#define W25QXX_CMD_WRITE_DISABLE 0x04   /*  */
#define W25QXX_CMD_FAST_READ 0x0B       /* 快速读 */
#define W25QXX_CMD_SECTOR_ERASE_4K 0x20 /* 块擦除 */
#define W25QXX_CMD_BLOCK_ERASE_32K 0x52 /* 扇区擦除 */
#define W25QXX_CMD_BLOCK_ERASE_64K 0xD8 /* 扇区擦除 */
#define W25QXX_CMD_CHIP_ERASE 0xC7      /* 片擦除 */
#define W25QXX_CMD_READ_SR1 0x05        /* 读SR1 */
#define W25QXX_CMD_PAGE_PROGRAM 0x02    /* 页编程 */
#define W25QXX_CMD_READ_JEDEC_ID 0x9F   /* 读JEDEC */

void w25_init(void);
void w25_deinit(void);
uint8_t w25_read_sr1(void);
void w25_enable_write_op(void);
void w25_disable_write_op(void);

uint8_t w25_read_byte(uint32_t flash_addr);
uint32_t w25_read_word(uint32_t flash_addr);
void w25_read_mass(uint32_t flash_addr, uint8_t *data_buff, uint32_t size);

void w25_write_byte(uint32_t flash_addr, uint8_t data);
void w25_write_mass(uint32_t flash_addr, uint8_t *data_buff, uint32_t size);

void w25_erase_sector_4k(uint32_t flash_addr);
void w25_erase_sector_64k(uint32_t flash_addr);
void w25_erase_chip(void);

uint32_t w25_read_jedec(void);

#endif /* __W25QXX_H__ */
