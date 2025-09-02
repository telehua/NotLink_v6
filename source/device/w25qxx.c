#include "w25qxx.h"

#include "bsp_spi.h"

/**
 * @brief 初始化
 *
 */
void w25_init(void)
{
    bsp_spi_set_cs(0, 0);
}

/**
 * @brief 取消初始化
 *
 */
void w25_deinit(void)
{
    bsp_spi_set_cs(0, 0);
}

/**
 * @brief 读SR1
 *
 * @return uint8_t
 */
uint8_t w25_read_sr1(void)
{
    bsp_spi_set_cs(0, 1);
    bsp_spi_transfer_byte(W25QXX_CMD_READ_SR1);
    uint8_t r = bsp_spi_transfer_byte(0xFF);
    bsp_spi_set_cs(0, 0);
    return r;
}

/**
 * @brief 允许写
 *
 */
void w25_enable_write_op(void)
{
    bsp_spi_set_cs(0, 1);
    bsp_spi_transfer_byte(W25QXX_CMD_WRITE_ENABLE);
    bsp_spi_set_cs(0, 0);
}

/**
 * @brief 不允许写
 *
 */
void w25_disable_write_op(void)
{
    bsp_spi_set_cs(0, 1);
    bsp_spi_transfer_byte(W25QXX_CMD_WRITE_DISABLE);
    bsp_spi_set_cs(0, 0);
}

/**
 * @brief 读byte
 *
 * @param flash_addr    地址
 * @return uint8_t      数据
 */
uint8_t w25_read_byte(uint32_t flash_addr)
{
    bsp_spi_set_cs(0, 1);
    bsp_spi_transfer_byte(W25QXX_CMD_FAST_READ);
    bsp_spi_transfer_byte(flash_addr >> 16);
    bsp_spi_transfer_byte(flash_addr >> 8);
    bsp_spi_transfer_byte(flash_addr);
    bsp_spi_transfer_byte(0xFF); // dummy
    uint8_t r = bsp_spi_transfer_byte(0xFF);
    bsp_spi_set_cs(0, 0);
    return r;
}

/**
 * @brief 读word
 *
 * @param flash_addr    地址
 * @return uint32_t     数据
 */
uint32_t w25_read_word(uint32_t flash_addr)
{
    uint32_t r;

    bsp_spi_set_cs(0, 1);
    bsp_spi_transfer_byte(W25QXX_CMD_FAST_READ);
    bsp_spi_transfer_byte(flash_addr >> 16);
    bsp_spi_transfer_byte(flash_addr >> 8);
    bsp_spi_transfer_byte(flash_addr);
    bsp_spi_transfer_byte(0xFF); // dummy
    r = bsp_spi_transfer_byte(0xFF);
    r |= bsp_spi_transfer_byte(0xFF) << 8;
    r |= bsp_spi_transfer_byte(0xFF) << 16;
    r |= bsp_spi_transfer_byte(0xFF) << 24;
    bsp_spi_set_cs(0, 0);
    return r;
}

/**
 * @brief 写byte
 *
 * @param flash_addr    地址
 * @param data          数据
 */
void w25_write_byte(uint32_t flash_addr, uint8_t data)
{
    w25_enable_write_op();

    bsp_spi_set_cs(0, 1);
    bsp_spi_transfer_byte(W25QXX_CMD_PAGE_PROGRAM);
    bsp_spi_transfer_byte(flash_addr >> 16);
    bsp_spi_transfer_byte(flash_addr >> 8);
    bsp_spi_transfer_byte(flash_addr);
    bsp_spi_transfer_byte(data);
    bsp_spi_set_cs(0, 0);

    while ((w25_read_sr1() & 0x01) == 1)
    {
    }

    w25_disable_write_op();
}

/**
 * @brief 块读取
 *
 * @param flash_addr    地址
 * @param data_buff     读出数据的缓冲区
 * @param size          大小(byte)
 */
void w25_read_mass(uint32_t flash_addr, uint8_t *data_buff, uint32_t size)
{
    bsp_spi_set_cs(0, 1);
    bsp_spi_transfer_byte(W25QXX_CMD_FAST_READ);
    bsp_spi_transfer_byte(flash_addr >> 16);
    bsp_spi_transfer_byte(flash_addr >> 8);
    bsp_spi_transfer_byte(flash_addr);
    bsp_spi_transfer_byte(0xFF); // dummy
    for (uint32_t i = 0; i < size; i++)
    {
        data_buff[i] = bsp_spi_transfer_byte(0xFF); // 无限连续读
    }
    bsp_spi_set_cs(0, 0);
}

/**
 * @brief 块写入
 * 
 * @param flash_addr    地址 
 * @param data_buff     数据缓冲区
 * @param size          大小(byte)
 */
void w25_write_mass(uint32_t flash_addr, uint8_t *data_buff, uint32_t size)
{
    uint32_t op_size;

    w25_enable_write_op();

    while (size)
    {
        // 本次操作的大小
        op_size = W25QXX_PAGE_SIZE - (flash_addr % W25QXX_PAGE_SIZE);

        if (size < op_size)
        {
            op_size = size;
        }

        bsp_spi_set_cs(0, 1);
        bsp_spi_transfer_byte(W25QXX_CMD_PAGE_PROGRAM);
        bsp_spi_transfer_byte(flash_addr >> 16);
        bsp_spi_transfer_byte(flash_addr >> 8);
        bsp_spi_transfer_byte(flash_addr);
        for (uint32_t i = 0; i < op_size; i++)
        {
            bsp_spi_transfer_byte(data_buff[i]); // 无限连续读
        }
        bsp_spi_set_cs(0, 0);

        while ((w25_read_sr1() & 0x01) == 1)
        {
        }

        size -= op_size;
        flash_addr += op_size;
        data_buff += op_size;
    }

    w25_disable_write_op();
}

/**
 * @brief 扇区擦除
 *
 * @param flash_addr
 */
void w25_erase_sector_4k(uint32_t flash_addr)
{
    w25_enable_write_op();

    bsp_spi_set_cs(0, 1);
    bsp_spi_transfer_byte(W25QXX_CMD_SECTOR_ERASE_4K);
    bsp_spi_transfer_byte(flash_addr >> 16);
    bsp_spi_transfer_byte(flash_addr >> 8);
    bsp_spi_transfer_byte(flash_addr);
    bsp_spi_set_cs(0, 0);

    while ((w25_read_sr1() & 0x01) == 1)
    {
    }

    w25_disable_write_op();
}

/**
 * @brief 扇区擦除
 *
 * @param flash_addr
 */
void w25_erase_sector_64k(uint32_t flash_addr)
{
    w25_enable_write_op();

    bsp_spi_set_cs(0, 1);
    bsp_spi_transfer_byte(W25QXX_CMD_BLOCK_ERASE_64K);
    bsp_spi_transfer_byte(flash_addr >> 16);
    bsp_spi_transfer_byte(flash_addr >> 8);
    bsp_spi_transfer_byte(flash_addr);
    bsp_spi_set_cs(0, 0);

    while ((w25_read_sr1() & 0x01) == 1)
    {
    }

    w25_disable_write_op();
}

/**
 * @brief 读
 *
 * @return uint32_t
 */
uint32_t w25_read_jedec(void)
{
    uint32_t id = 0;

    bsp_spi_set_cs(0, 1);
    bsp_spi_transfer_byte(W25QXX_CMD_READ_JEDEC_ID);
    id = bsp_spi_transfer_byte(0);
    id = id << 8;
    id |= bsp_spi_transfer_byte(0);
    id = id << 8;
    id |= bsp_spi_transfer_byte(0);
    bsp_spi_set_cs(0, 0);

    return id;
}

/**
 * @brief 全片擦除
 *
 */
void w25_erase_chip(void)
{
    w25_enable_write_op();

    bsp_spi_set_cs(0, 1);
    bsp_spi_transfer_byte(W25QXX_CMD_CHIP_ERASE);
    bsp_spi_set_cs(0, 0);

    while ((w25_read_sr1() & 0x01) == 1)
    {
    }

    w25_disable_write_op();
}
