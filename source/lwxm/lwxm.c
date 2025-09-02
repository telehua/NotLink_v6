#include "lwxm.h"

#include "ulog.h"

#define XM_SOH 0x01
#define XM_STX 0x02
#define XM_EOT 0x04
#define XM_ACK 0x06
#define XM_NCK 0x15
#define XM_CAN 0x18
#define XM_CTRLZ 0x1A

#define XM_MODE_IDLE 0
#define XM_MODE_PACKET 1
#define XM_MODE_END 2

#define XM_PACKET_SIZE (132)

extern int lwxm_get_byte(uint8_t *ch);
extern void lwxm_put_byte_poll(uint8_t *ch);
extern uint32_t lwxm_get_tick(void);
extern int lwxm_mem_write(uint32_t addr, uint8_t *data_buff, uint32_t size);

static uint8_t lwxm_packet_buff[XM_PACKET_SIZE];
static uint32_t mem_addr;

/**
 * @brief 计算校验和
 *
 * @param data_buff
 * @param size
 * @return uint8_t
 */
uint8_t lwxm_get_check_sum(const uint8_t *data_buff, uint32_t size)
{
    uint8_t check_sum = 0;
    for (uint32_t i = 0; i < size; i++)
    {
        check_sum += data_buff[i];
    }
    return check_sum;
}

/**
 * @brief 接收循环
 *
 * @param base_address  写入地址
 * @return int          结果0成功
 */
int lwxm_loop(uint32_t base_address)
{
    uint32_t mode = XM_MODE_IDLE;
    uint32_t tick = lwxm_get_tick();
    uint8_t ch;
    int result;
    uint32_t count = 0;

    mem_addr = base_address;

    // 清空缓冲区
    while (lwxm_get_byte(&ch) == 0)
    {
    }

    /* SOH[1] + num[1] + num[1] + data[128] + (check sum)[1] */

    while (1)
    {
        result = lwxm_get_byte(&ch);

        switch (mode)
        {
        case XM_MODE_IDLE:
            if ((lwxm_get_tick() - tick) > (1000))
            {
                tick = lwxm_get_tick();
                ch = XM_NCK;
                lwxm_put_byte_poll(&ch);
            }

            if (result == 0)
            {
                switch (ch)
                {
                case XM_SOH:
                    count = 0;
                    lwxm_packet_buff[count] = ch; // 开始接收数据包
                    mode = XM_MODE_PACKET;
                    break;
                case XM_EOT:
                    ch = XM_ACK; // 接收完成
                    lwxm_put_byte_poll(&ch);
                    return 0;
                    break;
                default:
                    break;
                }
            }
            break;
        case XM_MODE_PACKET:
            if ((result == 0))
            {
                count++;
                lwxm_packet_buff[count] = ch;

                if (count == (XM_PACKET_SIZE - 1))
                {
                    // 数据包校验
                    if (lwxm_get_check_sum(&lwxm_packet_buff[4 - 1], 128) == lwxm_packet_buff[XM_PACKET_SIZE - 1])
                    {
                        lwxm_mem_write(mem_addr, &lwxm_packet_buff[4 - 1], 128);
                        mem_addr += 128;

                        ch = XM_ACK;
                        lwxm_put_byte_poll(&ch);
                    }
                    else
                    {
                        ch = XM_NCK;
                        lwxm_put_byte_poll(&ch);
                    }

                    tick = lwxm_get_tick();
                    mode = XM_MODE_IDLE;
                }
            }
            break;
        default:
            break;
        }
    }

    return -1;
}
