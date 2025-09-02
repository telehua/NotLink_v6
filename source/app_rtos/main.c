#include <stdint.h>

#include "bsp.h"
#include "ulog.h"

#include "board_init.h"
#include "tx_api.h"

#include "thread_dap.h"
#include "thread_usbd.h"

#define THREAD_USBD_PRIO 3
static TX_THREAD thread_usbd;
#define THREAD_USBD_STACK_SIZE (1024U * 8)
uint32_t thread_usbd_stack[THREAD_USBD_STACK_SIZE / 4];

#define THREAD_DAP_PRIO 4
static TX_THREAD thread_dap;
#define THREAD_DAP_STACK_SIZE (1024U * 8)
uint32_t thread_dap_stack[THREAD_DAP_STACK_SIZE / 4];

int main(void)
{
    board_init();
    tx_kernel_enter(); // 永远不再返回
    return 0;
}

// 至少创建一个任务
void tx_application_define(void *first_unused_memory)
{
    tx_thread_create(&thread_usbd,
                     "thread_usbd",
                     thread_usbd_main,
                     0,
                     &thread_usbd_stack[0],
                     THREAD_USBD_STACK_SIZE,
                     THREAD_USBD_PRIO,
                     THREAD_USBD_PRIO,
                     TX_NO_TIME_SLICE,
                     TX_AUTO_START);

    tx_thread_create(&thread_dap,
                     "thread_dap",
                     thread_dap_main,
                     0,
                     &thread_dap_stack[0],
                     THREAD_DAP_STACK_SIZE,
                     THREAD_DAP_PRIO,
                     THREAD_DAP_PRIO,
                     TX_NO_TIME_SLICE,
                     TX_AUTO_START);
}

/**
 * @brief 格式化字符串打印
 *
 * @param ch
 * @return int
 */
int __io_putchar(int ch)
{
    bsp_debug_send_byte_poll(ch);
    return 0;
}
