#include "thread_dap.h"
#include "thread_usbd.h"

#include "dap.h"

extern TX_QUEUE dap_io_queue;           //
extern TX_EVENT_FLAGS_GROUP usbd_event; // usb事件组

VOID thread_dap_main(ULONG id)
{
    uint32_t *data_ptr[2] = {0, 0}; // OUT, IN
    uint32_t request_size;
    uint8_t *request;
    uint8_t *response;

    DAP_Setup();

    while (1)
    {
        // 等待数据就绪
        tx_queue_receive(&dap_io_queue, data_ptr, TX_WAIT_FOREVER);
        request = (uint8_t *)&data_ptr[0][1];
        response = (uint8_t *)&data_ptr[1][1];

        // 执行操作
        uint32_t result = DAP_ExecuteCommand(request, response);

        request_size = data_ptr[0][0];
        data_ptr[1][0] = result & 0x0000FFFF; // 写入IN数据大小，只取低半字

        if (request_size & 0x80000000)
        {
            // 通知内部处理
        }
        else
        {
            // 通知USB处理
            tx_event_flags_set(&usbd_event, (USBD_EVENT_FLAG_DAP_DONE), TX_OR);
        }
    }
}
