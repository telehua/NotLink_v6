#include "thread_usbd.h"

#include <stdbool.h>

#include "usbd_cdc_acm.h"
#include "usbd_core.h"

#include "bsp.h"
#include "dap.h"

#include "stm32h7xx_ll_dma.h"

static char _usb_vendor_string[60];  // 厂商
static char _usb_product_string[60]; // 产品
static char _usb_ser_num_string[60]; // 序列号

#define WINUSB_IN_EP 0x81
#define WINUSB_OUT_EP 0x01

#define CDC_IN_EP 0x82
#define CDC_OUT_EP 0x02
#define CDC_INT_EP 0x83

#define USBD_VID 0x0D28
#define USBD_PID 0x0204
#define USBD_MAX_POWER 500
#define USBD_LANGID_STRING 1033

#define USB_CONFIG_SIZE (9 + 9 + 7 + 7 + CDC_ACM_DESCRIPTOR_LEN)
#define INTF_NUM 3

#ifdef CONFIG_USB_HS
#define WINUSB_EP_MPS DAP_PACKET_SIZE
#else
#define WINUSB_EP_MPS 64
#endif

#ifdef CONFIG_USB_HS
#define CDC_EP_MPS DAP_UART_PACKET_SIZE
#else
#define CDC_EP_MPS 64
#endif

#define USBD_WINUSB_VENDOR_CODE 0x20

#define USBD_WEBUSB_ENABLE 1
#define USBD_BULK_ENABLE 1
#define USBD_WINUSB_ENABLE 1

/* WinUSB Microsoft OS 2.0 descriptor sizes */

#define WINUSB_DESCRIPTOR_SET_HEADER_SIZE 10 //
#define WINUSB_FUNCTION_SUBSET_HEADER_SIZE 8 //
#define WINUSB_FEATURE_COMPATIBLE_ID_SIZE 20 //

#define FUNCTION_SUBSET_LEN 160
#define DEVICE_INTERFACE_GUIDS_FEATURE_LEN 132

/* WINUSB描述符长度 */
#define USBD_WINUSB_DESC_SET_LEN (WINUSB_DESCRIPTOR_SET_HEADER_SIZE +        \
                                  USBD_WEBUSB_ENABLE * FUNCTION_SUBSET_LEN + \
                                  USBD_BULK_ENABLE * FUNCTION_SUBSET_LEN)

// clang-format off

__ALIGN_BEGIN const uint8_t USBD_WinUSBDescriptorSetDescriptor[] = {
    WBVAL(WINUSB_DESCRIPTOR_SET_HEADER_SIZE), /* wLength */
    WBVAL(WINUSB_SET_HEADER_DESCRIPTOR_TYPE), /* wDescriptorType */
    0x00, 0x00, 0x03, 0x06,                   /* dwWindowsVersion >= Win 8.1 */
    WBVAL(USBD_WINUSB_DESC_SET_LEN),          /* wDescriptorSetTotalLength */
#if USBD_BULK_ENABLE
    WBVAL(WINUSB_FUNCTION_SUBSET_HEADER_SIZE), /* wLength */
    WBVAL(WINUSB_SUBSET_HEADER_FUNCTION_TYPE), /* wDescriptorType */
    0,                                         /* bFirstInterface USBD_BULK_IF_NUM */
    0,                                         /* bReserved */
    WBVAL(FUNCTION_SUBSET_LEN),                /* wSubsetLength */
    WBVAL(WINUSB_FEATURE_COMPATIBLE_ID_SIZE),  /* wLength */
    WBVAL(WINUSB_FEATURE_COMPATIBLE_ID_TYPE),  /* wDescriptorType */
    'W', 'I', 'N', 'U', 'S', 'B', 0, 0,        /* CompatibleId */
    0, 0, 0, 0, 0, 0, 0, 0,                    /* SubCompatibleId */
    WBVAL(DEVICE_INTERFACE_GUIDS_FEATURE_LEN), /* wLength */
    WBVAL(WINUSB_FEATURE_REG_PROPERTY_TYPE),   /* wDescriptorType */
    WBVAL(WINUSB_PROP_DATA_TYPE_REG_MULTI_SZ), /* wPropertyDataType */
    WBVAL(42),                                 /* wPropertyNameLength */
    'D', 0, 'e', 0, 'v', 0, 'i', 0, 'c', 0, 'e', 0,
    'I', 0, 'n', 0, 't', 0, 'e', 0, 'r', 0, 'f', 0, 'a', 0, 'c', 0, 'e', 0,
    'G', 0, 'U', 0, 'I', 0, 'D', 0, 's', 0, 0, 0,
    WBVAL(80), /* wPropertyDataLength */
    '{', 0,
    'C', 0, 'D', 0, 'B', 0, '3', 0, 'B', 0, '5', 0, 'A', 0, 'D', 0, '-', 0,
    '2', 0, '9', 0, '3', 0, 'B', 0, '-', 0,
    '4', 0, '6', 0, '6', 0, '3', 0, '-', 0,
    'A', 0, 'A', 0, '3', 0, '6', 0, '-', 0,
    '1', 0, 'A', 0, 'A', 0, 'E', 0, '4', 0, '6', 0, '4', 0, '6', 0, '3', 0, '7', 0, '7', 0, '6', 0,
    '}', 0, 
    0, 0, 0, 0,
#endif
#if (USBD_WEBUSB_ENABLE)
    WBVAL(WINUSB_FUNCTION_SUBSET_HEADER_SIZE), // wLength
    WBVAL(WINUSB_SUBSET_HEADER_FUNCTION_TYPE), // wDescriptorType
    0,                                         // bFirstInterface USBD_WINUSB_IF_NUM
    0,                                         // bReserved
    WBVAL(FUNCTION_SUBSET_LEN),                // wSubsetLength
    WBVAL(WINUSB_FEATURE_COMPATIBLE_ID_SIZE),  // wLength
    WBVAL(WINUSB_FEATURE_COMPATIBLE_ID_TYPE),  // wDescriptorType
    'W', 'I', 'N', 'U', 'S', 'B', 0, 0,        // CompatibleId
    0, 0, 0, 0, 0, 0, 0, 0,                    // SubCompatibleId
    WBVAL(DEVICE_INTERFACE_GUIDS_FEATURE_LEN), // wLength
    WBVAL(WINUSB_FEATURE_REG_PROPERTY_TYPE),   // wDescriptorType
    WBVAL(WINUSB_PROP_DATA_TYPE_REG_MULTI_SZ), // wPropertyDataType
    WBVAL(42),                                 // wPropertyNameLength
    'D', 0, 'e', 0, 'v', 0, 'i', 0, 'c', 0, 'e', 0,
    'I', 0, 'n', 0, 't', 0, 'e', 0, 'r', 0, 'f', 0, 'a', 0, 'c', 0, 'e', 0,
    'G', 0, 'U', 0, 'I', 0, 'D', 0, 's', 0, 0, 0,
    WBVAL(80), // wPropertyDataLength
    '{', 0,
    '9', 0, '2', 0, 'C', 0, 'E', 0, '6', 0, '4', 0, '6', 0, '2', 0, '-', 0,
    '9', 0, 'C', 0, '7', 0, '7', 0, '-', 0,
    '4', 0, '6', 0, 'F', 0, 'E', 0, '-', 0,
    '9', 0, '3', 0, '3', 0, 'B', 0, '-', 0,
    '3', 0, '1', 0, 'C', 0, 'B', 0, '9', 0, 'C', 0, '5', 0, 'A', 0, 'A', 0, '3', 0, 'B', 0, '9', 0,
    '}', 0,
    0, 0, 0, 0,
#endif
};

// clang-format on

#define USBD_NUM_DEV_CAPABILITIES (USBD_WEBUSB_ENABLE + USBD_WINUSB_ENABLE)

#define USBD_WEBUSB_DESC_LEN 24U // WEBUSB描述符长度
#define USBD_WINUSB_DESC_LEN 28U // WINUSB描述符长度

/* BOS描述符总长度 */
#define USBD_BOS_WTOTALLENGTH (0x05 +                                      \
                               USBD_WEBUSB_DESC_LEN * USBD_WEBUSB_ENABLE + \
                               USBD_WINUSB_DESC_LEN * USBD_WINUSB_ENABLE)

__ALIGN_BEGIN const uint8_t USBD_BinaryObjectStoreDescriptor[] = {
    0x05,                         /* bLength */
    0x0f,                         /* bDescriptorType */
    WBVAL(USBD_BOS_WTOTALLENGTH), /* wTotalLength */
    USBD_NUM_DEV_CAPABILITIES,    /* bNumDeviceCaps */
#if (USBD_WEBUSB_ENABLE)
    USBD_WEBUSB_DESC_LEN,           /* bLength */
    0x10,                           /* bDescriptorType */
    USB_DEVICE_CAPABILITY_PLATFORM, /* bDevCapabilityType */
    0x00,                           /* bReserved */
    0x38, 0xB6, 0x08, 0x34,         /* PlatformCapabilityUUID 16byte */
    0xA9, 0x09, 0xA0, 0x47,
    0x8B, 0xFD, 0xA0, 0x76,
    0x88, 0x15, 0xB6, 0x65,
    WBVAL(0x0100),           /* bcdVersion mast be 1.00 */
    USBD_WINUSB_VENDOR_CODE, /* bVendorCode */
    0,                       /* iLandingPage */
#endif
#if (USBD_WINUSB_ENABLE)
    USBD_WINUSB_DESC_LEN,            /* bLength */
    0x10,                            /* bDescriptorType */
    USB_DEVICE_CAPABILITY_PLATFORM,  /* bDevCapabilityType */
    0x00,                            /* bReserved */
    0xDF, 0x60, 0xDD, 0xD8,          /* PlatformCapabilityUUID */
    0x89, 0x45, 0xC7, 0x4C,          //
    0x9C, 0xD2, 0x65, 0x9D,          //
    0x9E, 0x64, 0x8A, 0x9F,          //
    0x00, 0x00, 0x03, 0x06,          /* dwWindowsVersion >= Win 8.1 */
    WBVAL(USBD_WINUSB_DESC_SET_LEN), /* wDescriptorSetTotalLength */
    USBD_WINUSB_VENDOR_CODE,         /* bVendorCode */
    0,                               /* bAltEnumCode */
#endif
};

struct usb_msosv2_descriptor msosv2_desc = {
    .vendor_code = USBD_WINUSB_VENDOR_CODE,
    .compat_id = USBD_WinUSBDescriptorSetDescriptor,
    .compat_id_len = USBD_WINUSB_DESC_SET_LEN,
};

struct usb_bos_descriptor bos_desc = {
    .string = USBD_BinaryObjectStoreDescriptor,
    .string_len = USBD_BOS_WTOTALLENGTH,
};

static const uint8_t device_descriptor[] = {
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_1, 0xEF, 0x02, 0x01, USBD_VID, USBD_PID, 0x0100, 0x01),
};

static const uint8_t config_descriptor[] = {
    /* Configuration 0 */
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, INTF_NUM, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    /* Interface 0 */
    USB_INTERFACE_DESCRIPTOR_INIT(0x00, 0x00, 0x02, 0xFF, 0x00, 0x00, 0x02),
    /* Endpoint OUT 2 */
    USB_ENDPOINT_DESCRIPTOR_INIT(WINUSB_OUT_EP, USB_ENDPOINT_TYPE_BULK, WINUSB_EP_MPS, 0x00),
    /* Endpoint IN 1 */
    USB_ENDPOINT_DESCRIPTOR_INIT(WINUSB_IN_EP, USB_ENDPOINT_TYPE_BULK, WINUSB_EP_MPS, 0x00),
    /* CDC */
    CDC_ACM_DESCRIPTOR_INIT(0x01, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, CDC_EP_MPS, 0x00),
};

/* device qualifier descriptor */
static const uint8_t device_quality_descriptor[] = {
    0x0a,
    USB_DESCRIPTOR_TYPE_DEVICE_QUALIFIER,
    0x10,
    0x02,
    0x00,
    0x00,
    0x00,
    0x40,
    0x00,
    0x00,
};

static const char *string_descriptors[] = {
    (const char[]){0x09, 0x04}, /* Langid */
    _usb_vendor_string,         /* Manufacturer */
    _usb_product_string,        /* Product */
    _usb_ser_num_string,        /* Serial Number */
};

static const uint8_t *device_descriptor_callback(uint8_t speed)
{
    if (speed == USB_SPEED_HIGH)
    {
    }
    return device_descriptor;
}

static const uint8_t *config_descriptor_callback(uint8_t speed)
{
    if (speed == USB_SPEED_HIGH)
    {
    }
    return config_descriptor;
}

static const uint8_t *device_quality_descriptor_callback(uint8_t speed)
{
    if (speed == USB_SPEED_HIGH)
    {
    }
    return device_quality_descriptor;
}

static const char *string_descriptor_callback(uint8_t speed, uint8_t index)
{
    if (speed == USB_SPEED_HIGH)
    {
    }

    if (index > 3)
    {
        return NULL;
    }
    return string_descriptors[index];
}

const struct usb_descriptor winusbv2_descriptor = {
    .device_descriptor_callback = device_descriptor_callback,
    .config_descriptor_callback = config_descriptor_callback,
    .device_quality_descriptor_callback = device_quality_descriptor_callback,
    .string_descriptor_callback = string_descriptor_callback,
    .msosv2_descriptor = &msosv2_desc,
    .bos_descriptor = &bos_desc,
};

// len(Word) + n*data(Byte)
static USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint32_t cdc_out_buff[DAP_UART_PACKET_COUNT][1 + CDC_EP_MPS / 4];
static USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint32_t cdc_in_buff[DAP_UART_PACKET_COUNT][1 + CDC_EP_MPS / 4];

static volatile uint32_t cdc_out_buff_index; // 接收位置
static volatile uint32_t cdc_in_buff_index;  // 发送位置
static volatile uint32_t cdc_tx_buff_index;  // 串口发送位置
static volatile uint32_t cdc_rx_buff_index;  // 串口接收位置
static volatile bool cdc_out_busy;           // OUT端点正在接收
static volatile bool cdc_in_busy;            // IN端点正在发送
static volatile bool cdc_tx_busy;            // TX忙
static volatile bool cdc_rx_busy;            // RX忙

// len(Word) + n*data(Byte)
static USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint32_t dap_out_buff[DAP_PACKET_COUNT][1 + WINUSB_EP_MPS / 4];
static USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint32_t dap_in_buff[DAP_PACKET_COUNT][1 + WINUSB_EP_MPS / 4];

static volatile uint32_t dap_out_buff_index; // 接收位置
static volatile uint32_t dap_op_buff_index;  // 处理位置
static volatile uint32_t dap_in_buff_index;  // 发送位置
static volatile bool dap_out_busy;           // OUT端点正在接收
static volatile bool dap_in_busy;            // IN端点正在发送

#define DAP_IO_QUEUE_BUFF_SIZE (((DAP_PACKET_COUNT) * 2) * 4) // Word * 2 * DAP_PACKET_COUNT
#define CDC_OUT_QUEUE_BUFF_SIZE ((DAP_UART_PACKET_COUNT) * 4) // Word * DAP_UART_PACKET_COUNT
#define CDC_IN_QUEUE_BUFF_SIZE ((DAP_UART_PACKET_COUNT) * 4)  // Word * DAP_UART_PACKET_COUNT

static uint32_t dap_io_queue_buff[DAP_IO_QUEUE_BUFF_SIZE / 4]; // request point + respond point

TX_EVENT_FLAGS_GROUP usbd_event; // usb事件组
TX_QUEUE dap_io_queue;           // dap处理消息队列

static void usbd_event_handler(uint8_t busid, uint8_t event)
{
    switch (event)
    {
    case USBD_EVENT_RESET:
        break;
    case USBD_EVENT_CONNECTED:
        break;
    case USBD_EVENT_DISCONNECTED:
        break;
    case USBD_EVENT_RESUME:
        break;
    case USBD_EVENT_SUSPEND:
        break;
    case USBD_EVENT_CONFIGURED:
        dap_out_buff_index = 0;
        dap_in_buff_index = 0;
        dap_op_buff_index = 0;
        dap_in_busy = false;
        dap_out_busy = true;
        cdc_out_buff_index = 0;
        cdc_in_buff_index = 0;
        cdc_tx_buff_index = 0;
        cdc_rx_buff_index = 0;
        cdc_out_busy = true;
        cdc_in_busy = false;
        cdc_tx_busy = false;
        cdc_rx_busy = true;

        /* 串口接收 */
        bsp_vcom_start_rx_dma((uint8_t *)&cdc_in_buff[cdc_rx_buff_index][1],
                              (uint8_t *)&cdc_in_buff[cdc_rx_buff_index + 1][1],
                              CDC_EP_MPS);

        /* setup first out ep read transfer */
        usbd_ep_start_read(busid, WINUSB_OUT_EP,
                           (uint8_t *)&dap_out_buff[dap_out_buff_index][1],
                           WINUSB_EP_MPS);
        usbd_ep_start_read(busid, CDC_OUT_EP,
                           (uint8_t *)&cdc_out_buff[cdc_out_buff_index][1],
                           CDC_EP_MPS);
        break;
    case USBD_EVENT_SET_REMOTE_WAKEUP:
        break;
    case USBD_EVENT_CLR_REMOTE_WAKEUP:
        break;

    default:
        break;
    }
}

/**
 * @brief 接收完成回调
 *
 * @param busid
 * @param ep
 * @param nbytes
 */
static void usbd_winusb_out(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    // OUT事件
    tx_event_flags_set(&usbd_event, USBD_EVENT_FLAG_WINUSB_OUT, TX_OR);
    dap_out_buff[dap_out_buff_index][0] = nbytes; // 记录数据长度
}

/**
 * @brief 发送完成回调
 *
 * @param busid
 * @param ep
 * @param nbytes
 */
static void usbd_winusb_in(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    // IN事件
    tx_event_flags_set(&usbd_event, USBD_EVENT_FLAG_WINUSB_IN, TX_OR);
    dap_in_buff[dap_in_buff_index][0] = nbytes; // 记录数据长度
}

/**
 * @brief CDC接收完成回调
 *
 * @param busid
 * @param ep
 * @param nbytes
 */
static void usbd_cdc_acm_out(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    // OUT事件
    tx_event_flags_set(&usbd_event, USBD_EVENT_FLAG_CDC_OUT, TX_OR);
    cdc_out_buff[cdc_out_buff_index][0] = nbytes;
}

/**
 * @brief CDC发送完成回调
 *
 * @param busid
 * @param ep
 * @param nbytes
 */
static void usbd_cdc_acm_in(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    // IN事件
    tx_event_flags_set(&usbd_event, USBD_EVENT_FLAG_CDC_IN, TX_OR);
    cdc_in_buff[cdc_in_buff_index][0] = nbytes; // 记录数据长度
}

struct usbd_endpoint winusb_out_ep1 = {
    .ep_addr = WINUSB_OUT_EP,
    .ep_cb = usbd_winusb_out};

struct usbd_endpoint winusb_in_ep1 = {
    .ep_addr = WINUSB_IN_EP,
    .ep_cb = usbd_winusb_in};

static struct usbd_endpoint cdc_out_ep = {
    .ep_addr = CDC_OUT_EP,
    .ep_cb = usbd_cdc_acm_out};

static struct usbd_endpoint cdc_in_ep = {
    .ep_addr = CDC_IN_EP,
    .ep_cb = usbd_cdc_acm_in};

struct usbd_interface winusb_intf;
struct usbd_interface cdc_intf1;
struct usbd_interface cdc_intf2;

void usbd_cdc_acm_set_line_coding(uint8_t busid, uint8_t intf, struct cdc_line_coding *line_coding)
{
    if (intf == cdc_intf1.intf_num)
    {
        bsp_vcom_set_config((uint8_t *)&line_coding->dwDTERate,
                            &line_coding->bCharFormat,
                            &line_coding->bParityType,
                            &line_coding->bDataBits);
    }
}

void usbd_cdc_acm_get_line_coding(uint8_t busid, uint8_t intf, struct cdc_line_coding *line_coding)
{
    if (intf == cdc_intf1.intf_num)
    {
        bsp_vcom_get_config((uint8_t *)&line_coding->dwDTERate,
                            &line_coding->bDataBits,
                            &line_coding->bParityType,
                            &line_coding->bCharFormat);
    }
}

void usbd_init(void)
{
    /* 获取字符串描述符 */
    DAP_GetVendorString(_usb_vendor_string);
    DAP_GetProductString(_usb_product_string);
    DAP_GetSerNumString(_usb_ser_num_string);

    usbd_desc_register(0, &winusbv2_descriptor);

    /*!< winusb */
    usbd_add_interface(0, &winusb_intf);
    usbd_add_endpoint(0, &winusb_out_ep1);
    usbd_add_endpoint(0, &winusb_in_ep1);

    /*!< cdc acm */
    usbd_add_interface(0, usbd_cdc_acm_init_intf(0, &cdc_intf1));
    usbd_add_interface(0, usbd_cdc_acm_init_intf(0, &cdc_intf2));
    usbd_add_endpoint(0, &cdc_out_ep);
    usbd_add_endpoint(0, &cdc_in_ep);

    usbd_initialize(0, USB1_OTG_HS_PERIPH_BASE, usbd_event_handler);
}

VOID thread_usbd_main(ULONG id)
{
    uint32_t usbd_event_flags;

    tx_event_flags_create(&usbd_event,
                          "usbd_event");

    tx_queue_create(&dap_io_queue,
                    "dap_io_queue",
                    2U,                    // 单位Word
                    dap_io_queue_buff,     //
                    DAP_IO_QUEUE_BUFF_SIZE // 单位byte
    );

    usbd_init();

    while (1)
    {
        tx_event_flags_get(&usbd_event,
                           (USBD_EVENT_FLAG_IRQ |         // 中断
                            USBD_EVENT_FLAG_WINUSB_OUT |  // DAP OUT
                            USBD_EVENT_FLAG_WINUSB_IN |   // DAP IN
                            USBD_EVENT_FLAG_DAP_DONE |    // DAP
                            USBD_EVENT_FLAG_CDC_OUT |     // CDC OUT
                            USBD_EVENT_FLAG_CDC_IN |      // CDC IN
                            USBD_EVENT_FLAG_CDC_TX_DONE | // TX DONE
                            USBD_EVENT_FLAG_CDC_RX_DONE | // RX IDLE
                            USBD_EVENT_FLAG_CDC_RX_IDLE), // RX DONE
                           TX_OR_CLEAR,                   // 等待任意事件并清除
                           &usbd_event_flags,
                           TX_WAIT_FOREVER); // 永远等待

        // USB中断事件
        if (usbd_event_flags & USBD_EVENT_FLAG_IRQ)
        {
            // CherryUSB处理
            usbd_irq_handler(0);
            NVIC_EnableIRQ(OTG_HS_IRQn);
        }

        // OUT事件，数据接收完成
        if (usbd_event_flags & USBD_EVENT_FLAG_WINUSB_OUT)
        {
            // 缓冲区指针发送给DAP处理
            uint32_t ptr[2] = {(uint32_t)&dap_out_buff[dap_out_buff_index][0],
                               (uint32_t)&dap_in_buff[dap_out_buff_index][0]};
            tx_queue_send(&dap_io_queue, ptr, TX_NO_WAIT);

            // 移动OUT缓冲区
            uint32_t out_index_now = (dap_out_buff_index + 1) % DAP_PACKET_COUNT;
            dap_out_buff_index = out_index_now;

            if (dap_out_buff_index == dap_op_buff_index)
            {
                // 缓冲区已满，不再接收
                dap_out_busy = false;
                ulog_debug("winusb out buff full, stop out");
            }
            else
            {
                // 缓冲区未满，继续接收
                dap_out_busy = true;
                usbd_ep_start_read(0, WINUSB_OUT_EP,
                                   (uint8_t *)&dap_out_buff[dap_out_buff_index][1],
                                   WINUSB_EP_MPS);
            }
        }

        // IN事件，数据发送完成
        if (usbd_event_flags & USBD_EVENT_FLAG_WINUSB_IN)
        {
            // 上次发送的数据长度
            uint32_t nbytes = dap_in_buff[dap_in_buff_index][0];

            // 发送zlp包，催促主机刷新数据
            if ((nbytes % usbd_get_ep_mps(0, WINUSB_IN_EP)) == 0 && (nbytes > 0))
            {
                /* send zlp */
                usbd_ep_start_write(0, WINUSB_IN_EP, NULL, 0);
                ulog_debug("winusb send zlp");
            }
            else
            {
                // 移动缓冲区
                uint32_t in_index_next = (dap_in_buff_index + 1) % DAP_PACKET_COUNT;
                dap_in_buff_index = in_index_next;

                if (dap_in_buff_index == dap_op_buff_index)
                {
                    // 发送缓冲区空，无数据可发
                    dap_in_busy = false;
                    ulog_debug("winusb in buff empty, stop in");
                }
                else
                {
                    dap_in_busy = true;
                    usbd_ep_start_write(0, WINUSB_IN_EP,
                                        (uint8_t *)&dap_in_buff[dap_in_buff_index][1],
                                        dap_in_buff[dap_in_buff_index][0]);
                }
            }
        }

        // DAP处理完成
        if (usbd_event_flags & USBD_EVENT_FLAG_DAP_DONE)
        {
            // 处理完成，移动缓冲区。空出一个OUT缓冲区，新增一个IN缓冲区
            dap_op_buff_index = (dap_op_buff_index + 1U) % DAP_PACKET_COUNT;

            // OUT空闲，此时必定有一个OUT缓冲区空闲
            if (!dap_out_busy)
            {
                ulog_debug("winusb start out");

                dap_out_busy = true;
                usbd_ep_start_read(0, WINUSB_OUT_EP,
                                   (uint8_t *)&dap_out_buff[dap_out_buff_index][1],
                                   WINUSB_EP_MPS);
            }

            // IN空闲，此时必定有一个IN缓冲区待发送
            if (!dap_in_busy)
            {
                ulog_debug("winusb start in");

                dap_in_busy = true;
                usbd_ep_start_write(0, WINUSB_IN_EP,
                                    (uint8_t *)&dap_in_buff[dap_in_buff_index][1],
                                    dap_in_buff[dap_in_buff_index][0]);
            }
        }

        // CDC OUT事件
        if (usbd_event_flags & USBD_EVENT_FLAG_CDC_OUT)
        {
            ulog_debug("cdc out %d byts", cdc_out_buff[cdc_out_buff_index][0]);

            // 移动CDC OUT缓冲区
            uint32_t out_index_now = (cdc_out_buff_index + 1) % DAP_UART_PACKET_COUNT;
            cdc_out_buff_index = out_index_now;

            // 检查缓冲区是否已满
            if (cdc_out_buff_index == cdc_tx_buff_index)
            {
                ulog_debug("cdc out buff full, stop out");
                // 缓冲区已满，不再接收
                cdc_out_busy = false;
            }
            else
            {
                ulog_debug("cdc out buff not full, start out");

                // 缓冲区未满，继续接收
                cdc_out_busy = true;
                usbd_ep_start_read(0, CDC_OUT_EP,
                                   (uint8_t *)&cdc_out_buff[cdc_out_buff_index][1],
                                   CDC_EP_MPS);
            }

            // 串口TX空闲，串口开始TX，因为此时必定有一个可用的包
            if (!cdc_tx_busy)
            {
                ulog_debug("cdc uart idle, start tx");

                cdc_tx_busy = true;
                bsp_vcom_start_tx_dma((uint8_t *)&cdc_out_buff[cdc_tx_buff_index][1], // 地址
                                      cdc_out_buff[cdc_tx_buff_index][0]);            // 长度
            }
        }

        // 串口发送完成
        if (usbd_event_flags & USBD_EVENT_FLAG_CDC_TX_DONE)
        {
            // 移动TX缓冲区
            uint32_t tx_index_now = (cdc_tx_buff_index + 1) % DAP_UART_PACKET_COUNT;
            cdc_tx_buff_index = tx_index_now;

            if (cdc_tx_buff_index == cdc_out_buff_index)
            {
                ulog_debug("cdc tx buff empty");
                // 缓冲区空
                cdc_tx_busy = false;
            }
            else
            {
                ulog_debug("cdc have data, tx them");
                // 串口TX空闲，此时TX DMA刚刚结束，继续TX
                cdc_tx_busy = true;
                bsp_vcom_start_tx_dma((uint8_t *)&cdc_out_buff[cdc_tx_buff_index][1], // 地址
                                      cdc_out_buff[cdc_tx_buff_index][0]);            // 长度
            }

            // CDC OUT空闲，此时必然空出一个缓冲区，故启动USB OUT接收数据
            if (!cdc_out_busy)
            {
                ulog_debug("cdc start out");
                cdc_out_busy = true;
                usbd_ep_start_read(0, CDC_OUT_EP,
                                   (uint8_t *)&cdc_out_buff[cdc_out_buff_index][1],
                                   CDC_EP_MPS);
            }
        }

        // ulog_set_level(ULOG_LEVEL_DEBUG);

        // 串口接收完成
        if (usbd_event_flags & USBD_EVENT_FLAG_CDC_RX_DONE)
        {
            // DMA TC 导致的中断，因为双缓冲模式，故DMA仍在运行中
            if (bsp_vcom_is_rx_dma_enable())
            {
                uint32_t n_bytes = CDC_EP_MPS; // 传输量等于数据量
                cdc_in_buff[cdc_rx_buff_index][0] = n_bytes;
                ulog_debug("cdc rx dma tc, %d bytes", n_bytes);

                uint32_t rx_index_now = (cdc_rx_buff_index + 1) % DAP_UART_PACKET_COUNT; // DMA正在接收的区域
                uint32_t rx_index_next = (rx_index_now + 1) % DAP_UART_PACKET_COUNT;     // 下一个接收区域

                // 无缓冲区可用
                if (rx_index_next == cdc_in_buff_index)
                {
                    ulog_warn("cdc rx full");

                    // 不移动缓冲区，继续覆盖区域接收，防止将溢出的危害限制在本区域
                    bsp_vcom_set_dma_next((uint8_t *)&cdc_in_buff[rx_index_now][1]);
                }
                else
                {
                    // 移动RX缓冲区，到当前工作区域
                    cdc_rx_buff_index = rx_index_now;
                    // 更新下一个缓冲区
                    bsp_vcom_set_dma_next((uint8_t *)&cdc_in_buff[rx_index_next][1]);
                }

                // CDC IN 空闲，开始传输
                if (!cdc_in_busy)
                {
                    ulog_debug("cdc start in %d bytes", cdc_in_buff[cdc_in_buff_index][0]);

                    cdc_in_busy = true;
                    usbd_ep_start_write(0, CDC_IN_EP,
                                        (uint8_t *)&cdc_in_buff[cdc_in_buff_index][1],
                                        cdc_in_buff[cdc_in_buff_index][0]);
                }
            }
            else // UART IDLE 导致的中断
            {
                uint32_t n_bytes = bsp_vcom_get_rx_dma_length(CDC_EP_MPS);
                cdc_in_buff[cdc_rx_buff_index][0] = n_bytes;
                ulog_debug("cdc rx uart idle, %d bytes", n_bytes);

                if (n_bytes > 0)
                {
                    // 移动CDC RX缓冲区
                    uint32_t rx_index_now = (cdc_rx_buff_index + 1) % DAP_UART_PACKET_COUNT;
                    uint32_t rx_index_next = (rx_index_now + 1) % DAP_UART_PACKET_COUNT;
                    cdc_rx_buff_index = rx_index_now;

                    if (cdc_rx_buff_index == cdc_in_buff_index)
                    {
                        ulog_debug("cdc rx full");
                        // 无缓冲区可用，停止接收，丢弃数据
                        cdc_rx_busy = false;
                    }
                    else // TC与IDLE同时发生，即接收到了整数倍的数据
                    {
                        ulog_debug("cdc start rx");
                        // 不用动，直接开始接收
                        cdc_rx_busy = true;
                        bsp_vcom_start_rx_dma((uint8_t *)&cdc_in_buff[rx_index_now][1],
                                              (uint8_t *)&cdc_in_buff[rx_index_next][1],
                                              CDC_EP_MPS);
                    }

                    // CDC IN 空闲，开始传输
                    if (!cdc_in_busy)
                    {
                        ulog_debug("cdc start in %d bytes", cdc_in_buff[cdc_in_buff_index][0]);
                        cdc_in_busy = true;
                        usbd_ep_start_write(0, CDC_IN_EP,
                                            (uint8_t *)&cdc_in_buff[cdc_in_buff_index][1],
                                            cdc_in_buff[cdc_in_buff_index][0]);
                    }
                }
                else // 0长度数据包
                {
                    ulog_debug("cdc start rx");

                    // 不移动缓冲区，重新开始传输
                    cdc_rx_busy = true;
                    uint32_t rx_index_next = (cdc_rx_buff_index + 1) % DAP_UART_PACKET_COUNT;
                    bsp_vcom_start_rx_dma((uint8_t *)&cdc_in_buff[cdc_rx_buff_index][1],
                                          (uint8_t *)&cdc_in_buff[rx_index_next][1],
                                          CDC_EP_MPS);

                    // 收到0长度数据包，不需要开始CDC IN
                }
            }
        }

        // CDC IN事件
        if (usbd_event_flags & USBD_EVENT_FLAG_CDC_IN)
        {
            uint32_t nbytes = cdc_in_buff[cdc_in_buff_index][0];
            ulog_debug("cdc in %d bytes", nbytes);

            if ((nbytes % usbd_get_ep_mps(0, CDC_IN_EP)) == 0 && (nbytes > 0))
            {
                // 发送zlp包
                usbd_ep_start_write(0, CDC_IN_EP, NULL, 0);
                ulog_debug("cdc send zlp");
            }
            else // 正常包
            {
                // 移动CDC IN缓冲区
                uint32_t in_index_now = (cdc_in_buff_index + 1) % DAP_UART_PACKET_COUNT;
                cdc_in_buff_index = in_index_now;

                if (cdc_in_buff_index == cdc_rx_buff_index)
                {
                    // 无数据可发
                    ulog_debug("cdc buff empty");
                    cdc_in_busy = false;
                }
                else
                {
                    ulog_debug("cdc start in %d bytes", cdc_in_buff[cdc_in_buff_index][0]);

                    cdc_in_busy = true;
                    usbd_ep_start_write(0, CDC_IN_EP,
                                        (uint8_t *)&cdc_in_buff[cdc_in_buff_index][1],
                                        cdc_in_buff[cdc_in_buff_index][0]);
                }

                // 启动串口接收，此时有空位
                if (!cdc_rx_busy)
                {
                    ulog_debug("cdc not busy, start rx");

                    cdc_rx_busy = true;
                    uint32_t rx_index_next = (cdc_rx_buff_index + 1) % DAP_UART_PACKET_COUNT;
                    bsp_vcom_start_rx_dma((uint8_t *)&cdc_in_buff[cdc_rx_buff_index][1],
                                          (uint8_t *)&cdc_in_buff[rx_index_next][1],
                                          CDC_EP_MPS);
                }
            }
        }

        if (usbd_event_flags & USBD_EVENT_FLAG_CDC_RX_IDLE)
        {
            ulog_debug("uart idle event");
            LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_1);
        }

        // ulog_set_level(ULOG_LEVEL_INFO);
    }
}
