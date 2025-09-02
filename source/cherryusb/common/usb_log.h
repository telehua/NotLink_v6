/*
 * Copyright (c) 2022, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef USB_LOG_H
#define USB_LOG_H

#include <stdio.h>

#include "ulog.h"


#ifndef USB_DBG_TAG
#define USB_DBG_TAG "USB"
#endif
/*
 * The color for terminal (foreground)
 * BLACK    30
 * RED      31
 * GREEN    32
 * YELLOW   33
 * BLUE     34
 * PURPLE   35
 * CYAN     36
 * WHITE    37
 */

#ifdef  CONFIG_USB_PRINTF_COLOR_ENABLE
#define _USB_DBG_COLOR(n) CONFIG_USB_PRINTF("\033[" #n "m")
#define _USB_DBG_LOG_HDR(lvl_name, color_n) \
    CONFIG_USB_PRINTF("\033[" #color_n "m[" lvl_name "/" USB_DBG_TAG "] ")
#define _USB_DBG_LOG_X_END \
    CONFIG_USB_PRINTF("\033[0m")
#else
#define _USB_DBG_COLOR(n)
#define _USB_DBG_LOG_HDR(lvl_name, color_n) \
    CONFIG_USB_PRINTF("[" lvl_name "/" USB_DBG_TAG "] ")
#define _USB_DBG_LOG_X_END
#endif


#ifndef CONFIG_USB_ASSERT_DISABLE
#define USB_ASSERT(f)                                                            \
    do {                                                                         \
        if (!(f)) {                                                              \
            ulog_error("ASSERT FAIL [%s] @ %s:%d", #f, __FILE__, __LINE__); \
            while (1) {                                                          \
            }                                                                    \
        }                                                                        \
    } while (false)

#define USB_ASSERT_MSG(f, fmt, ...)                                              \
    do {                                                                         \
        if (!(f)) {                                                              \
            ulog_error("ASSERT FAIL [%s] @ %s:%d", #f, __FILE__, __LINE__); \
            ulog_error(fmt "", ##__VA_ARGS__);                              \
            while (1) {                                                          \
            }                                                                    \
        }                                                                        \
    } while (false)
#else
#define USB_ASSERT(f) {}
#define USB_ASSERT_MSG(f, fmt, ...) {}
#endif

#define ___is_print(ch) ((unsigned int)((ch) - ' ') < 127u - ' ')
static inline void usb_hexdump(const void *ptr, uint32_t buflen)
{
    unsigned char *buf = (unsigned char *)ptr;
    unsigned int i, j;

    (void)buf;

    for (i = 0; i < buflen; i += 16) {
        CONFIG_USB_PRINTF("%08x:", i);

        for (j = 0; j < 16; j++)
            if (i + j < buflen) {
                if ((j % 8) == 0) {
                    CONFIG_USB_PRINTF("  ");
                }

                CONFIG_USB_PRINTF("%02X ", buf[i + j]);
            } else
                CONFIG_USB_PRINTF("   ");
        CONFIG_USB_PRINTF(" ");

        for (j = 0; j < 16; j++)
            if (i + j < buflen)
                CONFIG_USB_PRINTF("%c", ___is_print(buf[i + j]) ? buf[i + j] : '.');
        CONFIG_USB_PRINTF("\n");
    }
}

#endif /* USB_LOG_H */
