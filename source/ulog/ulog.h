#ifndef __ULOG_H__
#define __ULOG_H__

#include <stdint.h>

/* LOG等级，优先级从高到低 */
#define ULOG_LEVEL_RAW 0U
#define ULOG_LEVEL_ERROR 1U
#define ULOG_LEVEL_WARN 2U
#define ULOG_LEVEL_INFO 3U
#define ULOG_LEVEL_DEBUG 4U

#include "ulog_config.h"

enum _shell_color
{
    shell_color_black = 30U,
    shell_color_red = 31U,
    shell_color_green = 32U,
    shell_color_yellow = 33U,
    shell_color_blue = 34U,
    shell_color_purple = 35U,
    shell_color_cyan = 36U,
    shell_color_white = 37U,
};

void ulog_init(uint32_t level);
uint32_t ulog_set_level(uint32_t level);
void ulog_printf(uint8_t level, char const *const format, ...);

#if (ULOG_COMPILE_LEVEL > ULOG_LEVEL_RAW)
#define ulog_error(...) ulog_printf(ULOG_LEVEL_ERROR, __VA_ARGS__)
#else
#define ulog_error(...)
#endif

#if (ULOG_COMPILE_LEVEL > ULOG_LEVEL_ERROR)
#define ulog_warn(...) ulog_printf(ULOG_LEVEL_WARN, __VA_ARGS__)
#else
#define ulog_warn(...)
#endif

#if (ULOG_COMPILE_LEVEL > ULOG_LEVEL_WARN)
#define ulog_info(...) ulog_printf(ULOG_LEVEL_INFO, __VA_ARGS__)
#else
#define ulog_info(...)
#endif

#if (ULOG_COMPILE_LEVEL > ULOG_LEVEL_INFO)
#define ulog_debug(...) ulog_printf(ULOG_LEVEL_DEBUG, __VA_ARGS__)
#else
#define ulog_debug(...)
#endif

#define ulog_raw(...) ulog_printf(ULOG_LEVEL_RAW, __VA_ARGS__)

#endif // !__ULOG_H__
