#include "ulog.h"

#include <inttypes.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>

#if ULOG_INDEX_ENABLE
static uint32_t _ulog_index = 0;
#endif

static uint32_t _ulog_print_level = ULOG_LEVEL_RAW;

/* 前缀 */
static const char *ulog_level_type[] = {
    "[     ] ", // (never used)
    "[ERROR] ", //
    "[WARN ] ", //
    "[INFO ] ", //
    "[DEBUG] ", //
    "[     ] ", // (never used)
};

/* LOG等级对应颜色 */
#if (ULOG_COLOR_ENABLE == 1)
static uint8_t ulog_level_color[] = {
    shell_color_white,  // (never used)
    shell_color_red,    // ERROR
    shell_color_yellow, // WARN
    shell_color_white,  // INFO
    shell_color_green,  // DEBUG
    shell_color_white   // (never used)
};
#endif

/**
 * @brief 初始化
 *
 * @param level
 *      ULOG_LEVEL_NONE     无输出
 *      ULOG_LEVEL_ERROR
 *      ULOG_LEVEL_WARN
 *      ULOG_LEVEL_INFO
 *      ULOG_LEVEL_DEBUGU   所有输出
 */
void ulog_init(uint32_t level)
{
    _ulog_print_level = level;

#if ULOG_INDEX_ENABLE
    _ulog_index = 0;
#endif
}

/**
 * @brief 设置日至等级
 *
 * @param level     等级
 *                  ULOG_LEVEL_NONE     无输出
 *                  ULOG_LEVEL_ERROR
 *                  ULOG_LEVEL_WARN
 *                  ULOG_LEVEL_INFO
 *                  ULOG_LEVEL_DEBUGU   所有输出
 *
 * @return uint32_t 更改前的日志等级，便于恢复
 */
uint32_t ulog_set_level(uint32_t level)
{
    uint32_t l = _ulog_print_level;
    _ulog_print_level = level;
    return l;
}

/**
 * @brief 日志输出
 *
 * @param level
 * @param format
 * @param ...
 */
void ulog_printf(uint8_t level, char const *const format, ...)
{
    if (level == ULOG_LEVEL_RAW) // 原样输出
    {
        va_list args;
        va_start(args, format);

        vprintf(format, args);
        va_end(args);

        return;
    }

    if (level > _ulog_print_level) // 优先级太低
    {
        return;
    }

#if (ULOG_COLOR_ENABLE == 1)
    printf("\033[%dm", ulog_level_color[level]);
#endif

#if (ULOG_INDEX_ENABLE == 1)
    printf("%d %s", (int)_ulog_index++, ulog_level_type[level]);
#else
    printf("%s", ulog_level_type[level]);
#endif

    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);

    printf("\r\n");

#if (ULOG_COLOR_ENABLE == 1)
    printf("\033[0m");
#endif
}
