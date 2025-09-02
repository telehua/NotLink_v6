#ifndef __TICK_H__
#define __TICK_H__

#include <stdint.h>

void bsp_tick_init(void);
void bsp_tick_deinit(void);
uint32_t bsp_tick_get_count(void);
void bsp_tick_delay(uint32_t delay_tick);
void bsp_tick_inc(void);

#endif /* __TICK_H__ */
