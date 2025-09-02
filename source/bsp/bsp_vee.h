#ifndef __VEE_H__
#define __VEE_H__

#include <stdint.h>

#include "stm32h7xx_ll_gpio.h"

#define OPA_IN_GPIO_Port GPIOE
#define OPA_IN_Pin LL_GPIO_PIN_9

#define OPA_OUT_GPIO_Port GPIOE
#define OPA_OUT_Pin LL_GPIO_PIN_7

#define ADC_IN_GPIO_Port GPIOC
#define ADC_IN_Pin LL_GPIO_PIN_5

typedef enum _vee_source_enum
{
    VEE_SOURCE_VREF = 0, // 外部VREF参考
    VEE_SOURCE_DAC,      // 内部DAC固定输出
} vee_source_t;

void bsp_vee_init(void);
void bsp_vee_set_ref_source(vee_source_t src);
void bsp_vee_set_dac_voltage(uint32_t vee_mv);
uint32_t bsp_vee_get_vref_voltage(void);

#endif /* __VEE_H__ */
