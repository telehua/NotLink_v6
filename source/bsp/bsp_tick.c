#include "bsp_tick.h"

#include "stm32h7xx_hal_cortex.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_tim.h"
#include "stm32h7xx_ll_utils.h"

// #define TICK_USE_SYSTICK

__IO uint32_t _tick_count = 0;

/**
 * @brief 初始化
 *
 */
void bsp_tick_init(void)
{
#ifdef TICK_USE_SYSTICK
    LL_RCC_ClocksTypeDef f;
    LL_RCC_GetSystemClocksFreq(&f);
    ulog_info("SYSCLK freq: %d", f.SYSCLK_Frequency);

    /* Configure the SysTick to have interrupt in 1ms time base */
    SysTick->LOAD = (uint32_t)((f.SYSCLK_Frequency / 1000) - 1UL); /* set reload register */
    SysTick->VAL = 0UL;                                            /* Load the SysTick Counter Value */
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk |
                    SysTick_CTRL_ENABLE_Msk; /* Enable the Systick Timer */

    NVIC_SetPriority(SysTick_IRQn, 15); // 最低优先级，数字越小优先级越高
#else
    LL_TIM_InitTypeDef TIM_InitStruct;

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM6);

    LL_TIM_StructInit(&TIM_InitStruct);
    TIM_InitStruct.Prescaler = 240 - 1;
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload = 240000000 / 240 / 1000 - 1;
    LL_TIM_Init(TIM6, &TIM_InitStruct);
    LL_TIM_DisableARRPreload(TIM6);
    LL_TIM_SetTriggerOutput(TIM6, LL_TIM_TRGO_RESET);
    LL_TIM_DisableMasterSlaveMode(TIM6);
    LL_TIM_GenerateEvent_UPDATE(TIM6);
    LL_TIM_EnableIT_UPDATE(TIM6);

    LL_TIM_EnableCounter(TIM6);

    NVIC_SetPriority(TIM6_DAC_IRQn, 0xFF);
    NVIC_EnableIRQ(TIM6_DAC_IRQn);

#endif
}

void bsp_tick_deinit(void)
{
#ifdef TICK_USE_SYSTICK
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk; // 关闭定时器
#else
    LL_TIM_DisableIT_UPDATE(TIM6);
    LL_TIM_DisableCounter(TIM6);
#endif
}

/**
 * @brief
 *
 * @return uint32_t
 */
uint32_t bsp_tick_get_count(void)
{
    return _tick_count;
}

void bsp_tick_delay(uint32_t delay_tick)
{
    uint32_t start = bsp_tick_get_count();
    while ((bsp_tick_get_count() - start) < delay_tick)
    {
    }
}

void bsp_tick_inc(void)
{
    _tick_count++;
}
