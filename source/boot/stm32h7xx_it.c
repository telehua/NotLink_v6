#include "bsp.h"
#include "ulog.h"

void NMI_Handler(void)
{
    while (1)
    {
    }
}

void HardFault_Handler(void)
{
    ulog_error("%s", __func__);
    while (1)
    {
    }
}

void MemManage_Handler(void)
{
    ulog_error("%s", __func__);
    while (1)
    {
    }
}

void BusFault_Handler(void)
{
    ulog_error("%s", __func__);
    while (1)
    {
    }
}

void UsageFault_Handler(void)
{
    ulog_error("%s", __func__);
    while (1)
    {
    }
}

#include "stm32h7xx_ll_tim.h"

void TIM6_DAC_IRQHandler(void)
{
    LL_TIM_ClearFlag_UPDATE(TIM6);
    // NVIC_ClearPendingIRQ(TIM6_DAC_IRQn);
    bsp_tick_inc();
}

void OTG_HS_IRQHandler(void)
{
    // NVIC_DisableIRQ(OTG_HS_IRQn);
}
