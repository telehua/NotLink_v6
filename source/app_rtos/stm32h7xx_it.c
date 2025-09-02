#include "stm32h7xx_ll_dma.h"
#include "stm32h7xx_ll_tim.h"
#include "stm32h7xx_ll_usart.h"

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

void TIM6_DAC_IRQHandler(void)
{
    LL_TIM_ClearFlag_UPDATE(TIM6);
    NVIC_ClearPendingIRQ(TIM6_DAC_IRQn);
    bsp_tick_inc();
}

#include "thread_usbd.h"
extern TX_EVENT_FLAGS_GROUP usbd_event;

void OTG_HS_IRQHandler(void)
{
    NVIC_DisableIRQ(OTG_HS_IRQn);
    tx_event_flags_set(&usbd_event, (USBD_EVENT_FLAG_IRQ), TX_OR);
}

void DMA1_Stream0_IRQHandler(void)
{
    LL_DMA_ClearFlag_TC0(DMA1);
    tx_event_flags_set(&usbd_event, (USBD_EVENT_FLAG_CDC_TX_DONE), TX_OR);
}

void DMA1_Stream1_IRQHandler(void)
{
    LL_DMA_ClearFlag_TC1(DMA1);
    tx_event_flags_set(&usbd_event, (USBD_EVENT_FLAG_CDC_RX_DONE), TX_OR);
}

void UART4_IRQHandler(void)
{
    LL_USART_ClearFlag_IDLE(UART4); // 清除
    tx_event_flags_set(&usbd_event, (USBD_EVENT_FLAG_CDC_RX_IDLE), TX_OR);
}
