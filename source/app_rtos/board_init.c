#include "stm32h7xx_hal_cortex.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_cortex.h"
#include "stm32h7xx_ll_pwr.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_system.h"
#include "stm32h7xx_ll_utils.h"

#include "bsp.h"
#include "config.h"
#include "ulog.h"

void sys_mpu_config(void);
void sys_clock_config(void);

/**
 * @brief 板级初始化
 *
 */
void board_init(void)
{
    LL_APB4_GRP1_EnableClock(LL_APB4_GRP1_PERIPH_SYSCFG);
    NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4); // 4抢占0响应

    sys_mpu_config();
    sys_clock_config();

    bsp_debug_init();
    ulog_init(ULOG_LEVEL_INFO);
    ulog_info("App entry");

    bsp_tick_init();
    bsp_led_init();
    bsp_jtag_init();
    bsp_vee_init();
    bsp_usb_init();
    bsp_spi_init();
    bsp_crc_init();
    bsp_vcom_init();

    ulog_info("Build time: %s", CONFIG_BUILD_TIME);
    ulog_info("Build type: %s", CONFIG_BUILD_TYPE);
}

void sys_clock_config(void)
{
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
    while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_4)
    {
    }
    LL_PWR_ConfigSupply(LL_PWR_LDO_SUPPLY);
    LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE0);
    while (LL_PWR_IsActiveFlag_VOS() == 0)
    {
    }
    LL_RCC_HSE_Enable();

    /* Wait till HSE is ready */
    while (LL_RCC_HSE_IsReady() != 1)
    {
    }
    LL_RCC_PLL_SetSource(LL_RCC_PLLSOURCE_HSE);
    LL_RCC_PLL1P_Enable();
    LL_RCC_PLL1R_Enable();
    LL_RCC_PLL1_SetVCOInputRange(LL_RCC_PLLINPUTRANGE_4_8);
    LL_RCC_PLL1_SetVCOOutputRange(LL_RCC_PLLVCORANGE_WIDE);
    LL_RCC_PLL1_SetM(6);   // 24/6=4MHz
    LL_RCC_PLL1_SetN(240); // 4*240=960MHz
    LL_RCC_PLL1_SetP(2);   // 960/2=480MHz
    LL_RCC_PLL1_SetQ(20);  // 960/20=48MHz
    LL_RCC_PLL1_SetR(48);
    LL_RCC_PLL1_Enable();

    /* Wait till PLL is ready */
    while (LL_RCC_PLL1_IsReady() != 1)
    {
    }

    /* Intermediate AHB prescaler 2 when target frequency clock is higher than 80 MHz */
    LL_RCC_SetAHBPrescaler(LL_RCC_AHB_DIV_2);

    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL1);

    /* Wait till System clock is ready */
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL1)
    {
    }
    LL_RCC_SetSysPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAHBPrescaler(LL_RCC_AHB_DIV_2);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
    LL_RCC_SetAPB3Prescaler(LL_RCC_APB3_DIV_2);
    LL_RCC_SetAPB4Prescaler(LL_RCC_APB4_DIV_2);

    LL_SetSystemCoreClock(480000000);

    LL_RCC_ConfigMCO(LL_RCC_MCO2SOURCE_HSE, LL_RCC_MCO2_DIV_1); // 外部时钟输出，USB3300
    LL_RCC_SetUSARTClockSource(LL_RCC_USART16_CLKSOURCE_PCLK2); // USART1
    LL_RCC_SetUSARTClockSource(LL_RCC_USART234578_CLKSOURCE_PCLK1);

    LL_RCC_PLL2P_Enable();
    LL_RCC_PLL2_SetVCOInputRange(LL_RCC_PLLINPUTRANGE_4_8);
    LL_RCC_PLL2_SetVCOOutputRange(LL_RCC_PLLVCORANGE_WIDE);
    LL_RCC_PLL2_SetM(6);   // 24/6=4MHz
    LL_RCC_PLL2_SetN(200); // 4*200=800MHz
    LL_RCC_PLL2_SetP(4);   // 800/4=200MHz spi max
    LL_RCC_PLL2_SetQ(16);
    LL_RCC_PLL2_SetR(16);
    LL_RCC_PLL2_Enable();

    /* Wait till PLL is ready */
    while (LL_RCC_PLL2_IsReady() != 1)
    {
    }

    LL_RCC_SetSPIClockSource(LL_RCC_SPI123_CLKSOURCE_PLL2P); // SPI
    LL_RCC_SetCLKPClockSource(LL_RCC_CLKP_CLKSOURCE_HSE);
}

void sys_mpu_config(void)
{
    /* Disables the MPU */
    LL_MPU_Disable();

    /** Initializes and configures the Region and the memory to be protected */

    LL_MPU_ConfigRegion(LL_MPU_REGION_NUMBER0,                 // AXIRAM，程序执行区域，只读
                        0x00,                                  // 子区域全部使能
                        D1_AXISRAM_BASE,                       // 起始地址，AXIRAM
                        LL_MPU_REGION_SIZE_512KB               // 空间大小，512K
                            | LL_MPU_TEX_LEVEL0                // TEX0
                            | LL_MPU_REGION_PRIV_RO_URO        // 只读，仅用于执行程序
                            | LL_MPU_INSTRUCTION_ACCESS_ENABLE // 可执行
                            | LL_MPU_ACCESS_CACHEABLE          // C 开CACHE
                            | LL_MPU_ACCESS_BUFFERABLE         // B 开缓冲
                            | LL_MPU_ACCESS_NOT_SHAREABLE      // S 不共享，不需要多核一致性
    );

    LL_MPU_ConfigRegion(LL_MPU_REGION_NUMBER1,                  // 外设区，保证寄存器正确操作
                        0x00,                                   // 子区域全部使能
                        PERIPH_BASE,                            // 起始地址，外设区
                        LL_MPU_REGION_SIZE_512MB                // 空间大小，512M
                            | LL_MPU_TEX_LEVEL0                 // TEX0
                            | LL_MPU_REGION_FULL_ACCESS         // 全部可访问
                            | LL_MPU_INSTRUCTION_ACCESS_DISABLE // 不可执行
                            | LL_MPU_ACCESS_NOT_CACHEABLE       // C 关CACHE
                            | LL_MPU_ACCESS_BUFFERABLE          // B 开缓冲
                            | LL_MPU_ACCESS_NOT_SHAREABLE       // S 不共享，不需要多核一致性
    );

    LL_MPU_ConfigRegion(LL_MPU_REGION_NUMBER2,                  // D2SRAM，数据区域
                        0x00,                                   // 子区域全部使能
                        D2_AHBSRAM_BASE,                        // 起始地址，D2RAM
                        LL_MPU_REGION_SIZE_128KB                // 空间大小，128K
                            | LL_MPU_TEX_LEVEL0                 // TEX0
                            | LL_MPU_REGION_FULL_ACCESS         // 全部可访问
                            | LL_MPU_INSTRUCTION_ACCESS_DISABLE // 不可执行
                            | LL_MPU_ACCESS_NOT_CACHEABLE       // C 关CACHE
                            | LL_MPU_ACCESS_BUFFERABLE          // B 开缓冲
                            | LL_MPU_ACCESS_NOT_SHAREABLE       // S 不共享，不需要多核一致性
    );

    LL_MPU_ConfigRegion(LL_MPU_REGION_NUMBER3,                  // 核心寄存器，部分配置由硬件设定，MPU无法更改，这里仅作为占位
                        0x00,                                   // 子区域全部使能
                        0xE0000000UL,                           // 起始地址
                        LL_MPU_REGION_SIZE_1MB                  // 空间大小，1M
                            | LL_MPU_TEX_LEVEL0                 // TEX0
                            | LL_MPU_REGION_PRIV_RW             // 仅特权模式可访问
                            | LL_MPU_INSTRUCTION_ACCESS_DISABLE // 不可执行
                            | LL_MPU_ACCESS_NOT_CACHEABLE       // C 关CACHE
                            | LL_MPU_ACCESS_NOT_BUFFERABLE      // B 关缓冲
                            | LL_MPU_ACCESS_SHAREABLE           // S 需要多核一致性
    );

    LL_MPU_ConfigRegion(LL_MPU_REGION_NUMBER4,                 // ITCM，程序执行区域，只读
                        0x00,                                  // 子区域全部使能
                        D1_ITCMRAM_BASE,                       // 起始地址，AXIRAM
                        LL_MPU_REGION_SIZE_64KB                // 空间大小，64K
                            | LL_MPU_TEX_LEVEL0                // TEX0
                            | LL_MPU_REGION_PRIV_RO_URO        // 只读，仅用于执行程序
                            | LL_MPU_INSTRUCTION_ACCESS_ENABLE // 可执行
                            | LL_MPU_ACCESS_CACHEABLE          // C 开CACHE
                            | LL_MPU_ACCESS_BUFFERABLE         // B 开缓冲
                            | LL_MPU_ACCESS_NOT_SHAREABLE      // S 不共享，不需要多核一致性
    );

    LL_MPU_ConfigRegion(LL_MPU_REGION_NUMBER5,                  // DTCM，数据区域
                        0x00,                                   // 子区域全部使能
                        D1_DTCMRAM_BASE,                        // 起始地址，D2RAM
                        LL_MPU_REGION_SIZE_128KB                // 空间大小，128K
                            | LL_MPU_TEX_LEVEL0                 // TEX0
                            | LL_MPU_REGION_FULL_ACCESS         // 全部可访问
                            | LL_MPU_INSTRUCTION_ACCESS_DISABLE // 不可执行
                            | LL_MPU_ACCESS_CACHEABLE           // C 开CACHE
                            | LL_MPU_ACCESS_BUFFERABLE          // B 开缓冲
                            | LL_MPU_ACCESS_NOT_SHAREABLE       // S 不共享，不需要多核一致性
    );

    /* Enables the MPU */
    LL_MPU_Enable(LL_MPU_CTRL_PRIVILEGED_DEFAULT);

    // 打开指令缓存
    SCB_EnableICache();
}
