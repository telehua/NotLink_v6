#include "bsp_crc.h"

#include "stm32h7xx_hal_cortex.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_crc.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_utils.h"

/**
 * @brief 初始化
 *
 */
void bsp_crc_init(void)
{
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_CRC);

    LL_CRC_SetPolynomialCoef(CRC, 0x04C11DB7);
    LL_CRC_SetPolynomialSize(CRC, LL_CRC_POLYLENGTH_32B);
    LL_CRC_SetInitialData(CRC, 0xFFFFFFFF);
}

void bsp_crc_deinit(void)
{
    LL_AHB4_GRP1_DisableClock(LL_AHB4_GRP1_PERIPH_CRC);
}

/**
 * @brief
 *
 * @return uint32_t
 */
uint32_t bsp_crc_get_crc32(uint8_t *buff, uint32_t size)
{
    LL_CRC_ResetCRCCalculationUnit(CRC);
    for (uint32_t i = 0; i < size; i++)
    {
        LL_CRC_FeedData8(CRC, buff[i]);
    }

    return LL_CRC_ReadData32(CRC);
}
