#include "bsp_vee.h"

#include "stm32h7xx_hal.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_dac.h"
#include "stm32h7xx_ll_opamp.h"

#include "bsp_tick.h"

/**
 * @brief 初始化DAC
 *
 */
void bsp_vee_init(void)
{
    LL_GPIO_InitTypeDef GPIO_InitStruct;
    LL_OPAMP_InitTypeDef OPAMP_InitStruct;
    LL_DAC_InitTypeDef DAC_InitStruct;
    LL_GPIO_StructInit(&GPIO_InitStruct);
    LL_OPAMP_StructInit(&OPAMP_InitStruct);
    LL_DAC_StructInit(&DAC_InitStruct);

    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOC);
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOE);
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_OPAMP);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_DAC12);
    LL_APB4_GRP1_EnableClock(LL_APB4_GRP1_PERIPH_VREF);

    GPIO_InitStruct.Pin = OPA_IN_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(OPA_IN_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = OPA_OUT_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(OPA_OUT_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = ADC_IN_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(ADC_IN_GPIO_Port, &GPIO_InitStruct);

    /* OPAMP2 */
    LL_OPAMP_DeInit(OPAMP2);
    OPAMP_InitStruct.PowerMode = LL_OPAMP_POWERMODE_NORMAL;
    OPAMP_InitStruct.FunctionalMode = LL_OPAMP_MODE_FOLLOWER;          // 跟随器模式
    OPAMP_InitStruct.InputNonInverting = LL_OPAMP_INPUT_NONINVERT_IO0; // 外部电压输入
    LL_OPAMP_Init(OPAMP2, &OPAMP_InitStruct);
    LL_OPAMP_SetTrimmingMode(OPAMP2, LL_OPAMP_TRIMMING_FACTORY);
    LL_OPAMP_Enable(OPAMP2);

    /* DAC1 */
    LL_DAC_DeInit(DAC1);
    DAC_InitStruct.TriggerSource = LL_DAC_TRIG_SOFTWARE;
    DAC_InitStruct.WaveAutoGeneration = LL_DAC_WAVE_AUTO_GENERATION_NONE;
    DAC_InitStruct.OutputBuffer = LL_DAC_OUTPUT_BUFFER_DISABLE;
    DAC_InitStruct.OutputConnection = LL_DAC_OUTPUT_CONNECT_INTERNAL; // 输出到OPAMP2
    DAC_InitStruct.OutputMode = LL_DAC_OUTPUT_MODE_NORMAL;
    LL_DAC_Init(DAC1, LL_DAC_CHANNEL_2, &DAC_InitStruct);

    LL_DAC_Enable(DAC1, LL_DAC_CHANNEL_2);
    while (LL_DAC_IsEnabled(DAC1, LL_DAC_CHANNEL_2) == 0)
    {
    }

    LL_DAC_DisableTrigger(DAC1, LL_DAC_CHANNEL_2);
    LL_DAC_DisableDMAReq(DAC1, LL_DAC_CHANNEL_2);

    bsp_vee_set_ref_source(VEE_SOURCE_VREF);
    bsp_vee_set_dac_voltage(0);

    LL_ADC_InitTypeDef ADC_InitStruct = {0};
    LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
    LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    /** Initializes the peripherals clock */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_CLKP;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
        // Error_Handler();
    }

    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_ADC12);

    LL_ADC_DeInit(ADC1); // 关闭，防止ADC没关卡在校准
    ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
    ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE; // 不适用低功耗
    ADC_InitStruct.LeftBitShift = LL_ADC_LEFT_BIT_SHIFT_NONE;
    LL_ADC_Init(ADC1, &ADC_InitStruct);

    LL_ADC_SetOverSamplingScope(ADC1, LL_ADC_OVS_DISABLE); // 过采样关
    LL_ADC_SetBoostMode(ADC1, LL_ADC_BOOST_MODE_25MHZ);
    LL_ADC_SetChannelPreselection(ADC1, LL_ADC_CHANNEL_8);
    LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_8, LL_ADC_SINGLE_ENDED);

    ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE; // 软件触发
    ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
    ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
    ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;   //  单次采样
    ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_OVERWRITTEN; // 覆盖上次转换结果
    LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
    LL_ADC_REG_SetDataTransferMode(ADC1, LL_ADC_REG_DR_TRANSFER);

    ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_ASYNC_DIV1;
    ADC_CommonInitStruct.Multimode = LL_ADC_MULTI_INDEPENDENT;
    LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);

    /* Disable ADC deep power down (enabled by default after reset state) */
    LL_ADC_DisableDeepPowerDown(ADC1);
    /* Enable ADC internal voltage regulator */
    LL_ADC_EnableInternalRegulator(ADC1);

#if 0
    /* Delay for ADC internal voltage regulator stabilization. */
    /* Compute number of CPU cycles to wait for, from delay in us. */
    /* Note: Variable divided by 2 to compensate partially */
    /* CPU processing cycles (depends on compilation optimization). */
    /* Note: If system core clock frequency is below 200kHz, wait time */
    /* is only a few CPU processing cycles. */
    __IO uint32_t wait_loop_index;
    wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
    while (wait_loop_index != 0)
    {
        wait_loop_index--;
    }
#else
    bsp_tick_delay(5);
#endif

    LL_ADC_StartCalibration(ADC1, LL_ADC_CALIB_OFFSET, LL_ADC_SINGLE_ENDED);
    while (LL_ADC_IsCalibrationOnGoing(ADC1))
    {
    }

    /** Configure Regular Channel */
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_8);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_8, LL_ADC_SAMPLINGTIME_8CYCLES_5);
    LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_8, LL_ADC_SINGLE_ENDED);

    LL_ADC_Enable(ADC1);
}

void bsp_vee_set_ref_source(vee_source_t src)
{
    switch (src)
    {
    case VEE_SOURCE_DAC:
        LL_OPAMP_SetInputNonInverting(OPAMP2, LL_OPAMP_INPUT_NONINVERT_DAC); // 内部
        break;
    case VEE_SOURCE_VREF:
    default:
        LL_OPAMP_SetInputNonInverting(OPAMP2, LL_OPAMP_INPUT_NONINVERT_IO0); // 外部
        break;
    }
}

/**
 * @brief 设置DAC电压
 *
 * @param vee_mv 电压值(mV)
 */
void bsp_vee_set_dac_voltage(uint32_t vee_mv)
{
    uint32_t d = vee_mv * 4096 / 3300 / 2;
    if (d > 4095)
    {
        d = 4095;
    }

    LL_DAC_ConvertData12RightAligned(DAC1, LL_DAC_CHANNEL_2, d);
}

/**
 * @brief 获取外部参考电压(mV)
 *
 * @return uint32_t
 */
uint32_t bsp_vee_get_vref_voltage(void)
{
    LL_ADC_REG_StartConversion(ADC1);
    while (LL_ADC_REG_IsConversionOngoing(ADC1))
    {
    }

    uint32_t adc_raw = LL_ADC_REG_ReadConversionData12(ADC1);
    uint32_t voltage = adc_raw * 3300 * 2 / 4096; // 分压电阻2
    return voltage;
}
