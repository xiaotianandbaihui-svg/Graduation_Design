#include "current_sense.h"
#include "stm32f10x.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "Delay.h"

#define ADC_WAIT_LIMIT       10000u  /* 有限轮询次数，非精确微秒，这里只是做了一个最简单的超时检测*/
#define ZERO_SAMPLES         32u
#define ZERO_TOL_COUNTS      256.0f  /* 零点应接近半量程，拒绝掉电/明显接线异常 */
#define CAL_HOLD_US          200u    /* DRV8323自动校准要求至少100us */
#define ADC_VOLTS_PER_COUNT  (CURRENT_SENSE_VDDA_V / 4096.0f)

static uint8_t ready;/*用于判断ADC启动状态的全局变量*/
static float zero_a[2], zero_b[2];/*电机零点的原始数值采取*/

static int DriverActive(void)/*这边主要就是确认那两个DRV8323是否正常工作的*/
{
    return GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_0) == Bit_SET &&/*已经使能，对应EN*/
           GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_4) == Bit_RESET &&/*这个引脚是CAL，用于DRV8323的校准，低电平表示校准完了*/
           GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) == Bit_SET &&/*nFAULT*/
           GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5) == Bit_SET;/*nFAULT*/
}

static int Fail(void)
{
    ready = 0u;
    ADC_Cmd(ADC1, DISABLE);
    ADC_Cmd(ADC2, DISABLE);
    return 0;
}

static int CalibrateADC(ADC_TypeDef *adc)/*ADC校准程序*/
{
    uint32_t wait = ADC_WAIT_LIMIT;
    ADC_ResetCalibration(adc);
    while (ADC_GetResetCalibrationStatus(adc) != RESET)
        if (--wait == 0u) return 0;
    ADC_StartCalibration(adc);
    wait = ADC_WAIT_LIMIT;
    while (ADC_GetCalibrationStatus(adc) != RESET)
        if (--wait == 0u) return 0;
    return 1;
}

int CurrentSense_ReadRaw(CurrentSense_Raw *raw)
{
    CurrentSense_Raw sample;
    uint32_t wait = ADC_WAIT_LIMIT;
    if (!ready || raw == 0 || !DriverActive()) return 0;
    ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);
    ADC_ClearFlag(ADC2, ADC_FLAG_JEOC);
    ADC_SoftwareStartInjectedConvCmd(ADC1, ENABLE); /* ADC1触发双ADC */
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_JEOC) == RESET ||
           ADC_GetFlagStatus(ADC2, ADC_FLAG_JEOC) == RESET)
        if (--wait == 0u) return Fail();/*等待转化完成*/
    sample.a[0] = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);
    sample.b[0] = ADC_GetInjectedConversionValue(ADC2, ADC_InjectedChannel_1);
    sample.a[1] = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_2);
    sample.b[1] = ADC_GetInjectedConversionValue(ADC2, ADC_InjectedChannel_2);
    if (!DriverActive()) return 0;
    *raw = sample;
    return 1;
}

int CurrentSense_Init(void)
{
    GPIO_InitTypeDef gpio;
    ADC_InitTypeDef adc;
    CurrentSense_Raw raw;
    uint32_t sum_a[2] = {0u, 0u}, sum_b[2] = {0u, 0u};
    uint8_t i, id;
    ready = 0u;
    /* 只在驱动初始化的高阻阶段调用；不替调用方切换电机输出状态。 */
    if (!DriverActive() ||
        (GPIO_ReadOutputData(GPIOB) & (GPIO_Pin_12 | GPIO_Pin_11 | GPIO_Pin_10 |
                                     GPIO_Pin_6 | GPIO_Pin_3)) != 0u ||
        GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_15) != Bit_RESET) return 0;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                          RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2, ENABLE);


    RCC_ADCCLKConfig(RCC_PCLK2_Div6); /* 72MHz系统时为12MHz，不超过14MHz */
    gpio.GPIO_Mode = GPIO_Mode_AIN;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOA, &gpio);
    gpio.GPIO_Pin = GPIO_Pin_0;
    GPIO_Init(GPIOB, &gpio);

    ADC_DeInit(ADC1);
    ADC_DeInit(ADC2);
    ADC_StructInit(&adc);
    adc.ADC_ScanConvMode = ENABLE;
    adc.ADC_ContinuousConvMode = DISABLE;
    adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    adc.ADC_DataAlign = ADC_DataAlign_Right;
    adc.ADC_NbrOfChannel = 1u; /* 不启动规则组；下面单独设置注入组长度 */
    adc.ADC_Mode = ADC_Mode_Independent;
    ADC_Init(ADC2, &adc);
    adc.ADC_Mode = ADC_Mode_InjecSimult;
    ADC_Init(ADC1, &adc);
    ADC_InjectedSequencerLengthConfig(ADC1, 2u);
    ADC_InjectedSequencerLengthConfig(ADC2, 2u);
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_5, 1u, ADC_SampleTime_28Cycles5);
    ADC_InjectedChannelConfig(ADC2, ADC_Channel_6, 1u, ADC_SampleTime_28Cycles5);
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_7, 2u, ADC_SampleTime_28Cycles5);
    ADC_InjectedChannelConfig(ADC2, ADC_Channel_8, 2u, ADC_SampleTime_28Cycles5);
    ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_None);
    ADC_ExternalTrigInjectedConvConfig(ADC2, ADC_ExternalTrigInjecConv_None);
    ADC_ExternalTrigInjectedConvCmd(ADC2, ENABLE);
    ADC_Cmd(ADC1, ENABLE);
    ADC_Cmd(ADC2, ENABLE);
    Delay_us(10u); /* 上电稳定后才能启动ADC校准 ，等待上电稳定*/
    if (!CalibrateADC(ADC1) || !CalibrateADC(ADC2)) return Fail();

    /* 此时六个外部MOS均不切换；CAL期间增益临时变为40倍，不能在此时
     * 保存20倍正常工作时的零偏。拉低CAL、恢复正常增益后再采样。 */
    GPIO_SetBits(GPIOB, GPIO_Pin_4);/*拉高校准*/
    Delay_us(CAL_HOLD_US);/*等待校准完成，这个手册是至少100us*/
    GPIO_ResetBits(GPIOB, GPIO_Pin_4);/*拉低，校准完成*/
    Delay_us(20u);
    ready = 1u;/*准备就绪完成*/

    /*开始采样，得到零偏电压值（DRV是正负）*/
    for (i = 0u; i < ZERO_SAMPLES; ++i) {
        if (!CurrentSense_ReadRaw(&raw)) return Fail();
        for (id = 0u; id < 2u; ++id) {
            sum_a[id] += raw.a[id];
            sum_b[id] += raw.b[id];
        }
    }
    for (id = 0u; id < 2u; ++id) {
        float a = (float)sum_a[id] / ZERO_SAMPLES;
        float b = (float)sum_b[id] / ZERO_SAMPLES;
        if (a < 2048.0f - ZERO_TOL_COUNTS || a > 2048.0f + ZERO_TOL_COUNTS ||
            b < 2048.0f - ZERO_TOL_COUNTS || b > 2048.0f + ZERO_TOL_COUNTS)/*这个ZERO_TOL_COUNTS是零偏容差*/
            return Fail();
        zero_a[id] = a;
        zero_b[id] = b;
    }
    return 1;
}

/*电压转化*/
int CurrentSense_Convert(uint8_t id, const CurrentSense_Raw *raw,
                         CurrentSense_Phase *current)
{
    float va, vb;
    CurrentSense_Phase result;
    const float scale = ADC_VOLTS_PER_COUNT /
                        (CURRENT_SENSE_GAIN * CURRENT_SENSE_SHUNT_OHM);
    if (!ready || id > 1u || raw == 0 || current == 0) return 0;
    va = raw->a[id] * ADC_VOLTS_PER_COUNT;
    vb = raw->b[id] * ADC_VOLTS_PER_COUNT;
    /* 取0.3V余量，超量程直接报错，不把被削顶的值当真实电流。 */
    if (va <= 0.3f || va >= CURRENT_SENSE_VDDA_V - 0.3f ||
        vb <= 0.3f || vb >= CURRENT_SENSE_VDDA_V - 0.3f) return 0;
    /* 正方向：星点经绕组流向逆变器；与原流入电机的定义相反。 */
    result.ia_a = (zero_a[id] - (float)raw->a[id]) * scale;
    result.ib_a = (zero_b[id] - (float)raw->b[id]) * scale;
    result.ic_a = -result.ia_a - result.ib_a;
    *current = result;
    return 1;
}
