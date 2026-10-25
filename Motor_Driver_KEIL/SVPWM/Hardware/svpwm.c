#include "svpwm.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "tle5012b.h"
#include <math.h>
#include "Delay.h"
#include "current_sense.h"

float zero_point_1=0,zero_point_2=0;//记录两个电机在磁编码器特定安装下的A相角度
float first_mechanical_angle=0,second_mechanical_angle=0;//记录电机当前机械角度
float electrical1=0,electrical2=0;//记录当前电机的电角度 [0°,360°)
static TIM_TypeDef * const timer[2] = {TIM2, TIM4};
static uint16_t period, current_compare_limit;
static uint8_t ready, running[2], calibrated;
static int8_t encoder_sign[2];

static void ClearCalibration(void)
{
    calibrated = 0u;
    encoder_sign[0] = encoder_sign[1] = 0;
    zero_point_1 = zero_point_2 = electrical1 = electrical2 = 0.0f;
}

int8_t SVPWM_GetEncoderSign(uint8_t id)
{
    return calibrated && id < 2u ? encoder_sign[id] : 0;
}


static float Wrap360(float angle)
{
    angle = fmodf(angle, 360.0f);
    if (angle < 0.0f) angle += 360.0f;
    return angle >= 360.0f ? 0.0f : angle;
}

/* 3PWM模式：INL=1时响应INH的PWM；INL=0时该相上下管均关闭。 */
static void PhaseEnable(uint8_t id, BitAction state)
{
    if (id == 0u) {
        GPIO_WriteBit(GPIOB, GPIO_Pin_12 | GPIO_Pin_11 | GPIO_Pin_10, state);
    } else {
        GPIO_WriteBit(GPIOB, GPIO_Pin_6 | GPIO_Pin_3, state);
        GPIO_WriteBit(GPIOA, GPIO_Pin_15, state);
    }
}

void SVPWM_Stop(uint8_t id)
{
    if (ready && id < 2u) {
        PhaseEnable(id, Bit_RESET);
        running[id] = 0u;
    }
}

static int Fail(void)
{
    SVPWM_Stop(0u);
    SVPWM_Stop(1u);
    GPIO_ResetBits(GPIOA, GPIO_Pin_0);
    ready = 0u;
    ClearCalibration();
    return 0;
}

static int DriverOK(void)
{
    return GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) == Bit_SET &&
           GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5) == Bit_SET;
}

static void PinMode(GPIO_TypeDef *port, uint16_t pins, GPIOMode_TypeDef mode)
{
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = pins;
    gpio.GPIO_Mode = mode;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(port, &gpio);
}

static void InitPWM(TIM_TypeDef *tim)
{
    TIM_TimeBaseInitTypeDef base;
    TIM_OCInitTypeDef oc;

    TIM_DeInit(tim);
    TIM_TimeBaseStructInit(&base);
    base.TIM_Prescaler = 0u;
    base.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
    base.TIM_Period = period; /* 中心对齐：fPWM=fTIM/(2*ARR)，这里不减1。 */
    TIM_TimeBaseInit(tim, &base);

    TIM_OCStructInit(&oc);
    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    oc.TIM_Pulse = period / 2u;
    TIM_OC2Init(tim, &oc);
    TIM_OC3Init(tim, &oc);
    TIM_OC4Init(tim, &oc);
    TIM_OC2PreloadConfig(tim, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(tim, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(tim, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(tim, ENABLE);
    TIM_GenerateEvent(tim, TIM_EventSource_Update);
    TIM_ClearFlag(tim, TIM_FLAG_Update);
    TIM_Cmd(tim, ENABLE);
}
/*这个函数要在所有电流环相关代码前调用，否则对应的电流环函数在检查DRV芯片状态的时候大概率出错卡死*/
int SVPWM_Init(void)
{
    RCC_ClocksTypeDef clocks;
    uint32_t tim_hz, counts;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                          RCC_APB2Periph_AFIO, ENABLE);
    /* 先预置ODR低，再切输出模式，避免切换时出现高电平。 */
    GPIO_ResetBits(GPIOA, GPIO_Pin_0);
    PinMode(GPIOA, GPIO_Pin_0, GPIO_Mode_Out_PP);
    ready = running[0] = running[1] = 0u;
    current_compare_limit = 0u;
    ClearCalibration();
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, DISABLE);
    GPIO_PinRemapConfig(GPIO_Remap_TIM4, DISABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    GPIO_ResetBits(GPIOB, GPIO_Pin_4); /* CAL：释放JTAG后拉低，先于EN使能。 */
    
    PinMode(GPIOB, GPIO_Pin_4, GPIO_Mode_Out_PP);
    PhaseEnable(0u, Bit_RESET);
    PhaseEnable(1u, Bit_RESET);
    PinMode(GPIOB, GPIO_Pin_12 | GPIO_Pin_11 | GPIO_Pin_10 |
                   GPIO_Pin_6 | GPIO_Pin_3, GPIO_Mode_Out_PP);
    PinMode(GPIOA, GPIO_Pin_15, GPIO_Mode_Out_PP);
    PinMode(GPIOA, GPIO_Pin_8, GPIO_Mode_IPU);
    PinMode(GPIOB, GPIO_Pin_5, GPIO_Mode_IPU);

    /* 只读取已配置的时钟；APB1有分频时，定时器时钟为PCLK1的两倍。 */
    RCC_GetClocksFreq(&clocks);
    tim_hz = clocks.PCLK1_Frequency;
    if (clocks.PCLK1_Frequency != clocks.HCLK_Frequency) tim_hz *= 2u;
    counts = tim_hz / (2u * SVPWM_PWM_HZ);
    if (counts < 2u || counts > 65535u) return 0;
    period = (uint16_t)counts;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM4, ENABLE);
    InitPWM(TIM2);
    InitPWM(TIM4);
    PinMode(GPIOA, GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3, GPIO_Mode_AF_PP);
    PinMode(GPIOB, GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9, GPIO_Mode_AF_PP);
    Delay_us(2000u);
    GPIO_SetBits(GPIOA, GPIO_Pin_0);
    Delay_us(2000u); /* 等待DRV唤醒，INL仍全为低。 */
    ready = 1u;
    /* INL仍全低、MOS不切换时完成CSA校准及四路ADC零偏采集。 */
    return DriverOK() && CurrentSense_Init() ? 1 : Fail();
}

static uint16_t Compare(float duty)
{
    /* 吸收浮点舍入误差；CCR范围0~ARR。 */
    if (duty <= 0.0f) return 0u;
    if (duty >= 1.0f) return period;
    return (uint16_t)(duty * (float)period + 0.5f);
}

int SVPWM_Set(uint8_t id, float voltage_deg, uint16_t modulation)
{
    float angle, amplitude, alpha, beta, a, b, c, offset;
    uint16_t ca, cb, cc;
    TIM_TypeDef *tim;

    if (!ready) return 0;
    if (id > 1u || !isfinite(voltage_deg) || modulation > SVPWM_MAX_MOD ||
        !DriverOK()) return Fail();
    if (modulation == 0u) {
        SVPWM_Stop(id);
        return 1;
    }
    angle = Wrap360(voltage_deg);
    angle *= 0.01745329252f;
    amplitude = (float)modulation * (0.57735026919f / SVPWM_MAX_MOD);
    alpha = amplitude * cosf(angle);
    beta  = amplitude * sinf(angle);

    /* 逆Clarke：以母线电压归一化的三相电压。 */
    a = alpha;
    b = -0.5f * alpha + 0.86602540378f * beta;
    c = -0.5f * alpha - 0.86602540378f * beta;
    /* 注入公共偏移：与线性区对称SVPWM等价，不改变三相线电压。 */
    offset = 0.5f - 0.5f * (fmaxf(a, fmaxf(b, c)) + fminf(a, fminf(b, c)));
    ca = Compare(a + offset);
    cb = Compare(b + offset);
    cc = Compare(c + offset);

    tim = timer[id];
    /* 暂停更新事件，防止三个CCR写到一半就被装载；计数器照常运行。 */
    TIM_UpdateDisableConfig(tim, ENABLE);
    TIM_SetCompare2(tim, ca);
    TIM_SetCompare3(tim, cb);
    TIM_SetCompare4(tim, cc);
    TIM_UpdateDisableConfig(tim, DISABLE);
    if (!running[id]) {
        /* 仅从高阻启动时立即装载；正常运行不强制UG，避免重置PWM相位。 */
        TIM_GenerateEvent(tim, TIM_EventSource_Update);
        TIM_ClearFlag(tim, TIM_FLAG_Update);
        PhaseEnable(id, Bit_SET);
        running[id] = 1u;
    }
    return 1;
}

/* 保持给定矢量，先等待500ms，再要求100ms内角度变化不超过0.25度。
 * 最多等待1500ms；读取失败或nFAULT异常立即退出。 */
static int AlignAndRead(uint8_t id, float voltage_deg, float *mechanical)
{
    uint32_t elapsed, stable_ms = 0u;
    float current, reference = 0.0f;

    if (!SVPWM_Set(id, voltage_deg, SVPWM_ALIGN_MOD)) return 0;
    for (elapsed = 10u; elapsed <= 1500u; elapsed += 10u) {
        Delay_ms(10u);
        if (!DriverOK() || TLE5012B_ReadAngle(id, &current) != TLE_OK ||
            !isfinite(current) || current < 0.0f || current >= 360.0f) return 0;
        if (elapsed <= 500u ||
            fabsf(Wrap360(current - reference + 180.0f) - 180.0f) > 0.25f) {
            reference = current;
            stable_ms = 0u;
        } else {
            stable_ms += 10u;
        }
        if (stable_ms >= 100u) {
            *mechanical = current;
            return 1;
        }
    }
    return 0;
}

int Motor_init(void)
{
    float zeros[2], angle90[2], delta, expected;
    int8_t signs[2];
    uint8_t id;

    if (!SVPWM_Init()) return 0;
    if (SVPWM_POLE_PAIRS == 0u || SVPWM_ALIGN_MOD == 0u ||
        SVPWM_ALIGN_MOD > 100u) return Fail();
    Delay_ms(10u); /* SPI2/CS沿用已有配置，补足编码器上电等待。 */
    if (TLE5012B_Init(0u) != TLE_OK || TLE5012B_Init(1u) != TLE_OK) //这里就初始化了磁编码器
        return Fail();

    expected = 90.0f / (float)SVPWM_POLE_PAIRS; /* 7极对数：约12.86机械度 */
    for (id = 0u; id < 2u; ++id) {
        /* 0度记录零点，保持通电直接切90度；中间不Stop、不扫描。 */
        if (!AlignAndRead(id, 0.0f, &zeros[id]) ||
            !AlignAndRead(id, 90.0f, &angle90[id])) return Fail();
				delta = angle90[id] - zeros[id];

				if (delta >= 180.0f) delta -= 360.0f;
				if (delta < -180.0f) delta += 360.0f;
				/*上面两个if用于处理如：直接相减：2.86 − 350 = −347.14°
				修正之后：−347.14 + 360 = +12.86°的情况*/
				
        /* 位移大小须在理论值的±10%内，不能把堵转/抖动当成方向。 */
        if (fabsf(fabsf(delta) - expected) > expected * 0.10f) return Fail();
        signs[id] = delta > 0.0f ? 1 : -1;
        SVPWM_Stop(id);/*校准完就停机，一个电机一个电机的校准*/
    }

    /* 两路都通过后才发布结果；零点仍是0度矢量下的读数。 */
    encoder_sign[0] = signs[0];
    encoder_sign[1] = signs[1];
    zero_point_1 = zeros[0];
    zero_point_2 = zeros[1];
    first_mechanical_angle = angle90[0];
    second_mechanical_angle = angle90[1];
    electrical1 = Wrap360((float)signs[0] * SVPWM_POLE_PAIRS * (angle90[0] - zeros[0]));
    electrical2 = Wrap360((float)signs[1] * SVPWM_POLE_PAIRS * (angle90[1] - zeros[1]));
    calibrated = 1u;
    return 1;
}

int SVPWM_Update(int8_t dir1, int8_t dir2,
                 uint16_t modulation_1, uint16_t modulation_2)
{
    if (!ready) return 0;
    if (!calibrated || (dir1 != 1 && dir1 != -1) ||
        (dir2 != 1 && dir2 != -1) ||
        modulation_1 > SVPWM_MAX_MOD || modulation_2 > SVPWM_MAX_MOD ||
        !DriverOK()) return Fail();

    /* 停止命令立即执行；正在使用的编码器只尝试一次，失败立即停双路。 */
    if (modulation_1 == 0u) SVPWM_Stop(0u);
    if (modulation_2 == 0u) SVPWM_Stop(1u);
    if ((modulation_1 != 0u &&
         TLE5012B_ReadAngle(0u, &first_mechanical_angle) != TLE_OK) ||
        (modulation_2 != 0u &&
         TLE5012B_ReadAngle(1u, &second_mechanical_angle) != TLE_OK))
        return Fail();

    /* 先读完所需的两路角度，再更新PWM，避免第二路读取失败前更新第一路。 */
    if (modulation_1 != 0u) {
        electrical1 = Wrap360((float)SVPWM_ENCODER_SIGN_1 * SVPWM_POLE_PAIRS *
                             (first_mechanical_angle - zero_point_1));
        if (!SVPWM_Set(0u, electrical1 + (float)dir1 * 90.0f, modulation_1))
            return Fail();
    }
    if (modulation_2 != 0u) {
        electrical2 = Wrap360((float)SVPWM_ENCODER_SIGN_2 * SVPWM_POLE_PAIRS *
                             (second_mechanical_angle - zero_point_2));
        if (!SVPWM_Set(1u, electrical2 + (float)dir2 * 90.0f, modulation_2))
            return Fail();
    }
    return 1;
}



/* 正式电流环的入口：先关两轮，再将TIM2/TIM4从零同步启动。
 * TIM2_CH1只产生内部OC1REF，不输出到PA0；PA0仍是DRV的EN。
 * ADC两rank总耗时=(28.5+12.5)*2=82个ADC时钟。
 * 在计数峰值前半个转换时长触发，使整个序列位于共同低侧窗口中央。 */
int SVPWM_StartCurrent(uint32_t guard_us, float *voltage_ratio)
{
    RCC_ClocksTypeDef clocks;
    TIM_OCInitTypeDef oc;
    uint32_t tim_hz, adc_hz, half_sequence, guard, wait;
    if (!ready || !calibrated || !DriverOK()) return 0;
    SVPWM_Stop(0u); SVPWM_Stop(1u);
    RCC_GetClocksFreq(&clocks);
    tim_hz = clocks.PCLK1_Frequency;
    if (clocks.PCLK1_Frequency != clocks.HCLK_Frequency) tim_hz *= 2u;
    adc_hz = clocks.ADCCLK_Frequency;
    if (adc_hz == 0u || guard_us > 20u) return 0;
    half_sequence = (41u * tim_hz + adc_hz - 1u) / adc_hz;
    guard = (tim_hz / 1000000u) * guard_us + 8u; /* 含定时器同步的数拍延迟 */
    if (half_sequence + guard >= period) return 0;
    current_compare_limit = (uint16_t)(period - half_sequence - guard);
    *voltage_ratio = (float)current_compare_limit / period;

    TIM_Cmd(TIM2, DISABLE);
    TIM_Cmd(TIM4, DISABLE);
    TIM4->SMCR &= ~TIM_SMCR_SMS;
    TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Enable);
    TIM_OCStructInit(&oc);
    oc.TIM_OCMode = TIM_OCMode_PWM2;
    oc.TIM_OutputState = TIM_OutputState_Disable;
    oc.TIM_Pulse = (uint16_t)(period - half_sequence);
    TIM_OC1Init(TIM2, &oc);
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_SetCompare2(TIM2, 0u); TIM_SetCompare3(TIM2, 0u); TIM_SetCompare4(TIM2, 0u);
    TIM_SetCompare2(TIM4, 0u); TIM_SetCompare3(TIM4, 0u); TIM_SetCompare4(TIM4, 0u);
    /* 强制从向上计数的0开始；UG只在两轮已高阻时发出。 */
    TIM2->CR1 &= ~(TIM_CR1_CMS | TIM_CR1_DIR);
    TIM4->CR1 &= ~(TIM_CR1_CMS | TIM_CR1_DIR);
    TIM2->CR1 |= TIM_CounterMode_CenterAligned1;
    TIM4->CR1 |= TIM_CounterMode_CenterAligned1;
    TIM_GenerateEvent(TIM2, TIM_EventSource_Update);
    TIM_GenerateEvent(TIM4, TIM_EventSource_Update);
    TIM_SetCounter(TIM2, 0u);
    TIM_SetCounter(TIM4, 0u);
    TIM_SelectInputTrigger(TIM4, TIM_TS_ITR1); /* TIM4的ITR1连接TIM2 */
    TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_Trigger);
    TIM_Cmd(TIM2, ENABLE); /* TRGO=CEN，硬件同时启动TIM4 */
    wait = 10000u;
    while ((TIM4->CR1 & TIM_CR1_CEN) == 0u)
        if (--wait == 0u) return Fail();
    TIM4->SMCR &= ~TIM_SMCR_SMS; /* 只需同步启动，不要每次采样都复位计数器 */
    TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_OC1Ref);
    PhaseEnable(0u, Bit_SET); PhaseEnable(1u, Bit_SET);
    running[0] = running[1] = 1u; /* 三相占空比0：全低侧零矢量，不是高阻 */
    return 1;
}

int SVPWM_ReadRotor(uint8_t id, float *mechanical_deg, float *electrical_deg)
{
    float mechanical, zero;
    if (!ready || !calibrated || id > 1u || !DriverOK()) return 0;
    if (TLE5012B_ReadAngle(id, &mechanical) != TLE_OK) return Fail();
    zero = id == 0u ? zero_point_1 : zero_point_2;
    *mechanical_deg = mechanical;
    *electrical_deg = Wrap360(encoder_sign[id] * SVPWM_POLE_PAIRS * (mechanical - zero));
    if (id == 0u) { first_mechanical_angle = mechanical; electrical1 = *electrical_deg; }
    else { second_mechanical_angle = mechanical; electrical2 = *electrical_deg; }
    return 1;
}

/* 用相同的公共偏移把最小占空比移至0，不改变三相线电压。
 * 相比对称零矢量分配，这样把可用零矢量时间留给低侧采样。
 * PI已按current_compare_limit/period限制电压，不能再额外缩放其输出。 */
int SVPWM_WriteVoltage(uint8_t id, float alpha_v, float beta_v, float bus_v)
{
    float a, b, c, minimum;
    uint16_t ca, cb, cc;
    TIM_TypeDef *tim;
    if (!ready || !calibrated || id > 1u || !running[id] ||
        current_compare_limit == 0u || !DriverOK() || !(bus_v > 0.0f) ||
        !isfinite(bus_v) || !isfinite(alpha_v) || !isfinite(beta_v)) return Fail();
    a = alpha_v / bus_v;
    b = (-0.5f * alpha_v + 0.86602540378f * beta_v) / bus_v;
    c = (-0.5f * alpha_v - 0.86602540378f * beta_v) / bus_v;
    minimum = fminf(a, fminf(b, c));
    a -= minimum; b -= minimum; c -= minimum;
    if (fmaxf(a, fmaxf(b, c)) > (float)current_compare_limit / period + 0.00001f)
        return Fail();
    ca = Compare(a); cb = Compare(b); cc = Compare(c);
    /* 仅吸收舍入导致的1个计数误差，不能突破ADC窗口。 */
    if (ca > current_compare_limit) ca = current_compare_limit;
    if (cb > current_compare_limit) cb = current_compare_limit;
    if (cc > current_compare_limit) cc = current_compare_limit;
    tim = timer[id];
    TIM_UpdateDisableConfig(tim, ENABLE);
    TIM_SetCompare2(tim, ca); TIM_SetCompare3(tim, cb); TIM_SetCompare4(tim, cc);
    TIM_UpdateDisableConfig(tim, DISABLE);
    return 1;
}
