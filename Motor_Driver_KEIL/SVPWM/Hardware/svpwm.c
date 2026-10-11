#include "svpwm.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "tle5012b.h"
#include <math.h>
#include "Delay.h"

float zero_point_1=0,zero_point_2=0;//记录两个电机在磁编码器特定安装下的A相角度
float first_mechanical_angle=0,second_mechanical_angle=0;//记录电机当前机械角度
float electrical1=0,electrical2=0;//记录当前电机的电角度 [0°,360°)
static TIM_TypeDef * const timer[2] = {TIM2, TIM4};
static uint16_t period;
static uint8_t ready, running[2];

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
        GPIO_WriteBit(GPIOB, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14, state);
    } else {
        GPIO_WriteBit(GPIOB, GPIO_Pin_15 | GPIO_Pin_3, state);
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
    GPIO_ResetBits(GPIOB, GPIO_Pin_0);
    ready = 0u;
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

int SVPWM_Init(void)
{
    RCC_ClocksTypeDef clocks;
    uint32_t tim_hz, counts;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                          RCC_APB2Periph_AFIO, ENABLE);
    /* 先预置ODR低，再切输出模式，避免切换时出现高电平。 */
    GPIO_ResetBits(GPIOB, GPIO_Pin_0);
    PinMode(GPIOB, GPIO_Pin_0, GPIO_Mode_Out_PP);
    ready = running[0] = running[1] = 0u;
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, DISABLE);
    GPIO_PinRemapConfig(GPIO_Remap_TIM4, DISABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    PhaseEnable(0u, Bit_RESET);
    PhaseEnable(1u, Bit_RESET);
    PinMode(GPIOB, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 |
                   GPIO_Pin_15 | GPIO_Pin_3, GPIO_Mode_Out_PP);
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
    GPIO_SetBits(GPIOB, GPIO_Pin_0);
    Delay_us(2000u); /* 等待DRV唤醒，INL仍全为低。 */
    ready = 1u;
    return DriverOK() ? 1 : Fail();
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
		angle=Wrap360(voltage_deg);
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
void Motor_init()
{
	if (!SVPWM_Set(0, 0.0f, SVPWM_MAX_MOD/2) ||!SVPWM_Set(1, 0.0f, SVPWM_MAX_MOD/2)) return;
	Delay_ms(500);
	while(TLE5012B_ReadAngle(0,&zero_point_1)!=TLE_OK);
	while(TLE5012B_ReadAngle(1,&zero_point_2)!=TLE_OK);
	SVPWM_Stop(0);
	SVPWM_Stop(1);
	
}

void SVPWM_Update(int8_t dir1,int8_t dir2,uint16_t modulation_1,uint16_t modulation_2)
{
	while(TLE5012B_ReadAngle(0,&first_mechanical_angle)!=TLE_OK);
	while(TLE5012B_ReadAngle(1,&second_mechanical_angle)!=TLE_OK);
	
	electrical1 = Wrap360(7.0f *(first_mechanical_angle - zero_point_1)+dir1*90);
	SVPWM_Set(0,electrical1,modulation_1);
	
	electrical2 = Wrap360(7.0f *(second_mechanical_angle - zero_point_2)+dir2*90);
	SVPWM_Set(1,electrical2,modulation_2);
}
