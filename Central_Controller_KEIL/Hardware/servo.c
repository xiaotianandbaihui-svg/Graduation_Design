#include "servo.h"
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"

static uint16_t frame_us; /* 0表示尚未初始化 */

Servo_Result Servo_Init(uint16_t period_us)
{
    RCC_ClocksTypeDef clocks;
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef base;
    TIM_OCInitTypeDef oc;
    uint32_t timer_hz;

    if (period_us < 2u) return SERVO_ARGUMENT_ERROR;

    RCC_GetClocksFreq(&clocks);
    timer_hz = clocks.PCLK1_Frequency;
    /* APB1未分频时TIM3时钟=PCLK1；分频时TIM3时钟=2*PCLK1。 */
    if (clocks.PCLK1_Frequency != clocks.HCLK_Frequency) timer_hz *= 2u;
    if (timer_hz < 1000000u || timer_hz % 1000000u != 0u)
        return SERVO_CLOCK_ERROR;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                          RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    /* 先预置输出低电平，再切为普通推挽，配置期间不产生控制脉冲。 */
    GPIO_ResetBits(GPIOA, GPIO_Pin_6 | GPIO_Pin_7);
    GPIO_ResetBits(GPIOB, GPIO_Pin_0 | GPIO_Pin_1);
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOA, &gpio);
    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_Init(GPIOB, &gpio);

    TIM_DeInit(TIM3);
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, DISABLE);

    TIM_TimeBaseStructInit(&base);
    base.TIM_Prescaler = (uint16_t)(timer_hz / 1000000u - 1u);
    base.TIM_Period = period_us - 1u;
    base.TIM_CounterMode = TIM_CounterMode_Up;
    base.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM3, &base);

    TIM_OCStructInit(&oc);
    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    oc.TIM_Pulse = 0u;
    TIM_OC1Init(TIM3, &oc);
    TIM_OC2Init(TIM3, &oc);
    TIM_OC3Init(TIM3, &oc);
    TIM_OC4Init(TIM3, &oc);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_GenerateEvent(TIM3, TIM_EventSource_Update);
    TIM_ClearFlag(TIM3, TIM_FLAG_Update);

    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOA, &gpio);
    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_Init(GPIOB, &gpio);

    frame_us = period_us;
    TIM_Cmd(TIM3, ENABLE);
    return SERVO_OK;
}

Servo_Result Servo_SetPulse(uint8_t id, uint16_t pulse_us)
{
    if (frame_us == 0u) return SERVO_NOT_READY;
    if (id > 3u || pulse_us >= frame_us) return SERVO_ARGUMENT_ERROR;

    switch (id) {
    case 0u: TIM_SetCompare1(TIM3, pulse_us); break;
    case 1u: TIM_SetCompare2(TIM3, pulse_us); break;
    case 2u: TIM_SetCompare3(TIM3, pulse_us); break;
    default: TIM_SetCompare4(TIM3, pulse_us); break;
    }
    return SERVO_OK;
}

Servo_Result Servo_SetAngle(uint8_t id, float angle_deg)
{
    float pulse_us;

    if (frame_us == 0u) return SERVO_NOT_READY;
    if (id > 3u || frame_us <= SERVO_MAX_PULSE_US ||
        !(angle_deg >= 0.0f && angle_deg <= SERVO_MAX_ANGLE_DEG))
        return SERVO_ARGUMENT_ERROR;

    pulse_us = (float)SERVO_MIN_PULSE_US +
               (float)(SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) *
               (angle_deg / SERVO_MAX_ANGLE_DEG);

    return Servo_SetPulse(id, (uint16_t)(pulse_us + 0.5f)); /* 四舍五入到整数微秒 */
}

Servo_Result Servo_SetAngles(const float angle_deg[4])
{
    uint16_t pulse[4];
    unsigned i;
    if (frame_us == 0u) return SERVO_NOT_READY;
    if (!angle_deg || frame_us <= SERVO_MAX_PULSE_US) return SERVO_ARGUMENT_ERROR;
    for (i = 0; i < 4; ++i) {
        float a = angle_deg[i];
        if (!(a >= 0.0f && a <= SERVO_MAX_ANGLE_DEG)) return SERVO_ARGUMENT_ERROR;
        pulse[i] = (uint16_t)((float)SERVO_MIN_PULSE_US +
                   (float)(SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) *
                   a / SERVO_MAX_ANGLE_DEG + 0.5f);
    }
    /* 暂停预装载转移，避免四路写到一半恰逢PWM周期边界。 */
    TIM_UpdateDisableConfig(TIM3, ENABLE);
    TIM_SetCompare1(TIM3, pulse[0]);
    TIM_SetCompare2(TIM3, pulse[1]);
    TIM_SetCompare3(TIM3, pulse[2]);
    TIM_SetCompare4(TIM3, pulse[3]);
    TIM_UpdateDisableConfig(TIM3, DISABLE);
    return SERVO_OK;
}
