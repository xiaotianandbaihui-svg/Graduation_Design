#include "six_step.h"
#include "current_sense.h"
#include "tle5012b.h"
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include <math.h>

/* 使用应用已有的微秒延时函数；参数声明须与其实际定义一致。 */
extern void Delay_us(uint32_t us);

#define PWM_COUNTS  (72000000u / SIX_STEP_PWM_HZ)
#define NO_VECTOR   255u

/* 直接使用标准库，不另设motor_port层。 */
static TIM_TypeDef * const pwm_timer[2] = {TIM2, TIM4};
static GPIO_TypeDef * const low_port[2][3] = {
    {GPIOB, GPIOB, GPIOB}, {GPIOB, GPIOB, GPIOA}
};
static const uint16_t low_pin[2][3] = {
    {GPIO_Pin_12, GPIO_Pin_11, GPIO_Pin_10},
    {GPIO_Pin_6, GPIO_Pin_3, GPIO_Pin_15}
};
static const uint16_t pwm_channel[3] = {
    TIM_Channel_2, TIM_Channel_3, TIM_Channel_4
};

/* 矢量  0     1     2     3     4     5
 *       A+B-  A+C-  B+C-  B+A-  C+A-  C+B-
 * 高侧相：INH=PWM、INL=0；低侧相：INH=0、INL=1；剩余相：0/0。
 * 依据DRV832x手册SLVSDJ3D，8.3.1.1.1，表8-2（6PWM真值表）。 */
static const uint8_t high_phase[6] = {0, 0, 1, 1, 2, 2};
static const uint8_t low_phase[6]  = {1, 2, 2, 0, 0, 1};

static struct {
    SixStep_Calibration cal;
    uint8_t calibrated;
    uint8_t vector;
} motor[2];
static uint8_t ready;
static uint8_t faulted;

static float Wrap360(float angle)
{
    angle = fmodf(angle, 360.0f);
    if (angle < 0.0f) angle += 360.0f;
    return angle >= 360.0f ? 0.0f : angle;
}

/* 相邻机械角之差，处理359度->0度，结果[-180,180)。 */
static float AngleDifference(float now, float before)
{
    return Wrap360(now - before + 180.0f) - 180.0f;
}

static void GPIO_Mode(GPIO_TypeDef *port, uint16_t pins, GPIOMode_TypeDef mode)
{
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = pins;
    gpio.GPIO_Mode = mode;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(port, &gpio);
}

static void SetCompare(uint8_t id, uint8_t phase, uint16_t compare)
{
    if (phase == 0u) TIM_SetCompare2(pwm_timer[id], compare);
    else if (phase == 1u) TIM_SetCompare3(pwm_timer[id], compare);
    else TIM_SetCompare4(pwm_timer[id], compare);
}

/* 强制无效电平立即生效；仅CCR写0会被预装载推迟，不能用作立即关断。 */
static void AllOff(uint8_t id)
{
    uint8_t phase;
    TIM_ForcedOC2Config(pwm_timer[id], TIM_ForcedAction_InActive);
    TIM_ForcedOC3Config(pwm_timer[id], TIM_ForcedAction_InActive);
    TIM_ForcedOC4Config(pwm_timer[id], TIM_ForcedAction_InActive);
    for (phase = 0u; phase < 3u; ++phase) {
        GPIO_ResetBits(low_port[id][phase], low_pin[id][phase]);
        SetCompare(id, phase, 0u);
    }
    motor[id].vector = NO_VECTOR;
}

void SixStep_Stop(uint8_t id)
{
    if (ready && id < 2u) AllOff(id);
}

void SixStep_StopAll(void)
{
    if (ready) {
        /* 先把所有输入拉低，再让DRV8323H进入睡眠。 */
        AllOff(0u);
        AllOff(1u);
        GPIO_ResetBits(GPIOA, GPIO_Pin_0);
    }
    faulted = 1u;
}

static SixStep_Result Fail(SixStep_Result result)
{
    SixStep_StopAll();
    return result;
}

static SixStep_Result CheckDriver(void)
{
    if (!ready || faulted) return SIX_STEP_NOT_READY;
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) == Bit_RESET ||
        GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5) == Bit_RESET)
        return Fail(SIX_STEP_DRIVER_FAULT);
    return SIX_STEP_OK;
}

static SixStep_Result ReadAngle(uint8_t id, float *angle)
{
    if (TLE5012B_ReadAngle(id, angle) != TLE_OK ||
        !(*angle >= 0.0f && *angle < 360.0f))
        return Fail(SIX_STEP_ENCODER_ERROR);
    return SIX_STEP_OK;
}

/* 只在标定/载入标定时执行；运行期间不读STAT清除传感器故障。 */
static SixStep_Result CheckEncoder(uint8_t id)
{
    uint16_t mod1, mod2;
    if (TLE5012B_Init(id) != TLE_OK ||
        TLE5012B_ReadReg(id, TLE_REG_MOD1, &mod1) != TLE_OK ||
        TLE5012B_ReadReg(id, TLE_REG_MOD2, &mod2) != TLE_OK)
        return Fail(SIX_STEP_ENCODER_ERROR);
    /* 精简版ReadAngle按整圈换算，所以ANGRANGE必须为128，DSPU不能暂停。 */
    if ((mod1 & 0x0004u) || ((mod2 >> 4) & 0x07FFu) != 128u)
        return Fail(SIX_STEP_ENCODER_ERROR);
    return SIX_STEP_OK;
}

static void InitPWM(TIM_TypeDef *timer)
{
    TIM_TimeBaseInitTypeDef base;
    TIM_OCInitTypeDef oc;
    TIM_DeInit(timer);
    TIM_TimeBaseStructInit(&base);
    base.TIM_Prescaler = 0u;
    base.TIM_Period = PWM_COUNTS - 1u;
    base.TIM_CounterMode = TIM_CounterMode_Up;
    base.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(timer, &base);
    TIM_OCStructInit(&oc);
    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    oc.TIM_Pulse = 0u;
    TIM_OC2Init(timer, &oc);
    TIM_OC3Init(timer, &oc);
    TIM_OC4Init(timer, &oc);
    TIM_OC2PreloadConfig(timer, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(timer, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(timer, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(timer, ENABLE);
    TIM_GenerateEvent(timer, TIM_EventSource_Update);
    TIM_ClearFlag(timer, TIM_FLAG_Update);
    TIM_Cmd(timer, ENABLE);
}

SixStep_Result SixStep_Init(void)
{
    RCC_ClocksTypeDef clocks;
    uint8_t id;
    if (ready) SixStep_StopAll();
    ready = 0u;
    faulted = 1u;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                          RCC_APB2Periph_AFIO, ENABLE);
    GPIO_ResetBits(GPIOA, GPIO_Pin_0);
    GPIO_Mode(GPIOA, GPIO_Pin_0, GPIO_Mode_Out_PP);
    RCC_GetClocksFreq(&clocks);
    if (clocks.HCLK_Frequency != 72000000u || clocks.PCLK1_Frequency != 36000000u)
        return SIX_STEP_CLOCK_ERROR;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM4, ENABLE);
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, DISABLE);
    GPIO_PinRemapConfig(GPIO_Remap_TIM4, DISABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    GPIO_ResetBits(GPIOB, GPIO_Pin_4);
    GPIO_Mode(GPIOB, GPIO_Pin_4, GPIO_Mode_Out_PP);

    GPIO_ResetBits(GPIOA, GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_15);
    GPIO_ResetBits(GPIOB, GPIO_Pin_3 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 |
                         GPIO_Pin_12 | GPIO_Pin_11 | GPIO_Pin_10 | GPIO_Pin_6);
    GPIO_Mode(GPIOA, GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_15,
              GPIO_Mode_Out_PP);
    GPIO_Mode(GPIOB, GPIO_Pin_3 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 |
                    GPIO_Pin_12 | GPIO_Pin_11 | GPIO_Pin_10 | GPIO_Pin_6,
              GPIO_Mode_Out_PP);
    GPIO_Mode(GPIOA, GPIO_Pin_8, GPIO_Mode_IPU);
    GPIO_Mode(GPIOB, GPIO_Pin_5, GPIO_Mode_IPU);
    InitPWM(TIM2);
    InitPWM(TIM4);
    for (id = 0u; id < 2u; ++id) {
        motor[id].calibrated = 0u;
        AllOff(id);
    }
    GPIO_Mode(GPIOA, GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3, GPIO_Mode_AF_PP);
    GPIO_Mode(GPIOB, GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9, GPIO_Mode_AF_PP);
    ready = 1u;
    Delay_us(2000u);                  /* 确保此前ENABLE低已进入睡眠 */
    GPIO_SetBits(GPIOA, GPIO_Pin_0);  /*使能驱动芯片*/
    Delay_us(2000u);                  /* DRV832x的tWAKE最大1ms，留到2ms */
    faulted = 0u;
    if (CheckDriver() != SIX_STEP_OK) return SIX_STEP_DRIVER_FAULT;
    if (!CurrentSense_Init()) return Fail(SIX_STEP_CURRENT_SENSE_ERROR);
    return SIX_STEP_OK;
}

static SixStep_Result ApplyVector(uint8_t id, uint8_t vector, uint16_t duty)
{
    uint8_t high = high_phase[vector], low = low_phase[vector];
    /* duty=1000时CCR=ARR+1，PWM1在整个计数周期都有效，实现100%。 */
    uint16_t compare = (uint16_t)((uint32_t)duty * PWM_COUNTS / 1000u);
    SixStep_Result result = CheckDriver();
    if (result != SIX_STEP_OK) return result;
    if (motor[id].vector == vector) {
        SetCompare(id, high, compare);   /* 同扇区：下个PWM周期更新占空比 */
        return SIX_STEP_OK;
    }

    AllOff(id);
    Delay_us(2u);                       /* 换扇区先全部关闭；芯片另有硬件互锁 */
    SetCompare(id, high, compare);
    TIM_GenerateEvent(pwm_timer[id], TIM_EventSource_Update); /* 装载新CCR */
    TIM_ClearFlag(pwm_timer[id], TIM_FLAG_Update);
    result = CheckDriver();
    if (result != SIX_STEP_OK) return result;
    GPIO_SetBits(low_port[id][low], low_pin[id][low]);
    TIM_SelectOCxM(pwm_timer[id], pwm_channel[high], TIM_OCMode_PWM1);
    /* 标准库TIM_SelectOCxM会关闭CCxE，所以必须显式重新开启该通道。 */
    TIM_CCxCmd(pwm_timer[id], pwm_channel[high], TIM_CCx_Enable);
    motor[id].vector = vector;
    return SIX_STEP_OK;
}

SixStep_Result SixStep_SetCalibration(uint8_t id, const SixStep_Calibration *cal)
{
    SixStep_Result result = CheckDriver();
    if (result != SIX_STEP_OK) return result;
    if (id > 1u || cal == 0 || !(cal->zero_deg >= 0.0f && cal->zero_deg < 360.0f) ||
        (cal->encoder_sign != 1 && cal->encoder_sign != -1) ||
        motor[0].vector != NO_VECTOR || motor[1].vector != NO_VECTOR)
        return Fail(SIX_STEP_ARGUMENT_ERROR);
    result = CheckEncoder(id);
    if (result != SIX_STEP_OK) return result;
    motor[id].cal = *cal;
    motor[id].calibrated = 1u;
    return SIX_STEP_OK;
}

/* 固定矢量对齐，并取最后20次读数的跨零点平均，检查摆动范围<=1度。 */
static SixStep_Result HoldVector(uint8_t id, uint8_t vector, uint16_t duty,
                               uint16_t hold_ms, float *angle)
{
    uint16_t i;
    float now, anchor = 0.0f, delta, sum = 0.0f, min = 0.0f, max = 0.0f;
    SixStep_Result result = ApplyVector(id, vector, duty);
    if (result != SIX_STEP_OK) return result;
    for (i = 0u; i < hold_ms; ++i) {
        Delay_us(1000u);
        result = CheckDriver();
        if (result != SIX_STEP_OK) return result;
        result = ReadAngle(id, &now);
        if (result != SIX_STEP_OK) return result;
        if (i == hold_ms - 20u) anchor = now;
        if (i >= hold_ms - 20u) {
            delta = AngleDifference(now, anchor);
            sum += delta;
            if (delta < min) min = delta;
            if (delta > max) max = delta;
        }
    }
    if (max - min > 1.0f) return Fail(SIX_STEP_CALIBRATION_ERROR);
    *angle = Wrap360(anchor + sum / 20.0f);
    return SIX_STEP_OK;
}

SixStep_Result SixStep_Calibrate(uint8_t id, uint16_t duty, SixStep_Calibration *cal)
{
    uint8_t step;
    int8_t sign = 0;
    float before, now = 0.0f, movement;
    const float expected = 360.0f / (SIX_STEP_POLE_PAIRS * 6.0f);
    SixStep_Result result = CheckDriver();
    if (result != SIX_STEP_OK) return result;
    if (id > 1u || cal == 0 || duty == 0u || duty > SIX_STEP_MAX_ALIGN_DUTY ||
        motor[0].vector != NO_VECTOR || motor[1].vector != NO_VECTOR)
        return Fail(SIX_STEP_ARGUMENT_ERROR);
    motor[id].calibrated = 0u;
    result = CheckEncoder(id);
    if (result != SIX_STEP_OK) return result;
    result = HoldVector(id, 0u, duty, 600u, &before);
    if (result != SIX_STEP_OK) return result;
    for (step = 1u; step <= 6u; ++step) {
        result = HoldVector(id, (uint8_t)(step % 6u), duty, 300u, &now);
        if (result != SIX_STEP_OK) return result;
        movement = AngleDifference(now, before);
        if (step == 1u) sign = movement > 0.0f ? 1 : -1;
        movement *= sign;
        if (movement < expected * 0.5f || movement > expected * 1.5f)
            return Fail(SIX_STEP_CALIBRATION_ERROR);
        before = now;
    }
    SixStep_Stop(id);
    motor[id].cal.zero_deg = now;
    motor[id].cal.encoder_sign = sign;
    motor[id].calibrated = 1u;
    *cal = motor[id].cal;
    return SIX_STEP_OK;
}

SixStep_Result SixStep_Update(uint8_t id, uint16_t duty, SixStep_Direction direction)
{
    float mechanical, electrical, target;
    uint8_t vector;
    SixStep_Result result = CheckDriver();
    if (result != SIX_STEP_OK) return result;
    if (id > 1u || duty > SIX_STEP_MAX_DUTY ||
        (direction != SIX_STEP_FORWARD && direction != SIX_STEP_REVERSE))
        return Fail(SIX_STEP_ARGUMENT_ERROR);
    if (duty == 0) {
        SixStep_Stop(id);
        return SIX_STEP_OK;
    }
    if (!motor[id].calibrated) {
        SixStep_Stop(id);
        return SIX_STEP_NOT_CALIBRATED;
    }
    result = ReadAngle(id, &mechanical);
    if (result != SIX_STEP_OK) return result;
    /* mechanical直接来自现有TLE5012B_ReadAngle，范围[0,360)度。
     * 先减机械零位、统一编码器方向，再乘7得到电角度并归一化。 */
    electrical = Wrap360((float)motor[id].cal.encoder_sign * SIX_STEP_POLE_PAIRS *
                         (mechanical - motor[id].cal.zero_deg));
    /* 零位是矢量0的静态对齐位置；选相对转子约+/-90度的电压矢量。
     * +30后除60，实现选择最近矢量，量化误差约+/-30度。
     * 这是电压矢量选择，不保证电流与转子严格正交。 */
    target = electrical + (direction == SIX_STEP_FORWARD ? 90.0f : -90.0f);
    vector = (uint8_t)(Wrap360(target + 30.0f) / 60.0f);
    return ApplyVector(id, vector, duty);
}
