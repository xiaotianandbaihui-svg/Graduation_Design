#include "motor_current.h"
#include "current_loop.h"
#include "current_sense.h"
#include "svpwm.h"
#include "tle5012b.h"
#include "stm32f10x.h"
#include "cycle_counter.h"
#include <math.h>

static CurrentLoop loops[2];
static CurrentLoop_Feedback feedback[2];
static volatile MotorCurrent_Status state;
static volatile float target_iq[2], rpm[2];
static float previous_mechanical[2];
static uint32_t previous_tick[2], cpu_hz, period_cycles, last_entry;
static volatile uint32_t last_cycles, command_age;

static float AngleDifference(float now, float before)
{
    float delta = now - before;
    if (delta > 180.0f) delta -= 360.0f;
    if (delta < -180.0f) delta += 360.0f;
    return delta;
}

void MotorCurrent_Stop(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    TIM_Cmd(TIM1, DISABLE);
    NVIC_DisableIRQ(TIM1_UP_IRQn);
    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    NVIC_ClearPendingIRQ(TIM1_UP_IRQn);
    SVPWM_Stop(0u); SVPWM_Stop(1u);
    CurrentLoop_Reset(&loops[0]); CurrentLoop_Reset(&loops[1]);
    target_iq[0] = target_iq[1] = 0.0f;
    command_age = 0u;
    if (state >= MOTOR_CURRENT_STOPPED) state = MOTOR_CURRENT_STOPPED;
    __set_PRIMASK(mask);
}

static int Fault(MotorCurrent_Status error)
{
    state = error;
    MotorCurrent_Stop();
    return 0;
}

MotorCurrent_Status MotorCurrent_GetStatus(void) { return state; }
uint32_t MotorCurrent_LastStepUs(void)
{
    return cpu_hz ? last_cycles / (cpu_hz / 1000000u) : 0u;
}

int MotorCurrent_Init(void)
{
    RCC_ClocksTypeDef clocks;
    TIM_TimeBaseInitTypeDef timer;
    NVIC_InitTypeDef irq;
    uint32_t tim_hz;
    uint8_t id;
    CurrentLoop_Config config = {
        MOTOR_CURRENT_KP_D, MOTOR_CURRENT_KI_D,
        MOTOR_CURRENT_KP_Q, MOTOR_CURRENT_KI_Q,
        1.0f / MOTOR_CURRENT_HZ, MOTOR_CURRENT_LIMIT_A,
        MOTOR_CURRENT_VOLTAGE_RATIO
    };
    MotorCurrent_Stop();
    state = MOTOR_CURRENT_STOPPED;
    if (MOTOR_CURRENT_HZ == 0u || 1000000u % MOTOR_CURRENT_HZ != 0u ||
        MOTOR_CURRENT_HZ > SVPWM_PWM_HZ ||
        !(MOTOR_CURRENT_LIMIT_A > 0.0f && MOTOR_CURRENT_LIMIT_A < MOTOR_CURRENT_TRIP_A) ||
        !(MOTOR_CURRENT_BUS_VOLTAGE_V > 0.0f) ||
        MOTOR_CURRENT_COMMAND_TIMEOUT_MS == 0u)
        return Fault(MOTOR_CURRENT_CONFIG_ERROR);
    SystemCoreClockUpdate();
    RCC_GetClocksFreq(&clocks);
    cpu_hz = clocks.HCLK_Frequency;
    period_cycles = cpu_hz / MOTOR_CURRENT_HZ;
    tim_hz = clocks.PCLK2_Frequency;
    if (clocks.PCLK2_Frequency != clocks.HCLK_Frequency) tim_hz *= 2u;
    if (tim_hz % 1000000u != 0u) return Fault(MOTOR_CURRENT_CONFIG_ERROR);
    CycleCounter_Enable();
    for (id = 0u; id < 2u; ++id) {
        if (!CurrentLoop_Init(&loops[id], &config)) return Fault(MOTOR_CURRENT_CONFIG_ERROR);
        feedback[id].bus_voltage_v = MOTOR_CURRENT_BUS_VOLTAGE_V;
        rpm[id] = 0.0f;
    }
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    TIM_DeInit(TIM1);
    TIM_TimeBaseStructInit(&timer);
    timer.TIM_Prescaler = (uint16_t)(tim_hz / 1000000u - 1u);
    timer.TIM_Period = 1000000u / MOTOR_CURRENT_HZ - 1u;
    TIM_TimeBaseInit(TIM1, &timer);
    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
    irq.NVIC_IRQChannel = TIM1_UP_IRQn;
    irq.NVIC_IRQChannelPreemptionPriority = 0u;
    irq.NVIC_IRQChannelSubPriority = 0u;
    irq.NVIC_IRQChannelCmd = DISABLE;
    NVIC_Init(&irq);
    return 1;
}

int MotorCurrent_Start(void)
{
    float available_ratio, speed_dps;
    uint8_t id;
    if (state == MOTOR_CURRENT_RUNNING) return 1;
    if (state < MOTOR_CURRENT_STOPPED || period_cycles == 0u) return 0;
    /* 0增益只是未整定占位，不能误认为已经具备电流闭环。 */
    for (id = 0u; id < 2u; ++id) {
        if ((loops[id].d.kp == 0.0f && loops[id].d.ki_step == 0.0f) ||
            (loops[id].q.kp == 0.0f && loops[id].q.ki_step == 0.0f))
            return Fault(MOTOR_CURRENT_CONFIG_ERROR);
        CurrentLoop_Reset(&loops[id]);
        /* 开中断之前取得初始角度/轮速；以后只有TIM1中断访问SPI。 */
        if (!SVPWM_ReadRotor(id, &previous_mechanical[id], &feedback[id].electrical_deg))
            return Fault(MOTOR_CURRENT_ENCODER_ERROR);
        previous_tick[id] = CycleCounter_Now();
        if (TLE5012B_ReadAngularSpeed(id, &speed_dps) != TLE_OK)
            return Fault(MOTOR_CURRENT_ENCODER_ERROR);
        rpm[id] = speed_dps * SVPWM_GetEncoderSign(id) / 6.0f;
        target_iq[id] = 0.0f;
    }
    if (!SVPWM_StartCurrent(MOTOR_CURRENT_SAMPLE_GUARD_US, &available_ratio))
        return Fault(MOTOR_CURRENT_CONTROL_ERROR);
    for (id = 0u; id < 2u; ++id)
        loops[id].voltage_ratio = fminf(MOTOR_CURRENT_VOLTAGE_RATIO, available_ratio);
    command_age = 0u;
    last_cycles = 0u;
    last_entry = CycleCounter_Now();
    TIM_SetCounter(TIM1, 0u);
    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    NVIC_ClearPendingIRQ(TIM1_UP_IRQn);
    state = MOTOR_CURRENT_RUNNING;
    NVIC_EnableIRQ(TIM1_UP_IRQn);
    TIM_Cmd(TIM1, ENABLE);
    return 1;
}

int MotorCurrent_SetCommand(int16_t left, int16_t right)
{
    float iq0, iq1;
    uint32_t mask;
    if (left < -1000 || left > 1000 || right < -1000 || right > 1000) return 0;
    /* 正指令要求正电磁转矩；星点向外约定下对应负Iq。 */
    iq0 = -(float)left * (MOTOR_CURRENT_LIMIT_A / 1000.0f);
    iq1 = -(float)right * (MOTOR_CURRENT_LIMIT_A / 1000.0f);
    mask = __get_PRIMASK();
    __disable_irq();
    if (state != MOTOR_CURRENT_RUNNING) { __set_PRIMASK(mask); return 0; }
    target_iq[0] = iq0; target_iq[1] = iq1;
    command_age = 0u;
    __set_PRIMASK(mask);
    return 1;
}

int MotorCurrent_ReadSpeed(float *left_rpm, float *right_rpm)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (state != MOTOR_CURRENT_RUNNING) { __set_PRIMASK(mask); return 0; }
    *left_rpm = rpm[0]; *right_rpm = rpm[1];
    __set_PRIMASK(mask);
    return 1;
}

static int Step(void)
{
    CurrentSense_Raw raw;
    CurrentSense_Phase phase;
    CurrentLoop_Voltage voltage[2];
    float mechanical, delta, electrical_dps[2], speed;
    uint32_t tick[2], dt, sampled;
    uint8_t id;
    if (++command_age > (MOTOR_CURRENT_COMMAND_TIMEOUT_MS * MOTOR_CURRENT_HZ + 999u) / 1000u)
        return Fault(MOTOR_CURRENT_COMMAND_TIMEOUT);
    for (id = 0u; id < 2u; ++id) {
        if (!SVPWM_ReadRotor(id, &mechanical, &feedback[id].electrical_deg))
            return Fault(MOTOR_CURRENT_ENCODER_ERROR);
        tick[id] = CycleCounter_Now();
        dt = tick[id] - previous_tick[id]; /* 无符号差值允许CYCCNT自然回绕 */
        if (dt == 0u) return Fault(MOTOR_CURRENT_OVERRUN);
        delta = AngleDifference(mechanical, previous_mechanical[id]);
        electrical_dps[id] = delta * ((float)cpu_hz / dt) * SVPWM_GetEncoderSign(id);
        speed = electrical_dps[id] / 6.0f;
        rpm[id] += 0.2f * (speed - rpm[id]); /* 仅轮速反馈滤波，不滤波电流 */
        electrical_dps[id] *= SVPWM_POLE_PAIRS;
        previous_mechanical[id] = mechanical;
        previous_tick[id] = tick[id];
    }
    if (!CurrentSense_ReadSynchronized(&raw)) return Fault(MOTOR_CURRENT_SAMPLE_ERROR);
    sampled = CycleCounter_Now();
    for (id = 0u; id < 2u; ++id) {
        if (!CurrentSense_Convert(id, &raw, &phase)) return Fault(MOTOR_CURRENT_SAMPLE_ERROR);
        if (fabsf(phase.ia_a) >= MOTOR_CURRENT_TRIP_A ||
            fabsf(phase.ib_a) >= MOTOR_CURRENT_TRIP_A ||
            fabsf(phase.ic_a) >= MOTOR_CURRENT_TRIP_A) return Fault(MOTOR_CURRENT_OVERCURRENT);
        feedback[id].ia_a = phase.ia_a;
        feedback[id].ib_a = phase.ib_a;
        /* SPI角度与ADC并非硬件同时采样，按转速外推到ADC完成附近。
         * 仍包含编码器自身滤波延迟及几us的ADC误差，高速时需实测相位。 */
        feedback[id].electrical_deg += electrical_dps[id] *
                                      ((float)(uint32_t)(sampled - tick[id]) / cpu_hz);
    }
    /* 两轮都算完、且没有超期，再把电压交给原有PWM接口。 */
    for (id = 0u; id < 2u; ++id)
        if (!CurrentLoop_Update(&loops[id], &feedback[id], target_iq[id], &voltage[id]))
            return Fault(MOTOR_CURRENT_CONTROL_ERROR);
    if ((uint32_t)(CycleCounter_Now() - last_entry) >= period_cycles)
        return Fault(MOTOR_CURRENT_OVERRUN);
    for (id = 0u; id < 2u; ++id)
        if (!SVPWM_WriteVoltage(id, voltage[id].alpha_v, voltage[id].beta_v,
                               MOTOR_CURRENT_BUS_VOLTAGE_V)) return Fault(MOTOR_CURRENT_CONTROL_ERROR);
    return 1;
}

void TIM1_UP_IRQHandler(void)
{
    uint32_t began;
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == RESET) return;
    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    if (state != MOTOR_CURRENT_RUNNING) return;
    began = CycleCounter_Now();
    if ((uint32_t)(began - last_entry) > 2u * period_cycles) {
        Fault(MOTOR_CURRENT_OVERRUN); /* 严重迟到，不能把旧采样重复积分 */
        return;
    }
    last_entry = began;
    Step();
    last_cycles = CycleCounter_Now() - began;
    if (state == MOTOR_CURRENT_RUNNING &&
        (last_cycles >= period_cycles || TIM_GetFlagStatus(TIM1, TIM_FLAG_Update) != RESET))
        Fault(MOTOR_CURRENT_OVERRUN);
}
