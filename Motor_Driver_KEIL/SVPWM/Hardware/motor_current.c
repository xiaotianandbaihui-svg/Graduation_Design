#include "motor_current.h"
#include "current_loop.h"
#include "current_sense.h"
#include "svpwm.h"
#include "tle5012b.h"
#include "stm32f10x.h"
#include "cycle_counter.h"
#include <math.h>

static CurrentLoop loops[2];/*全局的电流环*/
static CurrentLoop_Feedback feedback[2];/*1.记录每一时刻的相电流，电角度，母线电压*/
static volatile MotorCurrent_Status state;/*记录电机的当前状态*/
static volatile float target_iq[2], rpm[2];
static float previous_mechanical[2];
static uint32_t previous_tick[2], cpu_hz, period_cycles, last_entry;/*这几个都是面向DWT计数的*/
static volatile uint32_t last_cycles, command_age;

/*范围限制到[-180°,180°)*/
static float AngleDifference(float now, float before)
{
    float delta = now - before;
    if (delta >= 180.0f) delta -= 360.0f;
    if (delta < -180.0f) delta += 360.0f;
    return delta;
}

void MotorCurrent_Stop(void)
{
    /*本质是在这是PRIMASK寄存器
    
    | PRIMASK | 含义 |
    |---|---|
    | 0 | 不全局屏蔽可配置优先级的中断 |
    | 1 | 屏蔽全部除了NMI、HardFault之外的中断 |
    */
    uint32_t mask = __get_PRIMASK();
    __disable_irq();


    TIM_Cmd(TIM1, DISABLE);
    NVIC_DisableIRQ(TIM1_UP_IRQn);
    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    NVIC_ClearPendingIRQ(TIM1_UP_IRQn);
    SVPWM_Stop(0u); SVPWM_Stop(1u);/*直接设置了DRV不接受PWM信号，进入高阻态*/


    CurrentLoop_Reset(&loops[0]); CurrentLoop_Reset(&loops[1]);/*清除电流环的全部存储的参数*/
    target_iq[0] = target_iq[1] = 0.0f;/*两轮的目标iq全部清空*/
    command_age = 0u;/*用来记录：电流环已经连续执行了多少次，却没有收到主循环提交的新电流目标。 它是普通的软件计数变量，不是硬件寄存器。
    目的是避免主循环停止更新后，电流环一直执行旧指令。*/
    if (state >= MOTOR_CURRENT_STOPPED) state = MOTOR_CURRENT_STOPPED;/*把电机的状态标识设置为停止状态*/
    __set_PRIMASK(mask);/*重新开启中断*/
}

static int Fault(MotorCurrent_Status error)
{
    state = error;
    MotorCurrent_Stop();
    return 0;
}

MotorCurrent_Status MotorCurrent_GetStatus(void) { return state; }/*获取电机的执行状态*/

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
    MotorCurrent_Stop();/*这里会让DRV芯片全部进入高阻态，不再接受PWM信号，同时清除电流环的全部存储的参数*/
    state = MOTOR_CURRENT_STOPPED;/*更新电机运行状态标识*/

    /*用于初始化的参数的校验，校验失败直接停机，一般也不会失败*/
    if (MOTOR_CURRENT_HZ == 0u || 1000000u % MOTOR_CURRENT_HZ != 0u ||
        MOTOR_CURRENT_HZ > SVPWM_PWM_HZ ||
        !(MOTOR_CURRENT_LIMIT_A > 0.0f && MOTOR_CURRENT_LIMIT_A < MOTOR_CURRENT_TRIP_A) ||
        !(MOTOR_CURRENT_BUS_VOLTAGE_V > 0.0f) ||
        MOTOR_CURRENT_COMMAND_TIMEOUT_MS == 0u)
        return Fault(MOTOR_CURRENT_CONFIG_ERROR);

    
    SystemCoreClockUpdate();
    RCC_GetClocksFreq(&clocks);
    cpu_hz = clocks.HCLK_Frequency;
    period_cycles = cpu_hz / MOTOR_CURRENT_HZ;/*看对应CPU的时钟频率要多少计数，这个是对后面进行时钟双重校验和电流环超时检测的总要依据
    这个算的其实就是一个电流环执行周期，DWT要计多少个数*/
    tim_hz = clocks.PCLK2_Frequency;
    if (clocks.PCLK2_Frequency != clocks.HCLK_Frequency) tim_hz *= 2u;/*高级定时器的时钟频率的确是有可能乘二的，我记得F407的时候被坑过*/
    if (tim_hz % 1000000u != 0u) return Fault(MOTOR_CURRENT_CONFIG_ERROR);

    CycleCounter_Enable();/*开启内核外设DWT，这算是一个计时器*/

    for (id = 0u; id < 2u; ++id) {
        if (!CurrentLoop_Init(&loops[id], &config)) return Fault(MOTOR_CURRENT_CONFIG_ERROR);/*根据前面的config结构体的参数进行PI配置*/
        feedback[id].bus_voltage_v = MOTOR_CURRENT_BUS_VOLTAGE_V;/*设置母线电压*/
        rpm[id] = 0.0f;/*轮速清空*/
    }

    /*准备定时器相关事情*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    TIM_DeInit(TIM1);
    TIM_TimeBaseStructInit(&timer);
    timer.TIM_Prescaler = (uint16_t)(tim_hz / 1000000u - 1u);
    timer.TIM_Period = 1000000u / MOTOR_CURRENT_HZ - 1u;/*500us中断一次，但是当前阶段的中断还没使能*/
    TIM_TimeBaseInit(TIM1, &timer);
    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
    irq.NVIC_IRQChannel = TIM1_UP_IRQn;
    irq.NVIC_IRQChannelPreemptionPriority = 0u;
    irq.NVIC_IRQChannelSubPriority = 0u;
    irq.NVIC_IRQChannelCmd = DISABLE;/*这里中断暂时没使能*/
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


        CurrentLoop_Reset(&loops[id]);/*清除电流环已经存在的全部数据*/
        /* 开中断之前取得初始角度/轮速；以后只有TIM1中断访问SPI。 */
        if (!SVPWM_ReadRotor(id, &previous_mechanical[id], &feedback[id].electrical_deg))/*读取转子当前的的电角度和机械角度*/
            return Fault(MOTOR_CURRENT_ENCODER_ERROR);

        previous_tick[id] = CycleCounter_Now();/*记录当前时刻，这个是从DWT读出来的数据*/
        if (TLE5012B_ReadAngularSpeed(id, &speed_dps) != TLE_OK)/*这个函数读取角速度，单位：°/s*/
            return Fault(MOTOR_CURRENT_ENCODER_ERROR);

        rpm[id] = speed_dps * SVPWM_GetEncoderSign(id) / 6.0f;/*规定到电角度正方向为正方向*/
        target_iq[id] = 0.0f;/*目标电流暂时为0*/
    }
    if (!SVPWM_StartCurrent(MOTOR_CURRENT_SAMPLE_GUARD_US, &available_ratio))/*这里就是关键了，会根据ADC配置，确定比例上限*/
        return Fault(MOTOR_CURRENT_CONTROL_ERROR);

    for (id = 0u; id < 2u; ++id)
        loops[id].voltage_ratio = fminf(MOTOR_CURRENT_VOLTAGE_RATIO, available_ratio);/*把算出来的上限给到电流环*/

    command_age = 0u;
    last_cycles = 0u;
    last_entry = CycleCounter_Now();/*这个变量就是来专门记录两次进入电流环调控中断的时间间隔的*/

    /*TIM1准备开始计数*/
    TIM_SetCounter(TIM1, 0u);
    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    NVIC_ClearPendingIRQ(TIM1_UP_IRQn);
    state = MOTOR_CURRENT_RUNNING;/*设置全局电机运行状态标识*/
    NVIC_EnableIRQ(TIM1_UP_IRQn);/*开启NVIC端的中断使能*/
    TIM_Cmd(TIM1, ENABLE);/*开始计数*/
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
    command_age = 0u;/*一次电流环目标值写入，下面的中断是会检查这个变量的，这边目前是超过
    MOTOR_CURRENT_COMMAND_TIMEOUT_MS毫秒没外部目标值输入信号进来，就认为程序出BUG了*/
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

    /*这里的意思是：超过MOTOR_CURRENT_COMMAND_TIMEOUT_MS没有收到新的目标电流传入，就认为死机了*/
    if (++command_age > (MOTOR_CURRENT_COMMAND_TIMEOUT_MS * MOTOR_CURRENT_HZ + 999u) / 1000u)
        return Fault(MOTOR_CURRENT_COMMAND_TIMEOUT);


    for (id = 0u; id < 2u; ++id) {
        if (!SVPWM_ReadRotor(id, &mechanical, &feedback[id].electrical_deg))
            return Fault(MOTOR_CURRENT_ENCODER_ERROR);
        tick[id] = CycleCounter_Now();
        dt = tick[id] - previous_tick[id]; /* 无符号差值允许CYCCNT自然回绕 */
        /*tick[id]：本次读取角度完成后的 DWT 时间戳。
            previous_tick[id]：上次的时间戳。
            dt：两次之间经过的 CPU 周期数，单位不是秒。
            也就说，记录的是两次调用Step()这个函数的时间间隔*/

        if (dt == 0u) return Fault(MOTOR_CURRENT_OVERRUN);


        delta = AngleDifference(mechanical, previous_mechanical[id]);
        electrical_dps[id] = delta * ((float)cpu_hz / dt) * SVPWM_GetEncoderSign(id);/*这里通过读取到的角度和DWT粗略计算机械角速度，单位还是°/s*/
        speed = electrical_dps[id] / 6.0f;/*这里转化为rpm*/
        rpm[id] += 0.2f * (speed - rpm[id]); /* 仅轮速反馈滤波，不滤波电流  等价于rpm[id] = 0.8f * rpm[id] + 0.2f * speed; */
        electrical_dps[id] *= SVPWM_POLE_PAIRS;/*电角速度*/
        previous_mechanical[id] = mechanical;/*更新全局的机械角度为当前机械角度*/
        previous_tick[id] = tick[id];/*更新全局的DWT时刻为当前时刻*/
    }


    if (!CurrentSense_ReadSynchronized(&raw)) return Fault(MOTOR_CURRENT_SAMPLE_ERROR);/*此处是关键！读取数值*/

    sampled = CycleCounter_Now();
    for (id = 0u; id < 2u; ++id) {
        if (!CurrentSense_Convert(id, &raw, &phase)) return Fault(MOTOR_CURRENT_SAMPLE_ERROR);/*把测到的电压转化为电流*/
        if (fabsf(phase.ia_a) >= MOTOR_CURRENT_TRIP_A ||
            fabsf(phase.ib_a) >= MOTOR_CURRENT_TRIP_A ||
            fabsf(phase.ic_a) >= MOTOR_CURRENT_TRIP_A) return Fault(MOTOR_CURRENT_OVERCURRENT);/*此处检查是否堵转*/

        feedback[id].ia_a = phase.ia_a;
        feedback[id].ib_a = phase.ib_a;
        /* SPI角度与ADC并非硬件同时采样，按转速外推到ADC完成附近。
         * 仍包含编码器自身滤波延迟及几us的ADC误差，高速时需实测相位。 */
        feedback[id].electrical_deg += electrical_dps[id] *
                                      ((float)(uint32_t)(sampled - tick[id]) / cpu_hz);/*补偿当前电角度*/
    }


    /* 两轮都算完、且没有超期，再把电压交给原有PWM接口。 */
    for (id = 0u; id < 2u; ++id)
        if (!CurrentLoop_Update(&loops[id], &feedback[id], target_iq[id], &voltage[id]))
            return Fault(MOTOR_CURRENT_CONTROL_ERROR);
    if ((uint32_t)(CycleCounter_Now() - last_entry) >= period_cycles)/*检查函数执行到现在，已经超过两个电流环周期了嘛(下一个中断来了吗？)*/
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
        Fault(MOTOR_CURRENT_OVERRUN); /* 严重迟到，不能把旧采样重复积分，一个period_cycles是一个电机调控周期，要是电流环周期进行的中断相邻两次发生超过2个电机调控周期，
        就认为中间存在了死机情况，电机或者板子或者代码可能出问题了，立马停机 */
        return;
    }
    last_entry = began;/*更新时间戳*/

    Step();
    last_cycles = CycleCounter_Now() - began;/*这个变量用于精确记录最新一次电流环运行花了多久*/

    
    if (state == MOTOR_CURRENT_RUNNING &&
        (last_cycles >= period_cycles || TIM_GetFlagStatus(TIM1, TIM_FLAG_Update) != RESET))
        Fault(MOTOR_CURRENT_OVERRUN);/*你的执行电流环的时间不能比两次调控间隔时间还长，不然肯定是出BUG了*/
}
