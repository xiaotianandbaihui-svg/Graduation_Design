#include "balance_pid.h"
#include <math.h>
static float Limit(float x, float limit)
{
    return x > limit ? limit : (x < -limit ? -limit : x);
}

void PID_Reset(PID *p)
{
    p->integral = p->previous_error = p->output = 0.0f;
    p->have_previous = 0u;
}

void PID_Init(PID *p, float kp, float ki, float kd, float limit)
{
    p->kp = kp; p->ki = ki; p->kd = kd; p->limit = limit;
    PID_Reset(p);
}

/* 所有环统一用误差差分计算D项；freeze仅暂停积分。 */
static float PID_Step(PID *p, float error, int freeze)
{
    float difference = (p->kd != 0.0f && p->have_previous) ?
                       error - p->previous_error : 0.0f;
    float pd = p->kp * error + p->kd * difference;
    float di = p->ki * error, next, u;
    if (p->ki == 0.0f) p->integral = 0.0f;
    /* 只检查一次未限幅结果，避免NaN/溢出被限幅掩盖。 */
    if (!isfinite(pd + p->integral + di)) {
        PID_Reset(p);
        return NAN;
    }
    next = Limit(p->integral + di, p->limit);/*积分限幅*/
    u = pd + next;
    /* 不向饱和方向继续积分，允许反向释放。 */
    if (!freeze && !((u > p->limit && di > 0.0f) ||
                     (u < -p->limit && di < 0.0f))) p->integral = next;
    p->output = Limit(pd + p->integral, p->limit);/*输出限幅*/
    p->previous_error = error;
    p->have_previous = 1u;/*喂给微分项的信号，告诉微分项，现在不是第一次PID，有了之前的误差记录了*/
    return p->output;
}

float PID_Update(PID *p, float error)
{
    return PID_Step(p, error, 0);/*freeze = 0则表示开启积分项不再积累(但是此时已有的积分值还是保留)，这说明PID_Update函数默认是打开积分积累的*/
}

void Balance_Stop(BalancePID *b)
{
    b->running = 0u;
    PID_Reset(&b->angle);
    PID_Reset(&b->speed[0]);
    PID_Reset(&b->speed[1]);
}

#define Speed_limit  1000.0f /*限制不超过1000rpm，理论最大值是6000rpm*/
#define Output_limit 1000.0f
void Balance_Init(BalancePID *b)
{
    PID_Init(&b->angle, 0.0f, 0.0f, 0.0f, Speed_limit);
    PID_Init(&b->speed[0], 0.0f, 0.0f, 0.0f, Output_limit);
    PID_Init(&b->speed[1], 0.0f, 0.0f, 0.0f, Output_limit);
    b->upright_deg = 0.0f; /* 目标直立角 */
    b->fall_deg = 60.0f;   /* 保留你设置的倾倒阈值 */
    b->running = 0u;
}

/* 配置只在Start时检查，不占每周期的计算路径。 */
static int PID_ConfigOK(const PID *p)
{
    return isfinite(p->kp) && isfinite(p->ki) && isfinite(p->kd) &&
           isfinite(p->limit) && p->limit > 0.0f;
}

int Balance_Start(BalancePID *b)
{
    Balance_Stop(b);
    if (!PID_ConfigOK(&b->angle) || !PID_ConfigOK(&b->speed[0]) ||
        !PID_ConfigOK(&b->speed[1]) || b->speed[0].limit > 1000.0f ||
        b->speed[1].limit > 1000.0f || !isfinite(b->upright_deg) ||
        !isfinite(b->fall_deg) || b->fall_deg <= 0.0f) return 0;
    b->running = 1u;
    return 1;
}

int Balance_Update(BalancePID *b, float tilt_deg,
                   float left_rpm, float right_rpm, int16_t command[2])
{
    float error =b->upright_deg-tilt_deg;
    float target, di = b->angle.ki * error, u;
    float measured[2] = {left_rpm, right_rpm};
    int i, freeze;
    if (!b->running || fabsf(error) > b->fall_deg) goto fail;

    /* 内环上次饱和时，暂停外环同方向积分。 */
    freeze = ((b->speed[0].output >= b->speed[0].limit ||
               b->speed[1].output >= b->speed[1].limit) && di > 0.0f) ||
             ((b->speed[0].output <= -b->speed[0].limit ||
               b->speed[1].output <= -b->speed[1].limit) && di < 0.0f);
    target = PID_Step(&b->angle, error, freeze);
    if (!isfinite(target)) goto fail;
    for (i = 0; i < 2; ++i) {
        u = PID_Update(&b->speed[i], target - measured[i]);
        if (!isfinite(u)) goto fail; /* 异常浮点不能转成整数命令 */
        command[i] = (int16_t)(u >= 0.0f ? u + 0.5f : u - 0.5f);/*这里的±0.5f就是专门用于四舍五入的*/
    }
    return 1;
fail:
    Balance_Stop(b);
    command[0] = command[1] = 0;
    return 0;
}





/*------------------------------------------------------------------------------*/
