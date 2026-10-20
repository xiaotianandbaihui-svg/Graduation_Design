#include "balance_pid.h"
#include <math.h>
#include "tle5012b.h"
#include "svpwm.h"

extern volatile int8_t car_forward_electircal_forward_1,car_forward_electircal_forward_2;//记录车运行的方向和轮子转动的机械方向是否匹配，如果电角度增长方向对应轮子使得车前进的方向一致，那么就为1，反之为-1 

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
static float PID_Step(PID *p, float error, int freeze,uint8_t if_angle_pid)
{
    float difference = (p->kd != 0.0f && p->have_previous) ?
                       error - p->previous_error : 0.0f;
		if(if_angle_pid)
		{
			if (difference > 180.0f)/*这里为了兼容俯仰角极限，不可以写180度，比如-90°然后目标90度(当然现实不可能，但是为了模型逻辑一致性，我觉得还是要这样)*/
			difference -= 360.0f;
			else if (difference < -180.0f)
			difference += 360.0f;
		}

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
    return PID_Step(p, error, 0,0);/*freeze = 0则表示开启积分项不再积累(但是此时已有的积分值还是保留)，这说明PID_Update函数默认是打开积分积累的*/
}

void Balance_Stop(BalancePID *b)
{
    b->running = 0u;
    PID_Reset(&b->angle);
    PID_Reset(&b->speed[0]);
    PID_Reset(&b->speed[1]);
}

void Balance_Init(uint8_t id,BalancePID *b)/*id==0 俯仰角PID  id==1 偏航角PID*/
{
    if(id==1)
    {
        PID_Init(&b->angle, 0.0f, 0.0f, 0.0f, Speed_limit);
        PID_Init(&b->speed[0], 0.0f, 0.0f, 0.0f, Output_limit);
        PID_Init(&b->speed[1], 0.0f, 0.0f, 0.0f, Output_limit);
        b->upright_deg = 0.0f; /*设置目标角度为0度*/
        b->fall_deg = 360.0f;   /* 设置倾倒阈值，Yaw360度，表示不倾倒 */
        b->running = 0u;
    }
    else if(id==0)
    {
        PID_Init(&b->angle, 0.0f, 0.0f, 0.0f, Output_limit);
        PID_Init(&b->speed[0], 0.0f, 0.0f, 0.0f, Output_limit);//这句代码在Pitch控制里无效，只是为了过后面的PID_ConfigOK函数和Balance_Start函数与保持统一
        PID_Init(&b->speed[1], 0.0f, 0.0f, 0.0f, Output_limit);//这句代码在Pitch控制里无效，只是为了过后面的PID_ConfigOK函数和Balance_Start函数与保持统一
        b->upright_deg = 0.0f; /* 设置目标角度为0度 */
        b->fall_deg = 60.0f;   /* 设置倾倒阈值 */
        b->running = 0u;
    }
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
	float left_rpm, float right_rpm, int16_t command[2],uint8_t Pitch_or_Yaw)
{
    float error =b->upright_deg-tilt_deg;
    if (error > 180.0f)/*这里为了兼容俯仰角极限，不可以写180度，比如-90°然后目标90度(当然现实不可能，但是为了模型逻辑一致性，我觉得还是要这样)*/
        error -= 360.0f;
    else if (error < -180.0f)
        error += 360.0f;
        float target, di = b->angle.ki * error, u;     
   if(Pitch_or_Yaw==1)
   {     

            float measured[2] = {
        car_forward_electircal_forward_1 * left_rpm,
        car_forward_electircal_forward_2 * right_rpm
            };    
            int freeze;
        if (!b->running || fabsf(error) > b->fall_deg) goto fail;

        /* 内环上次饱和时，暂停外环同方向积分。 */
        freeze = ((b->speed[0].output >= b->speed[0].limit ||
                b->speed[1].output >= b->speed[1].limit) && di > 0.0f) ||
                ((b->speed[0].output <= -b->speed[0].limit ||
                b->speed[1].output <= -b->speed[1].limit) && di < 0.0f);
        target = PID_Step(&b->angle, error, freeze,1);
        if (!isfinite(target)) goto fail;

        u = PID_Update(&b->speed[0], target - (measured[0]-measured[1]));
        if (!isfinite(u)) goto fail; /* 异常浮点不能转成整数命令 */
        command[0] = (int16_t)(u >= 0.0f ? u + 0.5f : u - 0.5f);/*这里的±0.5f就是专门用于四舍五入的*/
        command[1]=-command[0];

        return 1;
    }
    else if(Pitch_or_Yaw==0)
    {
        if (!b->running || fabsf(error) > b->fall_deg ) goto fail;
        u=PID_Step(&b->angle,error, 0,1);/*最后的1代表这是角度PID*/
        if (!isfinite(u)) goto fail; /* 异常浮点不能转成整数命令 */
        command[0]=command[1]=(int16_t)(u >= 0.0f ? u + 0.5f : u - 0.5f);/*这里的±0.5f就是专门用于四舍五入的*/
        return 1;
    }

fail:
    Balance_Stop(b);
    command[0] = command[1] = 0;
    return 0;
}





/*------------------------------------------------------------------------------*/
int Balance_ReadFeedback(float *left_rpm, float *right_rpm)
{
    float left_dps, right_dps;

    if (TLE5012B_ReadAngularSpeed(0, &left_dps) != TLE_OK ||
        TLE5012B_ReadAngularSpeed(1, &right_dps) != TLE_OK)
        return 0;

    *left_rpm  = left_dps  / 6.0f * SVPWM_GetEncoderSign(0);
    *right_rpm = right_dps / 6.0f * SVPWM_GetEncoderSign(1);
    return 1;
}
