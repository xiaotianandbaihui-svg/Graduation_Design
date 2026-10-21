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
    return PID_Step(p, error, 0); /* freeze=0：允许积分 */
}

void FootPID_Init(FootPID *p)
{
    PID_Init(&p->roll, FOOT_ROLL_KP, 0.0f, 0.0f, FOOT_Y_STEP_MAX_MM);
    PID_Init(&p->x[0], FOOT_X_KP_LEFT, 0.0f, 0.0f, FOOT_X_STEP_MAX_MM);
    PID_Init(&p->x[1], FOOT_X_KP_RIGHT, 0.0f, 0.0f, FOOT_X_STEP_MAX_MM);
    p->target_roll_deg = FOOT_TARGET_ROLL_DEG;
    p->target_rpm[0] = FOOT_TARGET_RPM_LEFT;
    p->target_rpm[1] = FOOT_TARGET_RPM_RIGHT;//严格的，我们是要保障正常运动的时候，两轮的x轴坐标始终一致的/
}

static float ClampRange(float x, float low, float high)
{
    return x < low ? low : (x > high ? high : x);
}

static int FootConfigOK(const PID *p)
{
    return isfinite(p->kp) && isfinite(p->ki) && isfinite(p->kd) &&
           isfinite(p->limit) && p->limit > 0.0f;
}

int FootPID_Update(FootPID *p, float roll_deg, const float wheel_rpm[2],
                   const float current_x[2], const float current_y[2],
                   float next_x[2], float next_y[2])
{
    const int sign[2] = {FOOT_WHEEL_SIGN_LEFT, FOOT_WHEEL_SIGN_RIGHT};
    FootPID state;
    float x[2], y[2], error, dy, low, high, dx;
    unsigned i;
    if (!p || !wheel_rpm || !current_x || !current_y || !next_x || !next_y ||
        !isfinite(roll_deg) || !isfinite(p->target_roll_deg) ||
        !FootConfigOK(&p->roll) || !FootConfigOK(&p->x[0]) ||
        !FootConfigOK(&p->x[1]) || !(FOOT_X_MIN_MM < FOOT_X_MAX_MM) ||
        !(FOOT_Y_MIN_MM < FOOT_Y_MAX_MM) ||
        !(FOOT_Y_STEP_MAX_MM > 0.0f) || !(FOOT_X_STEP_MAX_MM > 0.0f)) return 0;
    for (i = 0; i < 2; ++i) {
        if ((sign[i] != 1 && sign[i] != -1) || !isfinite(wheel_rpm[i]) ||
            !isfinite(p->target_rpm[i]) ||
            !(current_x[i] >= FOOT_X_MIN_MM && current_x[i] <= FOOT_X_MAX_MM) ||
            !(current_y[i] >= FOOT_Y_MIN_MM && current_y[i] <= FOOT_Y_MAX_MM)) return 0;
    }
    state = *p;
    error = p->target_roll_deg - roll_deg;
    if (fabsf(error) <= FOOT_ROLL_DEADBAND_DEG) error = 0.0f;
    dy = PID_Update(&state.roll, error);
    if (!isfinite(dy)) return 0;
    dy = Limit(dy, FOOT_Y_STEP_MAX_MM);/*腿长步进限幅*/
    /* 两侧共同限幅，避免单侧到限位后平均腿高继续漂移。*/
    low = fmaxf(FOOT_Y_MIN_MM - current_y[0], current_y[1] - FOOT_Y_MAX_MM);
    high = fminf(FOOT_Y_MAX_MM - current_y[0], current_y[1] - FOOT_Y_MIN_MM);
    dy = ClampRange(dy, low, high);
    y[0] = current_y[0] + dy;
    y[1] = current_y[1] - dy;
    for (i = 0; i < 2; ++i) {
        error = p->target_rpm[i] - (float)sign[i] * wheel_rpm[i];
        if (fabsf(error) <= FOOT_SPEED_DEADBAND_RPM) error = 0.0f;
        dx = PID_Update(&state.x[i], error);
        if (!isfinite(dx)) return 0;
        dx = Limit(dx, FOOT_X_STEP_MAX_MM);
        x[i] = ClampRange(current_x[i] + dx, FOOT_X_MIN_MM, FOOT_X_MAX_MM);/*限制x的幅度*/
    }
    for (i = 0; i < 2; ++i) { next_x[i] = x[i]; next_y[i] = y[i]; }/*设置输出赋值*/
    *p = state;
    return 1;
}

