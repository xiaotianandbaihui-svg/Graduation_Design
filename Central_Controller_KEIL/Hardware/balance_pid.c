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
    PID_Init(&p->x[0], FOOT_X_KP_LEFT, 0.0f, 0.0f, FOOT_X_MAX_MM);
    PID_Init(&p->x[1], FOOT_X_KP_RIGHT, 0.0f, 0.0f, FOOT_X_MAX_MM);
    p->target_roll_deg = FOOT_TARGET_ROLL_DEG;
    p->target_mean_height_mm = FOOT_INITIAL_Y_MM;
    p->target_rpm[0] = FOOT_TARGET_RPM_LEFT;
    p->target_rpm[1] = FOOT_TARGET_RPM_RIGHT;//严格的，我们是要保障正常运动的时候，两轮的x轴坐标始终一致的/
}

static float ClampRange(float x, float low, float high)
{
    return x < low ? low : (x > high ? high : x);
}

int FootPID_SetMeanHeight(FootPID *p, float height_mm)
{
    if (!isfinite(height_mm)) return 0;
    p->target_mean_height_mm = ClampRange(height_mm,
        FOOT_MEAN_HEIGHT_MIN_MM, FOOT_MEAN_HEIGHT_MAX_MM);
    return 1;
}

static int FootConfigOK(const PID *p)
{
    return isfinite(p->kp) && isfinite(p->ki) && isfinite(p->kd) &&
           isfinite(p->limit) && p->limit > 0.0f;
}
#define FOOT_X_TRIM_MM       0.0f/*x零点位置，当前就是0.0f，我觉得后面可能还是要挑一挑*/
int FootPID_Update(FootPID *p, float roll_deg, const float wheel_rpm[2],
                    float current_x[2],  float current_y[2],
                   float next_x[2], float next_y[2])
{
    const int sign[2] = {FOOT_WHEEL_SIGN_LEFT, FOOT_WHEEL_SIGN_RIGHT};
    FootPID state;
    float x[2], y[2], error, dy, low, high;
    float mean_y, target_mean, common_step, available_step;
    float offset, target_x, step;   /* 声明放到函数开头 */
    if (!p || !wheel_rpm || !current_x || !current_y || !next_x || !next_y ||
        !isfinite(roll_deg) || !isfinite(p->target_roll_deg) ||
        !isfinite(p->target_mean_height_mm) ||
        !FootConfigOK(&p->roll) || !FootConfigOK(&p->x[0]) ||
        !FootConfigOK(&p->x[1]) || !(FOOT_X_MIN_MM < FOOT_X_MAX_MM) ||
        !(FOOT_Y_MIN_MM < FOOT_Y_MAX_MM) ||
        !(FOOT_MEAN_HEIGHT_MIN_MM < FOOT_MEAN_HEIGHT_MAX_MM) ||
        !(FOOT_ROLL_HEIGHT_RESERVE_MM >= 0.0f) || !(FOOT_MEAN_HEIGHT_STEP_MAX_MM > 0.0f) ||
        !(FOOT_Y_STEP_MAX_MM > 0.0f) || !(FOOT_X_STEP_MAX_MM > 0.0f)) return 0;
    for (int i = 0; i < 2; ++i) {
        if ((sign[i] != 1 && sign[i] != -1) || !isfinite(wheel_rpm[i]) ||
            !isfinite(p->target_rpm[i]) ||
            !(current_x[i] >= FOOT_X_MIN_MM && current_x[i] <= FOOT_X_MAX_MM) ||
            !(current_y[i] >= FOOT_Y_MIN_MM && current_y[i] <= FOOT_Y_MAX_MM)) return 0;
    }
    state = *p;
    /* 先算Roll差动：保持原来的左右等量反向增量，不因升降而清零。 */
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

    /* 再算共同升降：两腿加同一个量，因此不会抹掉左右高度差。 */
    mean_y = 0.5f * (current_y[0] + current_y[1]);
    target_mean = ClampRange(p->target_mean_height_mm,
                            FOOT_MEAN_HEIGHT_MIN_MM, FOOT_MEAN_HEIGHT_MAX_MM);
    available_step = fminf(FOOT_MEAN_HEIGHT_STEP_MAX_MM,
                          FOOT_Y_STEP_MAX_MM - fabsf(dy));
    common_step = Limit(target_mean - mean_y, available_step);
    /* 用Roll调整后的两腿位置计算剩余行程，保留纠偏优先级。 */
    low = fmaxf(FOOT_Y_MIN_MM - y[0], FOOT_Y_MIN_MM - y[1]);
    high = fminf(FOOT_Y_MAX_MM - y[0], FOOT_Y_MAX_MM - y[1]);
    common_step = ClampRange(common_step, low, high);
    y[0] += common_step;
    y[1] += common_step;


    error = p->target_rpm[0] - ((float)sign[0] * wheel_rpm[0] + (float)sign[1] * wheel_rpm[1]) / 2.0f;
    if (fabsf(error) <= FOOT_SPEED_DEADBAND_RPM) error = 0.0f;
    offset = PID_Update(&state.x[0], error);   /* 相对配平点的偏移 */
    if (!isfinite(offset)) return 0;
    target_x = ClampRange(FOOT_X_TRIM_MM + offset, FOOT_X_MIN_MM, FOOT_X_MAX_MM);
    for (int i = 0; i < 2; ++i) {
        step = Limit(target_x - current_x[i], FOOT_X_STEP_MAX_MM);
        x[i] = ClampRange(current_x[i] + step, FOOT_X_MIN_MM, FOOT_X_MAX_MM);
        next_x[i] = x[i];
        next_y[i] = y[i];/*一次循环，顺手回传y轴坐标，以防止冗余执行*/
    }
    *p = state;
    return 1;
}


void SpeedPID_Init(PID *p)
{
    PID_Init(p, SPEED_KP, SPEED_KI, 0.0f, SPEED_PITCH_MAX_DEG);
}

int SpeedPID_Update(PID *p, float target_rpm,
                    const float wheel_rpm[2], float *target_pitch)
{
    /* 统一为车前进方向为正，再取平均轮速。 */
    float actual_rpm =
        (FOOT_WHEEL_SIGN_LEFT  * wheel_rpm[0] +
         FOOT_WHEEL_SIGN_RIGHT * wheel_rpm[1]) * 0.5f;

    float pitch = PID_Update(p, target_rpm - actual_rpm);

    if (!isfinite(pitch)) return 0;

    *target_pitch = pitch;
    return 1;
}
