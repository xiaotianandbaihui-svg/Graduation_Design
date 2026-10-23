#include "jump.h"
#include "leg_ik.h"
#include "servo.h"

extern LegIK_Geometry My_LegIK_Geometry;
extern float current_leg_x[2], current_leg_height[2];

RobotState Robot_State = ROBOT_NORMAL;
volatile uint8_t Jump_Request;

/* 冻结起跳前的x和左右高度差，跳跃只改变两腿共同的平均高度。 */
static float foot_x[2], half_difference, low_mean, high_mean;
static uint32_t started, phase_time, sampled, count;

static int8_t Fault(void)/*这里设置了跳跃状态和Robot_State状态*/
{
    Robot_State = ROBOT_FAULT;
    return JUMP_ERROR;
}

/* 仅启动前检查三组目标；每条腿使用自己的x和舵机偏置。 */
static int TargetOK(float mean)
{
    const float bias[4] = {Servo_Bias_1, Servo_Bias_2, Servo_Bias_3, Servo_Bias_4};
    LegIK_Angles q;
    unsigned i;
    for (i = 0; i < 2; ++i) {
        float y = mean + (i == 0 ? half_difference : -half_difference);
        float a, b;
        if (LegIK_Solve(&My_LegIK_Geometry, foot_x[i], y, &q) != LEG_IK_OK) return 0;
        a = bias[2*i] - q.q1_deg;
        b = bias[2*i+1] - q.q4_deg;
        if (!(a >= 0 && a <= SERVO_MAX_ANGLE_DEG &&
              b >= 0 && b <= SERVO_MAX_ANGLE_DEG)) return 0;
    }
    return 1;
}

static int8_t MoveTo(float mean, uint8_t immediate, RobotState next, uint32_t now)
{
    float y[2] = {mean + half_difference, mean - half_difference};
    if (immediate) Leg_ResetFootPositionHistory();
    if (Leg_SetFootPositions(foot_x, y) != LEG_IK_OK) return Fault();
    current_leg_height[0] = y[0]; current_leg_height[1] = y[1];
    Robot_State = next;
    phase_time = now;
    count = 0;
    return JUMP_RUNNING;
}

int8_t Jump_Start(uint32_t now_ms)
{
    float difference;
    unsigned i;
    if (Robot_State == ROBOT_FAULT) return JUMP_ERROR;
    if (Robot_State != ROBOT_NORMAL) return JUMP_RUNNING;
    if (!(JUMP_LOW_MM > 0 && JUMP_LOW_MM < JUMP_MID_MM && JUMP_MID_MM < JUMP_HIGH_MM) ||
        !isfinite(JUMP_BUFFER_STEP_MM) || JUMP_BUFFER_STEP_MM <= 0 ||
        JUMP_SAMPLE_MS == 0 || JUMP_AIR_SAMPLES == 0 || JUMP_LAND_SAMPLES == 0)
        return JUMP_ERROR;
    for (i = 0; i < 2; ++i) {
        if (!(current_leg_x[i] >= FOOT_X_MIN_MM && current_leg_x[i] <= FOOT_X_MAX_MM &&
              current_leg_height[i] >= JUMP_LOW_MM && current_leg_height[i] <= JUMP_HIGH_MM))
            return JUMP_ERROR;
        foot_x[i] = current_leg_x[i];
    }

    half_difference = (current_leg_height[0] - current_leg_height[1]) * 0.5f;
    difference = fabsf(half_difference);
    low_mean = JUMP_LOW_MM + difference;   /* 短腿先到最低，另一条腿保留高度差 */
    high_mean = JUMP_HIGH_MM - difference; /* 长腿先到最高 */
    
    if (!(low_mean < JUMP_MID_MM && JUMP_MID_MM < high_mean &&
          JUMP_MID_MM - difference >= FOOT_Y_MIN_MM &&
          JUMP_MID_MM + difference <= FOOT_Y_MAX_MM) ||
        !TargetOK(low_mean) || !TargetOK(JUMP_MID_MM) || !TargetOK(high_mean))
        return JUMP_ERROR;
    started = sampled = now_ms; /* 无符号差值计时，允许毫秒时钟自然回绕 */
    return MoveTo(low_mean, 1, ROBOT_JUMP_SQUAT, now_ms);/*这里更改了Robot_State*/
}

int8_t Jump_Update(uint32_t now_ms, float accel_g)
{
    uint32_t gap = (uint32_t)(now_ms - sampled);
    uint32_t elapsed = (uint32_t)(now_ms - phase_time);/*不同阶段相隔的时间*/

    if (Robot_State == ROBOT_NORMAL) return JUMP_IDLE;
    if (Robot_State == ROBOT_FAULT) return JUMP_ERROR;

    if (!isfinite(accel_g) || accel_g < 0) return Fault();
    if (Robot_State <= ROBOT_JUMP_LAND &&
        (uint32_t)(now_ms - started) >= JUMP_TIMEOUT_MS) return Fault();/*跳跃执行全过程超时*/


    /* count只表示连续样本数；缓冲同样每个采样周期最多走一步。 */
    if (gap < JUMP_SAMPLE_MS) return JUMP_RUNNING;
    if (gap > 2u * JUMP_SAMPLE_MS) count = 0;
    sampled = now_ms;

    switch (Robot_State) {
    case ROBOT_JUMP_SQUAT:
        if (elapsed >= JUMP_SQUAT_MS)/*这里等待舵机响应达到设定的最低位置*/
            return MoveTo(high_mean, 1, ROBOT_JUMP_PUSH, now_ms);
        break;
    case ROBOT_JUMP_PUSH:
        if (elapsed < JUMP_PUSH_MIN_MS) break;/*这里主要是保障舵机能完全蹬地发力，是等待舵机响应到位的*/
        count = accel_g < JUMP_AIR_G ? count + 1u : 0;
        if (count >= JUMP_AIR_SAMPLES)
            return MoveTo(low_mean, 1, ROBOT_JUMP_RETRACT, now_ms);/*已经满足腾空了，立马收腿*/
        break;
    case ROBOT_JUMP_RETRACT:
    case ROBOT_JUMP_LAND:
        if (elapsed >= JUMP_LAND_BLANK_MS) {
            count = accel_g > JUMP_LAND_G ? count + 1u : 0;
            if (count >= JUMP_LAND_SAMPLES) {
                Robot_State = Robot_State == ROBOT_JUMP_RETRACT ?
                              ROBOT_JUMP_LOW_HOLD : ROBOT_JUMP_BUFFER;
                phase_time = now_ms;
                count = 0;
                break;
            }
        }
        /* 短腿提前触地时不再伸腿；已有疑似触地样本时先完成确认。 */
        if (Robot_State == ROBOT_JUMP_RETRACT && count == 0 && elapsed >= JUMP_PREP_DELAY_MS)/*MoveTo函数会清零count*/
            return MoveTo(JUMP_MID_MM, 1, ROBOT_JUMP_LAND, now_ms);
        break;
    case ROBOT_JUMP_BUFFER: {
        float mean = (current_leg_height[0] + current_leg_height[1]) * 0.5f;
        mean = fmaxf(low_mean, mean - JUMP_BUFFER_STEP_MM);
        return MoveTo(mean, 0, mean <= low_mean ? ROBOT_JUMP_LOW_HOLD : ROBOT_JUMP_BUFFER, now_ms);
    }
    case ROBOT_JUMP_LOW_HOLD:
        if (elapsed >= JUMP_LOW_HOLD_MS)
            return MoveTo(JUMP_MID_MM, 1, ROBOT_JUMP_RECOVER, now_ms);
        break;
    case ROBOT_JUMP_RECOVER:
        if (elapsed >= JUMP_RECOVER_MS) {
            Robot_State = ROBOT_NORMAL;
            return JUMP_LANDED; /* 主循环确认电驱恢复后，才重新运行PID；姿态一直连续更新 */
        }
        break;
    default:
        return Fault();
    }
    return JUMP_RUNNING;
}
