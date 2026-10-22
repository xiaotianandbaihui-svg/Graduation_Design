#include "jump.h"
#include "leg_ik.h"
#include "servo.h"

extern LegIK_Geometry My_LegIK_Geometry;
extern float current_leg_x[2], current_leg_height[2];

uint8_t Robot_State = Stand_Stop;
uint8_t State_Lock = 0;
volatile uint8_t Jump_Request = 0;

static int8_t result = JUMP_IDLE;
static uint8_t phase, count, buffering;
static uint32_t started, phase_time, sampled;

/* 只在启动时预检三个目标，不在每个控制周期重复扫描。 */
uint8_t Get_Angles(float x_mm, float y_mm, float angle[4])
{
    const float bias[4] = {Servo_Bias_1, Servo_Bias_2, Servo_Bias_3, Servo_Bias_4};
    LegIK_Angles q;
    float temp[4];
    unsigned i;
    if (!angle || LegIK_Solve(&My_LegIK_Geometry, x_mm, y_mm, &q) != LEG_IK_OK)
        return 0;
    for (i = 0; i < 4; ++i) {
        temp[i] = bias[i] - ((i & 1u) ? q.q4_deg : q.q1_deg);
        if (!(temp[i] >= 0.0f && temp[i] <= SERVO_MAX_ANGLE_DEG)) return 0;
    }
    for (i = 0; i < 4; ++i) angle[i] = temp[i];
    return 1;
}

static uint8_t SetHeight(float height, uint8_t immediate)
{
    float x[2] = {JUMP_X_MM, JUMP_X_MM};
    float y[2] = {height, height};
    /* 跳跃阶段切换需要直接下发目标；普通步长限制不用于蹬伸/收腿。 */
    if (immediate) Leg_ResetFootPositionHistory();
    if (Leg_SetFootPositions(x, y) != LEG_IK_OK) return 0;
    current_leg_x[0] = current_leg_x[1] = JUMP_X_MM;
    current_leg_height[0] = current_leg_height[1] = height;
    return 1;
}

static int8_t Jump_Fault(void)
{
    Robot_State = Robot_Fault;
    State_Lock = 1;
    return result = JUMP_ERROR;
}

int8_t Jump_Start(uint32_t now_ms)
{
    float angles[4];
    if (result == JUMP_RUNNING) return result;
    if (State_Lock || (Robot_State != Stand_Stop && Robot_State != Running))
        return JUMP_ERROR;
    if (!(JUMP_LOW_MM > 0.0f && JUMP_LOW_MM < JUMP_MID_MM &&
          JUMP_MID_MM < JUMP_HIGH_MM) ||
        !(JUMP_X_MM >= FOOT_X_MIN_MM && JUMP_X_MM <= FOOT_X_MAX_MM) ||
        !(JUMP_MID_MM >= FOOT_Y_MIN_MM && JUMP_MID_MM <= FOOT_Y_MAX_MM) ||
        !isfinite(JUMP_BUFFER_STEP_MM) || JUMP_BUFFER_STEP_MM <= 0.0f ||
        JUMP_SAMPLE_MS == 0u || JUMP_AIR_SAMPLES == 0u || JUMP_LAND_SAMPLES == 0u ||
        !Get_Angles(JUMP_X_MM, JUMP_LOW_MM, angles) ||
        !Get_Angles(JUMP_X_MM, JUMP_HIGH_MM, angles) ||
        !Get_Angles(JUMP_X_MM, JUMP_MID_MM, angles)) return JUMP_ERROR;

    State_Lock = 1;
    Robot_State = Jumping;
    if (!SetHeight(JUMP_LOW_MM, 1)) return Jump_Fault();/*先把腿高降低到最低*/
    phase = 1; count = buffering = 0;
    started = phase_time = sampled = now_ms;
    return result = JUMP_RUNNING;
}

int8_t Jump_Update(uint32_t now_ms, float accel_g)
{
    uint32_t gap;
    float height;
    if (result != JUMP_RUNNING) return result;
    if (!isfinite(accel_g) || accel_g < 0.0f) return Jump_Fault();

    /* 第五阶段：先留出到达最短指令的时间，再恢复中间位置，等待后解锁。 */
    if (phase == 5) {
        if (count == 0) {
            if ((uint32_t)(now_ms-phase_time) < JUMP_LOW_HOLD_MS) return result;
            if (!SetHeight(JUMP_MID_MM, 1)) return Jump_Fault();
            count = 1; phase_time = now_ms;
        } else if ((uint32_t)(now_ms-phase_time) >= JUMP_RECOVER_MS) {
            Robot_State = Stand_Stop;
            State_Lock = 0;
            result = JUMP_LANDED;
        }
        return result;
    }

    /* 第四阶段触地后的缓冲：每次只推进一步，调度迟到时不追赶位移。 */
    if (buffering) {
        if ((uint32_t)(now_ms-sampled) < JUMP_SAMPLE_MS) return result;
        sampled = now_ms;
        height = fmaxf(JUMP_LOW_MM, current_leg_height[0]-JUMP_BUFFER_STEP_MM);
        if (!SetHeight(height, 0)) return Jump_Fault();
        if (height <= JUMP_LOW_MM) {
            phase = 5; count = 0; phase_time = now_ms;
        }
        return result;
    }
    if ((uint32_t)(now_ms-started) >= JUMP_TIMEOUT_MS) return Jump_Fault();/*跳跃过程超时检测，超时锁死状态*/

    /* 第一阶段下蹲等待完成，进入第二阶段蹬伸。 */
    if (phase == 1) {
        if ((uint32_t)(now_ms-phase_time) >= JUMP_SQUAT_MS) {
            if (!SetHeight(JUMP_HIGH_MM, 1)) return Jump_Fault();/*直接以最快速度释放腿*/
            phase = 2; count = 0; phase_time = sampled = now_ms;/*进入二阶段*/
        }
        return result;
    }

    gap = (uint32_t)(now_ms-sampled);
    if (gap < JUMP_SAMPLE_MS) return result;
    if (gap > 2u*JUMP_SAMPLE_MS) count = 0;
    sampled = now_ms;

    if (phase == 2) {
        if ((uint32_t)(now_ms-phase_time) < JUMP_PUSH_MIN_MS) return result;
        count = accel_g < JUMP_AIR_G ? (uint8_t)(count+1u) : 0;
        if (count >= JUMP_AIR_SAMPLES) {
            if (!SetHeight(JUMP_LOW_MM, 1)) return Jump_Fault();
            phase = 3; count = 0; phase_time = now_ms;
        }
        return result;
    }

    /* 第三、四阶段都检测触地；短腿提前触地时直接进入恢复，避免再蹬一下。 */
    if ((uint32_t)(now_ms-phase_time) >= JUMP_LAND_BLANK_MS) {
        count = accel_g > JUMP_LAND_G ? (uint8_t)(count+1u) : 0;
        if (count >= JUMP_LAND_SAMPLES) {
            if (phase == 3) {
                phase = 5; count = 0; phase_time = now_ms;
            } else {
                buffering = 1;
            }
            return result;
        }
    }
    if (phase == 3 && count == 0 &&
        (uint32_t)(now_ms-phase_time) >= JUMP_PREP_DELAY_MS) {
        if (!SetHeight(JUMP_MID_MM, 1)) return Jump_Fault();
        phase = 4; count = 0; phase_time = now_ms;
    }
    return result;
}

void Jump_Stop(void)
{
    if (result != JUMP_RUNNING) result = JUMP_IDLE;
}
