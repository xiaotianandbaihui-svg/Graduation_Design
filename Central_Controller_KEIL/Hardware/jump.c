#include "jump.h"
#include "leg_ik.h"

/* 保留逐飞原始读取函数；只有SPI读取成功才交付三个原始值。 */
static uint8_t Jump_ReadAccel(int16_t raw[3])
{
    imu963ra_get_acc();
    if (imu963ra_spi_error) return 0;
    raw[0] = imu963ra_acc_x;
    raw[1] = imu963ra_acc_y;
    raw[2] = imu963ra_acc_z;
    return 1;
}

static const LegIK_Geometry leg = JUMP_GEOMETRY;/*轮足的五连杆机械结构*/
static const uint8_t servo_id[4] = JUMP_SERVO_IDS;
static const int8_t direction[4] = JUMP_DIRECTION;
static const float offset[4] = JUMP_OFFSET_DEG;
static const float lower[4] = JUMP_MIN_DEG, upper[4] = JUMP_MAX_DEG;
static float target[3][4]; /* 最低、最高、中间目标 */
static int8_t result;
static uint8_t phase, count; /* 下蹲、等离地、空中收腿、等触地、平滑屈腿 */
static uint32_t started, phase_time, sampled;
static float buffer_progress, buffer_step;

/*本函数用于根据足端位置得到4个舵机的角度，跳跃只能发生在轮足正常奔跑和静止直立时，其他都是违法的*/
uint8_t Get_Angles(float x_mm, float y_mm, float angle[4])
{
    LegIK_Angles q;
    float temp[4];
    unsigned i;
    if (!angle || LegIK_Solve(&leg, x_mm, y_mm, &q) != LEG_IK_OK) return 0;
    for (i = 0; i < 4; ++i) {
        if ((direction[i] != 1 && direction[i] != -1) ||
            !(lower[i] >= 0 && lower[i] < upper[i] && upper[i] <= SERVO_MAX_ANGLE_DEG))
            return 0;
        temp[i] = offset[i] + direction[i] * ((i & 1u) ? q.q4_deg : q.q1_deg);
        if (!(temp[i] >= lower[i] && temp[i] <= upper[i])) return 0;
    }
    for (i = 0; i < 4; ++i) angle[i] = temp[i];
    return 1;
}

static uint8_t SendAngles(const float angle[4])
{
    unsigned i;
    for (i = 0; i < 4; ++i)
        if (!JUMP_WRITE_SERVO(servo_id[i], angle[i])) return 0;
    return 1;
}

int8_t Jump_Start(uint32_t now_ms)
{
    unsigned i, j;
    int16_t raw[3];
    float angle[4], duration_ms;
    if (result == JUMP_RUNNING) return result;
    result = JUMP_ERROR;
    for (i = 0; i < 4; ++i) {
        if (servo_id[i] > 3) return result;
        for (j = 0; j < i; ++j) if (servo_id[i] == servo_id[j]) return result;
    }
    if (!(JUMP_LOW_MM > 0 && JUMP_BUFFER_MM > JUMP_LOW_MM &&
          JUMP_MID_MM > JUMP_BUFFER_MM && JUMP_HIGH_MM > JUMP_MID_MM) ||
        !isfinite(JUMP_BUFFER_MAX_MM_S) || !(JUMP_BUFFER_MAX_MM_S > 0) ||
        JUMP_BUFFER_MS == 0 || JUMP_SAMPLE_MS == 0 ||
        !isfinite(JUMP_ACCEL_LSB_PER_G) || !(JUMP_ACCEL_LSB_PER_G > 0.0f) ||
        !Get_Angles(JUMP_X_MM, JUMP_LOW_MM, target[0]) ||
        !Get_Angles(JUMP_X_MM, JUMP_HIGH_MM, target[1]) ||
        !Get_Angles(JUMP_X_MM, JUMP_MID_MM, target[2]) ||
        !Get_Angles(JUMP_X_MM, JUMP_BUFFER_MM, angle)) return result;
    /* 五次曲线的峰值速度为1.875*行程/用时，过快则延长用时。 */
    duration_ms = fmaxf((float)JUMP_BUFFER_MS,
        1875.0f*(JUMP_MID_MM-JUMP_BUFFER_MM)/JUMP_BUFFER_MAX_MM_S);
    if (!isfinite(duration_ms)) return result;
    buffer_step = (float)JUMP_SAMPLE_MS/duration_ms;
    if (!JUMP_READ_ACCEL(raw) || !SendAngles(target[0])) return result;
    phase = 1; count = 0;
    started = phase_time = sampled = now_ms;
    return result = JUMP_RUNNING;
}

int8_t Jump_Update(uint32_t now_ms)
{
    int16_t raw[3];
    float ax, ay, az, accel_g, s, height, angle[4];
    uint32_t gap;
    if (result != JUMP_RUNNING) return result;
    if (phase == 5) {
        if ((uint32_t)(now_ms-sampled) < JUMP_SAMPLE_MS) return result;
        sampled = now_ms;
        /* 每次只推进一个采样周期，调度迟到时不一次追赶积累位移。 */
        buffer_progress = fminf(1.0f, buffer_progress+buffer_step);
        s = buffer_progress;
        s = s*s*s*(10.0f+s*(-15.0f+6.0f*s)); /* 起止速度、加速度均为0 */
        s = fminf(1.0f, fmaxf(0.0f, s));
        height = JUMP_MID_MM+(JUMP_BUFFER_MM-JUMP_MID_MM)*s;
        if (!Get_Angles(JUMP_X_MM, height, angle) || !SendAngles(angle))
            return result = JUMP_ERROR;
        if (buffer_progress >= 1.0f) result = JUMP_LANDED;
        return result;
    }
    if ((uint32_t)(now_ms-started) >= JUMP_TIMEOUT_MS) return result = JUMP_ERROR;

    if (phase == 1) {
        if ((uint32_t)(now_ms-phase_time) >= JUMP_SQUAT_MS) {
            if (!SendAngles(target[1])) return result = JUMP_ERROR;
            phase = 2; count = 0;
            phase_time = sampled = now_ms;
        }
        return result;
    }
    gap = (uint32_t)(now_ms-sampled);
    if (gap < JUMP_SAMPLE_MS) return result;
    if (gap > 2u*JUMP_SAMPLE_MS) count = 0; /* 不把间隔很久的样本当作连续样本 */
    sampled = now_ms;
    if (!JUMP_READ_ACCEL(raw)) return result = JUMP_ERROR; /* 读失败不能当作0g */
    ax = (float)raw[0]; ay = (float)raw[1]; az = (float)raw[2];
    accel_g = sqrtf(ax*ax + ay*ay + az*az) / JUMP_ACCEL_LSB_PER_G;

    if (phase == 2) {
        if ((uint32_t)(now_ms-phase_time) < JUMP_PUSH_MIN_MS) return result;
        count = accel_g < JUMP_AIR_G ? (uint8_t)(count+1u) : 0;
        if (count >= JUMP_AIR_SAMPLES) {
            if (!SendAngles(target[0])) return result = JUMP_ERROR; /* 确认腾空后收腿 */
            phase = 3; count = 0; phase_time = now_ms;
        }
    } else {
        if ((uint32_t)(now_ms-phase_time) >= JUMP_LAND_BLANK_MS) {
            count = accel_g > JUMP_LAND_G ? (uint8_t)(count+1u) : 0;
            if (count >= JUMP_LAND_SAMPLES) {
                if (phase == 3) return result = JUMP_LANDED; /* 已收短，没有继续屈腿余量 */
                phase = 5; buffer_progress = 0.0f; /* 从当前中间目标开始，不跳变 */
                return result;
            }
        }
        /* 有触地迹象时先等判据确认，避免已经落地还把腿放长。
         * 这是按时间准备落地，原始加速度无法区分弹道上升/下降。 */
        if (phase == 3 && count == 0 && (uint32_t)(now_ms-phase_time) >= JUMP_PREP_DELAY_MS) {
            if (!SendAngles(target[2])) return result = JUMP_ERROR;
            phase = 4; count = 0; phase_time = now_ms;
        }
    }
    return result;
}

void Jump_Stop(void)
{
    result = JUMP_IDLE;
}
