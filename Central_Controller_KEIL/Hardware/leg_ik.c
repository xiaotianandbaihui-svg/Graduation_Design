#include "leg_ik.h"
#include <math.h>
#include "servo.h"

#define RAD_TO_DEG 57.29577951308232f
#define LENGTH_EPS 0.00001f
/* 从动杆夹角距0/180度不足约0.57度时，单精度角度误差会明显放大。 */
#define PARALLEL_SIN_EPS 0.01f




/*定义车的机械结构参数*/
LegIK_Geometry My_LegIK_Geometry = {
    .l1_mm = 60.0f,
    .l2_mm = 90.0f,
    .l3_mm = 90.0f,
    .l4_mm = 60.0f,
    .l5_mm = 37.5f
};
/*左轮的两个舵机计算出应该的角度*/
LegIK_Angles Left_LegIK_Angles = {
    .q1_deg = 0.0f,
    .q4_deg = 0.0f
};
/*右轮的两个舵机计算出应该的角度*/
LegIK_Angles Right_LegIK_Angles = {
    .q1_deg = 0.0f,
    .q4_deg = 0.0f
};


static float Wrap360(float angle)
{
    angle = fmodf(angle, 360.0f);
    if (angle < 0.0f) angle += 360.0f;
    return angle >= 360.0f ? 0.0f : angle;
}

/* 一侧二连杆逆解。dx、dy是目标点相对该舵机轴的位置。 */
static LegIK_Result SolveSide(float upper, float lower, float dx, float dy,
                              int bend_sign, float *angle_rad)
{
    float distance = hypotf(dx, dy);
    float scale = fmaxf(upper, lower);
    float tolerance = LENGTH_EPS * scale;
    float outer = upper + lower, inner = fabsf(upper - lower);
    float a, b, d, cosine;

    if (!isfinite(distance) || !isfinite(outer)) return LEG_IK_ARGUMENT_ERROR;
    if (distance > outer + tolerance || distance < inner - tolerance)
        return LEG_IK_UNREACHABLE;
    /* 伸直、完全折叠或目标重合于轴心时，逆解不唯一或微分关系奇异。 */
    if (distance >= outer - tolerance || distance <= inner + tolerance)
        return LEG_IK_SINGULAR;

    /* 余弦定理。先按杆长缩放，避免直接对很大的mm数值求平方。 */
    a = upper / scale;
    b = lower / scale;
    d = distance / scale;
    cosine = (a * a + d * d - b * b) / (2.0f * a * d);
    if (!isfinite(cosine)) return LEG_IK_ARGUMENT_ERROR;
    if (cosine > 1.0f) cosine = 1.0f;
    if (cosine < -1.0f) cosine = -1.0f;

    *angle_rad = atan2f(dy, dx) + (float)bend_sign * acosf(cosine);
    return LEG_IK_OK;
}

LegIK_Result LegIK_Solve(const LegIK_Geometry *g,
                         float x_mm, float y_mm, LegIK_Angles *out)
{
    float q1, q4, bx, by, dx, dy, u1x, u1y, u4x, u4y, cross, margin;
    LegIK_Result result;
    LegIK_Angles angles;

    if (g == 0 || out == 0 || !isfinite(x_mm) || !isfinite(y_mm) ||
        !isfinite(g->l1_mm) || !isfinite(g->l2_mm) ||
        !isfinite(g->l3_mm) || !isfinite(g->l4_mm) || !isfinite(g->l5_mm) ||
        g->l1_mm <= 0.0f || g->l2_mm <= 0.0f ||
        g->l3_mm <= 0.0f || g->l4_mm <= 0.0f || g->l5_mm < 0.0f)
        return LEG_IK_ARGUMENT_ERROR;

    /* 固定为图中选定的张开姿态，左右采用镜像的弯折方向。 */
    result = SolveSide(g->l1_mm, g->l2_mm, x_mm + g->l5_mm * 0.5f,
                       y_mm, +1, &q1);
    if (result != LEG_IK_OK) return result;
    result = SolveSide(g->l4_mm, g->l3_mm, x_mm - g->l5_mm * 0.5f,
                       y_mm, -1, &q4);
    if (result != LEG_IK_OK) return result;

    /* 两侧各自可达仍不够：BP和DP共线时，五连杆存在并联奇异位形。 */
    bx = -g->l5_mm * 0.5f + g->l1_mm * cosf(q1);
    by = g->l1_mm * sinf(q1);
    dx =  g->l5_mm * 0.5f + g->l4_mm * cosf(q4);
    dy = g->l4_mm * sinf(q4);
    u1x = (x_mm - bx) / g->l2_mm;
    u1y = (y_mm - by) / g->l2_mm;
    u4x = (x_mm - dx) / g->l3_mm;
    u4y = (y_mm - dy) / g->l3_mm;
    cross = u1x * u4y - u1y * u4x;
    if (!isfinite(cross)) return LEG_IK_ARGUMENT_ERROR;
    if (fabsf(cross) < PARALLEL_SIN_EPS) return LEG_IK_SINGULAR;

    /* 强制钻石形：B在A左下、D在E右下，P在B/D之间且低于二者。
     * 排除边界附近的浮点误差；margin是数值容差，不是机械安全余量。 */
    margin = LENGTH_EPS * fmaxf(g->l5_mm,
                 fmaxf(fmaxf(g->l1_mm, g->l2_mm), fmaxf(g->l3_mm, g->l4_mm)));
    if (bx >= -g->l5_mm * 0.5f - margin || dx <= g->l5_mm * 0.5f + margin ||
        by <= margin || dy <= margin || y_mm <= fmaxf(by, dy) + margin ||
        x_mm <= bx + margin || x_mm >= dx - margin)
        return LEG_IK_POSTURE_ERROR;

    angles.q1_deg = Wrap360(q1 * RAD_TO_DEG);
    angles.q4_deg = Wrap360(q4 * RAD_TO_DEG);
    *out = angles;
    return LEG_IK_OK;
}

static uint8_t have_foot_command;
static float last_servo[4];

void Leg_ResetFootPositionHistory(void)
{
    have_foot_command = 0u;
}
LegIK_Result Leg_SetFootPositions(float x_mm[2], float y_mm[2])
{
    LegIK_Angles q[2];
    float command[4];
    LegIK_Result result;
    unsigned i;

    /* 左右各逆解一次；任意一侧失败都不更新舵机。 */
    for (i = 0; i < 2; ++i) {
        result = LegIK_Solve(&My_LegIK_Geometry, x_mm[i], y_mm[i], &q[i]);
        if (result != LEG_IK_OK) return result;
    }
    command[0] = Servo_Bias_1 - q[0].q1_deg;
    command[1] = Servo_Bias_2 - q[0].q4_deg;
    command[2] = Servo_Bias_3 - q[1].q1_deg;
    command[3] = Servo_Bias_4 - q[1].q4_deg;

    /* 超过步长直接拒绝，由上层减小目标步长，不在这里重复逆解。 */
    if (have_foot_command) {
        for (i = 0; i < 4; ++i)
            if (fabsf(command[i] - last_servo[i]) > FOOT_SERVO_STEP_MAX_DEG)
                return LEG_IK_STEP_ERROR;
    }
    /* 舵机驱动负责角度范围检查和四路统一提交，上层不重复检查。 */
    if (Servo_SetAngles(command) != SERVO_OK) return LEG_IK_SERVO_ERROR;
    for (i = 0; i < 4; ++i) last_servo[i] = command[i];
    Left_LegIK_Angles = q[0];
    Right_LegIK_Angles = q[1];
    have_foot_command = 1u;
    return LEG_IK_OK;
}

