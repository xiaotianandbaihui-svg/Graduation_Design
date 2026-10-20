#ifndef BALANCE_PID_H
#define BALANCE_PID_H

#include <stdint.h>

/* 固定周期PID：时间因子并入参数，不再传dt。
 * I += ki*error；差分D = kd*(error-previous_error)。 */
typedef struct {
    float kp, ki, kd;
    float limit;                 /* 输出范围±limit，须为有限正数 */
    float integral;              /* 当前累计的I项，已包含ki */
    float previous_error;        /* 上一次误差 */
    float output;                /* 限幅后的输出 */
    uint8_t have_previous;       /* 是否已有历史误差；首次差分D=0 */
} PID;

/* 指针须有效，先Init；通用PID的有限参数/正limit由调用者保证。
 * 参数为0可禁用对应项；ki=0时清掉旧积分。
 * 更新结果异常返回NAN；保留限幅和条件积分。 */
void PID_Init(PID *p, float kp, float ki, float kd, float limit);
void PID_Reset(PID *p);           /* 清运行状态，保留参数 */
float PID_Update(PID *p, float error);

/* 主控足端控制：每次输出的是坐标增量(mm)，再加到上次成功下发的坐标。
 * 默认只用P；因此位置随误差逐周期累加，并非“位置=Kp*误差”。
 * 以下数值是调试入口，不是实测参数。改变周期后需要重新整定。 */
#define FOOT_PID_PERIOD_MS             20u
#ifndef FOOT_ROLL_KP
#define FOOT_ROLL_KP                   0.0f /* mm/(deg*次)，符号由实测纠偏方向决定 */
#endif
#ifndef FOOT_X_KP_LEFT
#define FOOT_X_KP_LEFT                 0.0f /* mm/(rpm*次)，左右可独立整定 */
#endif
#ifndef FOOT_X_KP_RIGHT
#define FOOT_X_KP_RIGHT                0.0f
#endif
#define FOOT_TARGET_ROLL_DEG           0.0f/*初始的ROLL目标角度，后面或许会更改*/
#define FOOT_TARGET_RPM_LEFT           0.0f/*左轮初始目标速度，后续可能会修改*/
#define FOOT_TARGET_RPM_RIGHT          0.0f/*右轮初始目标速度，后续可能会修改*/

/* CAN轮速当前以各电机电角度增加方向为正；这里转换为车前进为正。
 * 两项应与电驱板的car_forward_electircal_forward_1/2一致。 */
#ifndef FOOT_WHEEL_SIGN_LEFT
#define FOOT_WHEEL_SIGN_LEFT           1
#endif
#ifndef FOOT_WHEEL_SIGN_RIGHT
#define FOOT_WHEEL_SIGN_RIGHT          1
#endif

#define FOOT_ROLL_DEADBAND_DEG         0.2f
#define FOOT_SPEED_DEADBAND_RPM        1.0f
#define FOOT_Y_STEP_MAX_MM             0.5f /* 每次每条腿最大y增量，带载实测后修改 */
#define FOOT_X_STEP_MAX_MM             0.5f /* 每次每条腿最大x增量，带载实测后修改 */
/* 坐标沿用leg_ik.h：+y向下。以下工作区/初始点须按实体机构确认。 */
#define FOOT_X_MIN_MM                (-20.0f)
#define FOOT_X_MAX_MM                 20.0f
#define FOOT_Y_MIN_MM                 60.0f
#define FOOT_Y_MAX_MM                120.0f
#define FOOT_INITIAL_X_MM              0.0f
#define FOOT_INITIAL_Y_MM            100.0f
#define FOOT_INITIAL_SETTLE_MS        500u /* 初始位姿等待时间，须实测；不是到位反馈 */

typedef struct {
    PID roll;                      /* 正输出：左腿y增加，右腿y减少 */
    PID x[2];                      /* 左右轮各自控制对应足端x */
    float target_roll_deg;
    float target_rpm[2];            /* 车前进为正 */
} FootPID;

void FootPID_Init(FootPID *p);
/* 只计算，不驱动舵机；1成功，0输入/参数无效且不修改输出和PID状态。
 * current_x/y是上次成功下发的指令坐标，不是实测足端位置。
 * roll输入为主控angle[2]。输出再统一交给Leg_SetFootPositions。
 * Roll两侧等量反向调整，到边界时共同缩小增量，保持平均腿高。
 * 保持默认ki=kd=0；若启用它们，须另行考虑执行器限速/限位的抗积分饱和。 */
int FootPID_Update(FootPID *p, float roll_deg, const float wheel_rpm[2],
                   const float current_x[2], const float current_y[2],
                   float next_x[2], float next_y[2]);

#endif

