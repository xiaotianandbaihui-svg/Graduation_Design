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

/* 主控足端控制：输出下一组足端坐标(mm)，默认只用P。
 * Roll左右等量反向纠偏，平均腿高左右等量同向调整；二者合成后统一下发。
 * x输出作为配平点偏移，两腿逐步接近同一目标x。
 * 以下数值是调试入口，不是实测参数。改变周期后需要重新整定。 */
#define FOOT_PID_PERIOD_MS             20u
#ifndef FOOT_ROLL_KP
#define FOOT_ROLL_KP                   0.1f /* mm/(deg*次)，符号由实测纠偏方向决定 */
#endif
#ifndef FOOT_X_KP_LEFT
#define FOOT_X_KP_LEFT                 0.1f /* 共同x环Kp，单位mm/rpm */
#endif
#ifndef FOOT_X_KP_RIGHT
#define FOOT_X_KP_RIGHT                0.1f
#endif
#define FOOT_TARGET_ROLL_DEG           0.0f/*初始的ROLL目标角度，后面或许会更改*/
#define FOOT_TARGET_RPM_LEFT           0.0f /* 共同x环目标平均轮速 */
#define FOOT_TARGET_RPM_RIGHT          0.0f /* 当前共同x环不使用，保留旧接口 */

/* CAN轮速当前以各电机电角度增加方向为正；这里转换为车前进为正。
 * 两项应与电驱板的car_forward_electircal_forward_1/2一致。 */
#ifndef FOOT_WHEEL_SIGN_LEFT
#define FOOT_WHEEL_SIGN_LEFT           1
#endif
#ifndef FOOT_WHEEL_SIGN_RIGHT
#define FOOT_WHEEL_SIGN_RIGHT          1
#endif

#define FOOT_ROLL_DEADBAND_DEG         1.0f
#define FOOT_SPEED_DEADBAND_RPM        1.0f
#define FOOT_Y_STEP_MAX_MM             2.0f /* 每次每条腿最大y增量，带载实测后修改 */
#define FOOT_X_STEP_MAX_MM             2.0f /* 每次每条腿最大x增量，带载实测后修改 */
/* 坐标沿用leg_ik.h：+y向下。以下工作区/初始点须按实体机构确认。 */
#define FOOT_X_MIN_MM                (-20.0f)
#define FOOT_X_MAX_MM                 20.0f
#define FOOT_Y_MIN_MM                 60.0f
#define FOOT_Y_MAX_MM                120.0f
#define FOOT_INITIAL_X_MM              0.0f
#define FOOT_INITIAL_Y_MM            100.0f
#define FOOT_INITIAL_SETTLE_MS        500u /* 初始位姿等待时间，须实测；不是到位反馈 */

/* 普通运行时的平均腿高调节，+y向下，增大数值表示伸长腿。
 * Roll优先，升降只使用剩余步长；较大纠偏时允许暂缓升降。 */
#define FOOT_MEAN_HEIGHT_STEP_MAX_MM    1.0f /* 每次平均腿高最大变化，mm，带载实测后修改 */
#define FOOT_ROLL_HEIGHT_RESERVE_MM     5.0f /* 平均目标距上下限的余量，给Roll差动留空间 */
#define FOOT_MEAN_HEIGHT_MIN_MM (FOOT_Y_MIN_MM + FOOT_ROLL_HEIGHT_RESERVE_MM)
#define FOOT_MEAN_HEIGHT_MAX_MM (FOOT_Y_MAX_MM - FOOT_ROLL_HEIGHT_RESERVE_MM)

#define SPEED_KP              0.1f /* 度/rpm，待调参 */
#define SPEED_KI              0.05f /* 度/(rpm·次)，按20ms周期调参 */
#define SPEED_PITCH_MAX_DEG   45.0f /* 目标Pitch限幅，调试初值 */


typedef struct {
    PID roll;                      /* 正输出：左腿y增加，右腿y减少 */
    PID x[2];                      /* [0]为共同x环，[1]保留旧接口 */
    float target_roll_deg;
    float target_rpm[2];            /* [0]目标平均轮速，车前进为正；[1]保留 */
    float target_mean_height_mm;    /* 目标平均足端y，mm；不是离地高度测量值 */
} FootPID;

void FootPID_Init(FootPID *p);
/* 在主循环处理升降命令时调用，p须有效；只改目标，不清Roll历史、不直接驱动舵机。
 * 有限目标限幅到MEAN_HEIGHT_MIN/MAX；返回1成功，0非有限输入且保留旧目标。
 * 跳跃期间普通FootPID仍暂停；解锁后从恢复位置逐步接近此目标。 */
int FootPID_SetMeanHeight(FootPID *p, float height_mm);
/* 只计算，不驱动舵机；1成功，0输入/参数无效且不修改输出和PID状态。
 * current_x/y是上次成功下发的指令坐标，不是实测足端位置。
 * roll输入为主控angle[2]。输出再统一交给Leg_SetFootPositions。
 * 先计算Roll差动，再按目标平均腿高计算共同升降，始终保留Roll纠偏。
 * |共同升降量|+|Roll差动量|不超过FOOT_Y_STEP_MAX_MM，避免叠加后超步长。
 * 某腿到限位时限制共同升降，不强行压平左右高度差；目标可能暂时无法到达。
 * 平均值每次从current_y计算，只有下发成功后更新它们，拒绝的指令不会累计升降。
 * 保持默认ki=kd=0；若启用它们，须另行考虑执行器限速/限位的抗积分饱和。 */
int FootPID_Update(FootPID *p, float roll_deg, const float wheel_rpm[2],
                    float current_x[2], float current_y[2],
                   float next_x[2], float next_y[2]);

void SpeedPID_Init(PID *p);/*供电机驱动使用*/

int SpeedPID_Update(PID *p, float target_rpm,
                    const float wheel_rpm[2], float *target_pitch);

#endif

