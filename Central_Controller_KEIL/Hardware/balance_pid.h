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

typedef struct {
    PID angle;                   /* 倾角外环，输出目标轮速rpm */
    PID speed[2];                /* 左右轮内环，输出有符号调制幅值 */
    float upright_deg;           /* 目标直立角 */
    float fall_deg;              /* 偏离直立角超过此值时停控制 */
    uint8_t running;
} BalancePID;

/* 保留当前设置：目标轮速±1000rpm、输出±1000、倾倒阈值60度。
 * 增益默认0，由你按固定控制周期整定；所有对象指针须有效。
 * Start只在启动时检查配置，更新时不重复扫描配置。
 * 修改参数请在停止状态进行，之后重新Start。 */
void Balance_Init(BalancePID *b);
int Balance_Start(BalancePID *b);  /* 清状态并启动，1成功/0配置错误 */
void Balance_Stop(BalancePID *b);  /* 只停运算，实际电机仍需停止命令 */

/* 固定周期调用，输出command[0/1]为左右轮有符号幅值。
 * 前倾角为正；两轮机械rpm及命令以各自电角度增加方向为正。
 * 外环：误差=实际角-目标角；目标轮速方向由外环增益极性决定。
 * 左右内环共用此目标，外部须保证两轮对共同目标的方向约定一致。
 * 内环：误差=目标轮速-实际轮速；各环首次D=0，差分均不除时间。
 * 返回1成功；未启动/倾倒/异常结果返回0，清零两路并停止此实例。
 * b和command须有效；本次读取失败不要复用旧传感器数据。 */
int Balance_Update(BalancePID *b, float tilt_deg,
                   float left_rpm, float right_rpm, int16_t command[2]);

int Balance_ReadFeedback(float *pitch, float *left_rpm, float *right_rpm);
#endif

