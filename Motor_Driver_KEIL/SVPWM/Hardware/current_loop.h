#ifndef CURRENT_LOOP_H
#define CURRENT_LOOP_H

/* 基础FOC：Id目标固定0，只接收Iq目标；硬件启停由motor_current负责。 */
typedef struct {
    float kp_d, ki_d;          /* Kp: V/A；Ki: V/(A*s) */
    float kp_q, ki_q;
    float period_s;            /* 固定控制周期，初始化时预计算Ki*Ts */
    float current_limit_a;     /* 目标Iq限幅，A；不是硬件过流保护 */
    float voltage_ratio;       /* (0,1]；1对应Vbus/sqrt(3)，较小值留电压余量。
                               * 板级驱动还会按低侧采样窗口限制此比例。 */
} CurrentLoop_Config;

typedef struct {
    float ia_a, ib_a;          /* A/B相电流，A；正方向为星点->绕组->逆变器 */
    float electrical_deg;      /* 采样附近的转子d轴电角度，度；0沿A相，正向ABC */
    float bus_voltage_v;       /* 母线电压，V；当前板级驱动使用可配置估计值 */
} CurrentLoop_Feedback;

typedef struct {
    float alpha_v, beta_v;     /* 静止坐标系电压，V；相端对星点为正，这是PI(D)计算出来的最终的电压 */
} CurrentLoop_Voltage;

typedef struct {
    float kp, ki_step;         /* ki_step=Ki*Ts，其实就是积分项 */
    float integral;            /* 已乘Ki的积分项，累计值 */
} CurrentLoop_PI;

typedef struct {
    CurrentLoop_PI d, q;       /* 两轴共用同一种PI */
    float current_limit_a, voltage_ratio;/*一个是电流限幅(电流限幅限制的是目标电流的幅度)，一个是电压比例，这个电压比例是会基于ADC的配置情况去设置的，SVPWM_StartCurrent函数会设置它*/
    float id_a, iq_a;          /* 本拍实测电流，A，便于调参观察 */
    float ud_v, uq_v;          /* 本拍限幅后的电压，V */
} CurrentLoop;

/* 所有指针须有效；同一对象串行调用。修改参数后须在停机时重新Init。
 * Init返回1成功、0配置无效；Reset只清积分和观测值，保留参数。 */
int CurrentLoop_Init(CurrentLoop *loop, const CurrentLoop_Config *config);
void CurrentLoop_Reset(CurrentLoop *loop);

/* 每取得一组新采样调用一次：坐标变换->双PI->电压限幅->逆变换。
 * Id目标始终为0；target_iq_a与反馈均以星点向外为正。
 * 只计算，不读ADC、不操作PWM、不保存另一套运行状态。
 * 返回1时将voltage交给SVPWM；返回0时调用方必须关闭电机输出。
 * 输出零电压仍是控制指令，不等于高阻停机。 */
int CurrentLoop_Update(CurrentLoop *loop, const CurrentLoop_Feedback *feedback,
                       float target_iq_a, CurrentLoop_Voltage *voltage);

#endif
