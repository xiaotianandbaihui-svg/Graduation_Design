#ifndef CURRENT_LOOP_H
#define CURRENT_LOOP_H

#include <stdint.h>

/* 有位置传感器的FOC电流环：Clarke -> Park -> d/q双PI -> 逆Park。
 * 每台电机一个CurrentLoop对象；无ADC、GPIO、定时器或现有主循环依赖。
 * 当前硬件没有电流采样：本文件只提供逻辑，底层空接口默认报告失败。
 */
typedef struct {
    float kp_d, ki_d;          /* Kp: V/A；Ki: V/(A*s)，积分显式乘周期 */
    float kp_q, ki_q;
    float period_s;            /* 固定调用周期，单位秒，须与实际采样周期一致 */
    float current_limit_a;     /* 目标sqrt(Id^2+Iq^2)上限，A；不是硬件过流保护 */
    float voltage_ratio;       /* (0,1]，电压矢量上限=此比例*Vbus/sqrt(3) */
} CurrentLoop_Config;

typedef struct {
    float ia_a, ib_a;          /* 同一采样时刻的A/B相电流，A；正方向逆变器->电机 */
    float electrical_deg;      /* 同时刻转子d轴电角度，度；0沿A相，正向A->B->C */
    float bus_voltage_v;       /* 实际直流母线电压，V */
} CurrentLoop_Feedback;

typedef struct {
    CurrentLoop_Config config;
    uint8_t motor_id;
    uint8_t running;
    float integral_d, integral_q; /* 已乘Ki的积分项，V */
    float id_a, iq_a;             /* 实测d/q电流，A，便于调试 */
    float ud_v, uq_v;             /* 限幅后的d/q电压，V */
} CurrentLoop;

/* 对象/配置指针须有效。Init只保存配置并清状态，不启动PWM。
 * 停止后才能改参数或重新Init；同一对象由一个执行上下文调用。 */
void CurrentLoop_Init(CurrentLoop *loop, uint8_t motor_id,
                      const CurrentLoop_Config *config);
int CurrentLoop_Start(CurrentLoop *loop); /* 1成功，0配置无效；清旧积分 */
void CurrentLoop_Stop(CurrentLoop *loop); /* 清积分，并调用底层关闭PWM接口 */

/* 每取得一组新的有效采样调用一次；不能直接沿用20~50ms的CAN周期。
 * target_id_a通常设0；target_iq_a是带符号的转矩电流指令，单位A。
 * 正Iq对应ABC坐标内的正转矩，车体前进方向仍须按左右轮安装关系换算。
 * 目标电流做圆形限幅；两个PI输出合成后再按母线电压做圆形限幅。
 * 返回1=成功下发；0=未启动或失败。采样/计算/输出失败会Stop，须显式Start恢复。
 * 没有位置/速度外环、弱磁或交叉耦合前馈，属于基础d/q PI电流控制。
 */
int CurrentLoop_Update(CurrentLoop *loop, float target_id_a, float target_iq_a);

/* ---------- 待实现的三个底层接口 ----------
 * 默认空实现位于.c末尾。可以直接替换；也可对current_loop.c定义
 * CURRENT_LOOP_EXTERNAL_IO，由另一个.c提供这三个函数。
 */

/* 1=读取成功，0=失败/没有本周期新数据。
 * 实现时完成ADC零偏/增益/电流方向换算，必要的相电流重构以及角度零点校准。
 * ia+ib+ic=0用于两相采样重构；不能把直流母线电流当成ia/ib。
 * 低侧分流器只能在有效PWM采样窗口取值；这里不负责采样触发。
 */
int CurrentLoop_ReadFeedback(uint8_t motor_id, CurrentLoop_Feedback *feedback);

/* 1=本周期电压矢量接受成功，0=失败。
 * 输入为静止alpha/beta坐标系电压(V)，交给逆Clarke/SVPWM更新三相占空比。
 * alpha轴沿A相，beta轴沿ABC正方向旋转90度。
 * 零电压矢量仍是运行中的控制指令，应输出三相相等的占空比；不等于高阻停机。
 * 可用角度atan2(beta,alpha)、幅值sqrt(alpha^2+beta^2)适配电压型SVPWM；
 * 现有SVPWM_Update只支持正/负q轴电压，不能直接接收完整d/q PI输出。
 */
int CurrentLoop_WriteVoltage(uint8_t motor_id, float alpha_v, float beta_v);

/* 必须真正关闭该电机驱动输出，不能只清软件变量。当前空实现不操作硬件。 */
void CurrentLoop_DisablePWM(uint8_t motor_id);

#endif
