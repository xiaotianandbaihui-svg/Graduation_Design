#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>

/* 暂定控制参数：按用户提供的通用180度舵机关系配置。
 * BDS300产品图只给出供电、扭矩、速度、接线，未确认以下控制范围。
 * 后续拿到BDS300的具体协议或实测结果时，集中修改这里。 */
#define SERVO_PERIOD_US       20000u
#define SERVO_MIN_PULSE_US      500u
#define SERVO_MAX_PULSE_US     2500u
#define SERVO_MAX_ANGLE_DEG     180.0f

typedef enum {
    SERVO_OK = 0,
    SERVO_ARGUMENT_ERROR = -1,
    SERVO_NOT_READY = -2,
    SERVO_CLOCK_ERROR = -3
} Servo_Result;

/* 板3，STM32F103C8T6标准库；独占TIM3，不占用中断或延时函数。
 * id=0: PA6 / TIM3_CH1 / PWM1
 * id=1: PA7 / TIM3_CH2 / PWM2
 * id=2: PB0 / TIM3_CH3 / PWM3
 * id=3: PB1 / TIM3_CH4 / PWM4
 * 与板1电机程序是两块MCU；不能把这组引脚同时用于板1的SPI1/ENABLE/CS。
 */

/* period_us是重复周期，单位微秒，范围2~65535，四个通道共用。
 * 读取启动阶段SystemInit已配置的时钟，配置TIM3为1MHz计数；不重设系统时钟。
 * 常规调用Servo_Init(SERVO_PERIOD_US)，即暂定20ms周期。
 * 初始化后四路均为低电平，不自动给舵机发送位置指令。
 * 再次成功初始化会停止全部通道，并丢弃此前脉宽。
 * 参数/时钟检查失败时保留此前配置。运行中不要另行修改系统时钟。
 * BDS300允许的周期、脉宽范围仍须按成品舵机资料确认。
 */
Servo_Result Servo_Init(uint16_t period_us);

/* pulse_us是高电平持续时间（微秒），不是占空比；id范围0~3。
 * 要求pulse_us < period_us；这里只检查定时器范围，不保证舵机机械范围。
 * 设置值在下一次定时器更新事件生效，最多等待一个完整周期。
 * pulse_us=0：下一周期起不再发脉冲；不会切断舵机电源，也不保证释放力矩。
 * 硬件自动重复发脉冲，无需循环调用；只在需要修改脉宽时调用。
 * 非法参数不改变输出。所有接口由同一执行上下文调用。
 */
Servo_Result Servo_SetPulse(uint8_t id, uint16_t pulse_us);

/* 按上述暂定映射设置目标角度，单位度，允许0~SERVO_MAX_ANGLE_DEG。
 * 默认0/45/90/135/180度分别对应500/1000/1500/2000/2500us。
 * 支持小数角度，换算脉宽四舍五入到整数微秒；越界或NaN返回参数错误。
 * 当前周期必须大于SERVO_MAX_PULSE_US，保证整段角度映射都能输出。
 * 这是舵机目标角度，不是实测角度，也不等于腿部运动学定义的关节角度。
 */
Servo_Result Servo_SetAngle(uint8_t id, float angle_deg);

#endif
