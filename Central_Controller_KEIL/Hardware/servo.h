#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>

/* 以下是暂定映射，不是已确认的BDS300规格；确认实际脉宽/行程后修改。 */
#define SERVO_PERIOD_US       20000u  /* PWM周期，四路共用 */
#define SERVO_MIN_PULSE_US      500u
#define SERVO_MAX_PULSE_US     2500u
#define SERVO_MAX_ANGLE_DEG     180.0f

typedef enum {
    SERVO_OK = 0,
    SERVO_ARGUMENT_ERROR = -1,
    SERVO_NOT_READY = -2,
    SERVO_CLOCK_ERROR = -3
} Servo_Result;

/* 核心控制板 STM32F103C8T6，标准库，TIM3硬件PWM，无中断、无DMA。
 * id   原理图信号   引脚   通道
* 0    PWM1         PA6    TIM3_CH1   左上舵机
* 1    PWM2         PA7    TIM3_CH2		左下舵机
* 2    PWM3         PB0    TIM3_CH3		右上舵机
* 3    PWM4         PB1    TIM3_CH4		右下舵机
 * PB1不能同时调用旧Key_Init配置成按键；四路引脚不能复用于SPI1/ADC。
 */

/* 初始化GPIO和TIM3；成功返回SERVO_OK(0)，失败返回负数。
 * period_us范围2~65535，计数分辨率1us；只读取SystemInit配置的系统时钟。
 * 初始化后四路保持低电平，设置脉宽/角度后才发脉冲。
 */
Servo_Result Servo_Init(uint16_t period_us);

/* 设置高电平脉宽(us)，id=0~3；要求pulse_us < 初始化周期。
 * 下一PWM周期生效，此后硬件自动重复输出，不必循环调用。
 * pulse_us=0停止该路脉冲，但不会切断舵机电源。
 * 此函数只检查定时器范围，实际舵机允许的脉宽由调用者确认。
 */
Servo_Result Servo_SetPulse(uint8_t id, uint16_t pulse_us);

/* 设置舵机目标角(度)，按上面的暂定映射换算；越界返回错误，不改变输出。
 * 默认0/90/180度对应500/1500/2500us；支持小数角度。
 * 这是舵机指令角，不是五连杆q1/q4；安装零位与方向应在上层换算。
 * 所有接口在主循环调用，不在中断中与主循环交叉调用。
 */
Servo_Result Servo_SetAngle(uint8_t id, float angle_deg);

#endif
