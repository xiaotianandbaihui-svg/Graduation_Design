#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

/* STM32F103C8T6 + STM32F10x 标准库，HCLK=72 MHz，PCLK1=36 MHz，PCLK2=72 MHz。 */
#define MOTOR_POLE_PAIRS          7u       /* BDS3620：14 极，不是 14 对极 */
#define MOTOR_PWM_HZ              20000u
#define MOTOR_UPDATE_US           100u     /* 轮询内部限速到 10 kHz */
#define MOTOR_TIMEOUT_US          500u     /* 有输出时，更新失联则停机 */
#define MOTOR_MAX_DUTY            150      /* 千分比；初次调试最多 15%，不是限流 */
#define MOTOR_MAX_ALIGN_DUTY      100u     /* 标定最多 10%，不是限流 */
#define MOTOR_ENCODER_COUNTS      32768u
#define MOTOR_CPU_TICKS_PER_US    72u

#endif
