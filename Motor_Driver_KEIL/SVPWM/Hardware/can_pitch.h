#ifndef MOTOR_CAN_PITCH_H
#define MOTOR_CAN_PITCH_H

#include <stdint.h>

/* 电驱端：STM32F103C8T6，PA11=RX、PA12=TX，500 kbit/s。 */
#define CAN_ANGLE_LEGHEIGHT_RX_ID 0x101u  /* 主控 -> 电驱 */
#define CAN_ANGLE_LEGHEIGHT_TX_ID 0x102u  /* 电驱 -> 主控 */
#define CAN_SPEED_ID    0x103u
/* 本模块定义这两个变量，main.c不要重复定义。
 * 同一报文更新[0]=Pitch、[1]=Yaw；电驱端不保存Roll，单位度。 */

/* 在系统时钟初始化之后调用；1成功，0时钟不支持或CAN初始化失败。
 * 独占CAN1和滤波器0，不配置系统时钟、不使用定时器。 */
int CAN_Init_Together(void);

/* 一帧回传Pitch、Yaw：两个int16，小端，单位0.01度，共4字节。
 * 每轴输入范围[-327.68,327.67]度，四舍五入至0.01度。
 * 1已提交发送邮箱；0邮箱忙、输入非有限或超范围。
 * 返回1不代表对方已经收到；不等待发送完成。 */
int CAN_Angle_and_LegHeight_Send(float pitch_deg, float yaw_deg,float left_leg_height,float right_leg_height);
int CAN_Send_The_Wheels_Speed(float* left_rpm ,float* right_rpm);

#endif
