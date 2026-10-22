#ifndef MOTOR_CAN_PITCH_H
#define MOTOR_CAN_PITCH_H

#include <stdint.h>

/* 主控端：STM32F103C8T6，PA11=RX、PA12=TX，500 kbit/s。 */
#define CAN_ANGLE_LEGHEIGHT_ID 0x101u  /* 发送当前的角度和当前腿高 */
#define CAN_TARGET_ANGLE_ID    0x100u	 /*发送目标角度*/
#define CAN_SPEED_ID    0x200u

/* 在系统时钟初始化之后调用；1成功，0时钟不支持或CAN初始化失败。
 * 独占CAN1和滤波器0，不配置系统时钟、不使用定时器。 */
int CAN_Init_Together(void);

/* 原子取走一帧左右轮速并清通知；1有新帧，0无新帧且不修改rpm。
 * rpm[0]=左轮，rpm[1]=右轮，方向仍为各电机电角度正方向。
 * 当前协议int16*0.01 rpm，只能表示[-327.68,327.67] rpm。 */
int CAN_TakeWheelSpeed(float rpm[2]);

/* 第一包：Pitch、Yaw、左腿高、右腿高，四个int16小端，共8字节；
 * 第二包：目标Pitch、目标Yaw，两个int16小端，共4字节。
 * 角度单位0.01度，腿高单位0.01mm；每项范围[-327.68,327.67]。
 * 1已提交发送邮箱；0邮箱忙、输入非有限或超范围。
 * 返回1不代表对方已经收到；不等待发送完成。 */
int CAN_Angle_and_LegHeight_Send_First_Package(float pitch_deg, float yaw_deg,float left_leg_height,float right_leg_height);
int CAN_Angle_and_LegHeight_Send_Second_Package(float target_pitch_deg, float target_yaw_deg);
int CAN_Send_The_Wheels_Speed(float* left_rpm ,float* right_rpm);

#endif
