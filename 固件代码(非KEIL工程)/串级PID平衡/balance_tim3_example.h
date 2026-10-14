#ifndef BALANCE_TIM3_EXAMPLE_H
#define BALANCE_TIM3_EXAMPLE_H

#include "balance_pid.h"

extern volatile uint8_t PID_Refresh_Flag;

/* 仅设置TIM3：5ms更新中断，中断只置刷新标志；不改系统时钟。 */
int Balance_TIM3_Init(void);
void Balance_TIM3_Stop(void);

/* command为Balance_Update得到的±1000命令，正负直接对应电角度方向。
 * 拆出SVPWM需要的两个方向和非负幅值；数组均须有2个元素。 */
void Balance_CommandToDrive(const int16_t command[2],
                            int8_t dir[2], uint16_t modulation[2]);

/* main例程用到的应用接口，由你对接已有初始化和俯仰角读取函数。
 * Board_Init只初始化SPI1/CS/IMU或通信等，不要重复配置系统时钟。
 * ReadPitch返回前倾为正的俯仰角（度）；均返回1成功/0失败。 */
int Board_Init(void);
int Balance_ReadPitch(float *pitch_deg);

#endif