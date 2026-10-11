#ifndef MOTOR_EXAMPLE_H
#define MOTOR_EXAMPLE_H

#include "six_step.h"

/* 板1单次演示：初始化双路驱动，标定电机0，正向运行20000次换相更新后停止。
 * 调用前必须完成你自己的系统时钟、SPI1、CS及Delay_us初始化。
 * 本例会转动电机；轴须可自由转动，并按six_step.h要求使用限流电源。
 * 会重新初始化整个六步模块，不能在另一台电机运行时调用。
 * 只演示电机0，不保存Flash；重复调用会重新标定。
 * 阻塞执行，返回后电机0滑行停止；错误时双路关断。
 */
SixStep_Result Motor_Example_RunOnce(void);

#endif
