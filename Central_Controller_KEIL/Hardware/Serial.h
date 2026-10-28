#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdio.h>
#include <stdint.h>

#define SERIAL_BAUD_RATE          115200u /* USART1：PA9发送、PA10接收，8N1 */
#define SERIAL_CONTROL_HEAD1      0xAAu
#define SERIAL_CONTROL_HEAD2      0x55u
#define SERIAL_CONTROL_FRAME_SIZE 8u

/* 接收控制帧（8字节二进制；串口助手选择HEX发送，不添加回车换行，注意，一点不能加回车换行，不然数据格式就不对了）：
 * [0..1] AA 55
 * [2..3] int16_t目标速度，小端，数值=rpm*10；范围-3276.8~3276.7rpm
 * [4..5] int16_t目标Yaw，小端，数值=角度*100；范围-18000~18000
 * [6]    跳跃按钮：0/1；有效帧中0->1请求一次，下一次跳跃前须先发0
 * [7]    [0..6]各字节相加，取低8位
 * 例：AA 55 E8 03 28 23 00 35 = 100rpm、Yaw 90度、不跳跃。
 * Yaw是与姿态解算同一参考系的绝对目标角，不是转角增量。
 * 校验失败/参数越界整帧丢弃；多帧待处理时保留最新速度/Yaw，跳跃请求保留。
 * 无新帧时保持目标，本接口不增加遥控掉线超时逻辑。
 * printf/Serial_Printf仍可同时输出调试文本，发送接口保持原用法。
 */
void Serial_ProcessControl(void); /* 主循环调用：更新target_Speed、目标Yaw和Jump_Request */

void Serial_Init(void);
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array, uint16_t Length);
void Serial_SendString(char *String);
void Serial_SendNumber(uint32_t Number, uint8_t Length);
void Serial_Printf(char *format, ...);

uint8_t Serial_GetRxFlag(void);
uint8_t Serial_GetRxData(void);

#endif
