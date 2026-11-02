#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdio.h>
#include <stdint.h>

#define SERIAL_BAUD_RATE          115200u /* USART1：PA9发送、PA10接收，8N1 */
#define SERIAL_CONTROL_HEAD1      0xAAu
#define SERIAL_CONTROL_HEAD2      0x55u
#define SERIAL_CONTROL_FRAME_SIZE 8u
#define SERIAL_FEEDBACK_FRAME_SIZE 7u
#define SERIAL_LORA_TIMEOUT_MS    10000u /* 超过此间隔没有完整有效目标帧，目标速度归零 */
#define SERIAL_TX_TIMEOUT_MS      18u    /* 本地串口发送等待上限，不是无线响应超时 */

/* 中断置位、Serial_ProcessControl消费：bit0有新目标，bit1有跳跃沿。 */
extern volatile uint8_t Serial_ControlPending;
extern volatile uint8_t Lora_Timeout; /* 超时事件标志；收包不自动清零，使用者可清零 */
/* 最近完整收包时刻；超时处理也更新它，开始下一轮10秒检查。不要用它计算累计离线时间。 */
extern volatile uint32_t Lora_LastRxCycles;

/* 接收控制帧（8字节二进制；串口助手选择HEX发送，不添加回车换行，注意，一点不能加回车换行，不然数据格式就不对了）：
 * [0..1] AA 55
 * [2..3] int16_t目标速度，小端，数值=rpm*10；范围-3276.8~3276.7rpm
 * [4..5] int16_t目标Yaw，小端，数值=角度*100；范围-18000~18000
 * [6]    跳跃按钮：0/1；有效帧中0->1请求一次，下一次跳跃前须先发0
 * [7]    [0..6]各字节相加，取低8位
 * 例：AA 55 E8 03 28 23 00 35 = 100rpm、Yaw 90度、不跳跃。
 * Yaw是与姿态解算同一参考系的绝对目标角，不是转角增量。
 * 校验失败/参数越界整帧丢弃；多帧待处理时保留最新速度/Yaw，跳跃请求保留。
 * 10秒未收有效帧时，目标速度及未执行跳跃请求清零，保留目标Yaw。
 * printf/Serial_Printf仍可同时输出调试文本，发送接口保持原用法。
 */
/* 高频主循环调用（包括无新帧时）：返回1表示消费了新目标，需要回传一次实测数据。 */
int Serial_ProcessControl(void);
/* 响应：55 AA | int16小端实测rpm*10 | int16小端实测Yaw*100 | 前6字节和低8位。
 * speed_rpm取两轮前进方向归一后的平均值。返回1=UART发完，0=参数无效/发送超时。 */
int Serial_SendFeedback(float speed_rpm, float yaw_deg);

void Serial_Init(void);
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array, uint16_t Length);
void Serial_SendString(char *String);
void Serial_SendNumber(uint32_t Number, uint8_t Length);
void Serial_Printf(char *format, ...);

uint8_t Serial_GetRxFlag(void);
uint8_t Serial_GetRxData(void);

#endif
