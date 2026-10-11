#ifndef MOTOR_PORT_H
#define MOTOR_PORT_H

#include <stdint.h>
#include "motor_config.h"

/* 内部硬件接口。应用通常只需包含 motor.h。 */
#define PORT_FAULT_DRV1       0x0100u
#define PORT_FAULT_DRV2       0x0200u
#define PORT_FAULT_TIMEOUT    0x0400u
#define PORT_FAULT_USER       0x0800u

int      MotorPort_Init(void);
int      MotorPort_Enable(void);
uint32_t MotorPort_Ticks(void);     /* DWT 原始周期计数，使用无符号减法处理回绕 */
void     MotorPort_DelayUs(uint32_t us);
int      MotorPort_SSCRead(uint8_t id, uint16_t command,
                           uint16_t *data, uint16_t *safety);
void     MotorPort_Apply(uint8_t id, uint8_t vector, uint16_t duty);
void     MotorPort_Stop(uint8_t id);
void     MotorPort_Kill(uint32_t fault);
uint32_t MotorPort_Fault(void);
void     MotorPort_Feed(void);

#endif
