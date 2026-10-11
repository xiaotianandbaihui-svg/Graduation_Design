#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include "motor_config.h"
#include "tle5012b.h"

typedef struct {
    uint16_t zero_raw;       /* A+、B-、C悬空对齐时的机械角计数 */
    int8_t encoder_sign;    /* 相矢量正向转动时，读数增加=+1，减少=-1 */
    uint8_t pole_pairs;     /* 本电机为7 */
} MotorCalibration;

typedef enum {
    MOTOR_OK = 0,
    MOTOR_BAD_ARGUMENT = -1,
    MOTOR_NOT_READY = -2,
    MOTOR_NOT_CALIBRATED = -3,
    MOTOR_FAULTED = -4,
    MOTOR_CALIBRATION_FAILED = -5
} MotorResult;

#define MOTOR_FAULT_ENCODER1     0x0001u
#define MOTOR_FAULT_ENCODER2     0x0002u
#define MOTOR_FAULT_CALIBRATION  0x0004u
#define MOTOR_FAULT_INIT         0x0008u
#define MOTOR_FAULT_DRV1         0x0100u
#define MOTOR_FAULT_DRV2         0x0200u
#define MOTOR_FAULT_TIMEOUT      0x0400u
#define MOTOR_FAULT_USER         0x0800u

/* mask=1：只读编码器1/控制电机1；mask=2：电机2；mask=3：两路。
 * 始终把两路功率级初始化为关闭，因为两片驱动器共用 ENABLE。 */
MotorResult Motor_Init(uint8_t mask);

/* 主循环连续调用，内部每100us执行一次。不能用 Delay_ms(1) 代替！
 * 与其余 Motor_* 命令使用同一执行上下文，不在另一个中断并发调用。 */
void Motor_Update(void);

/* 低占空比自动检查方向并标定零位。阻塞约2.4秒，期间会转动电机。
 * 标定前必须两路都停止、轴可自由转动，使用限流电源。
 * 成功后在RAM中生效，同时返回可保存的数据；不自动擦写Flash。 */
MotorResult Motor_Calibrate(uint8_t id, uint16_t duty, MotorCalibration *result);
MotorResult Motor_SetCalibration(uint8_t id, const MotorCalibration *calibration);

/* duty为有符号千分比，例如+50=正向5%，-50=反向5%，0=停止。
 * 这是电压/PWM指令，不是转速闭环，也不是力矩/电流闭环。 */
MotorResult Motor_Run(uint8_t id, int16_t duty);
void Motor_Stop(uint8_t id);             /* 关闭该路6个输入，滑行停止 */
void Motor_EmergencyStop(void);          /* 共享ENABLE拉低，双路锁存停机 */
uint32_t Motor_GetFault(void);           /* 位掩码，0=没有锁存故障 */
TLE_Result Motor_GetEncoderError(uint8_t id);
MotorResult Motor_GetAngle(uint8_t id, uint16_t *raw, float *degrees);

/* 纯计算：由机械角和标定计算6个电压矢量之一；供学习/测试。 */
uint8_t Motor_AngleToVector(uint16_t raw, const MotorCalibration *cal, int direction);

#endif
