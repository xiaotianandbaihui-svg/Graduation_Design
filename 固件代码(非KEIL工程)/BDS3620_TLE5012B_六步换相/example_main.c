/* 独立示例；整合到已有工程时不要与自己的main同时编译。
 * 标准库工程的SystemInit需已配置为8MHz HSE -> 72MHz。
 * 默认为只读角度、不标定、不转动。 */
#include "stm32f10x.h"
#include "misc.h"
#include "motor.h"

#ifndef EXAMPLE_CALIBRATE_AND_RUN
#define EXAMPLE_CALIBRATE_AND_RUN 0    /* 完成README检查后，手动改1才会转动 */
#endif

volatile uint16_t encoder_raw;
volatile float encoder_degrees;
volatile uint32_t motor_fault;
volatile MotorResult init_result;
volatile MotorResult run_result;
MotorCalibration measured_calibration; /* 调试器查看；成功后可自行存入Flash */

int main(void)
{
    uint16_t raw;
    float degrees;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    init_result = Motor_Init(0x01u);    /* 1=电机1；3=同时启用两只编码器 */
    if (init_result != MOTOR_OK) {
        motor_fault = Motor_GetFault();
        while (1) { }                 /* 查故障，不循环自动重启电机 */
    }

#if EXAMPLE_CALIBRATE_AND_RUN
    /* 对齐电流取决于电机绕组，5%也可能过大/过小；必须限流、空载标定。 */
    run_result = Motor_Calibrate(0u, 50u, &measured_calibration);
    if (run_result == MOTOR_OK) run_result = Motor_Run(0u, 50); /* 正向5% */
#else
    /* 若已有本台电机实测的标定，可改为：
     * MotorCalibration saved = {实际zero_raw, 实际encoder_sign, 7};
     * Motor_SetCalibration(0, &saved);
     * Motor_Run(0, 50);
     * 不要直接复制别的电机的零位数字。
     */
#endif

    while (1) {
        Motor_Update();
        if (Motor_GetAngle(0u, &raw, &degrees) == MOTOR_OK) {
            encoder_raw = raw;
            encoder_degrees = degrees;
        }
        motor_fault = Motor_GetFault();
        /* 在这里处理非阻塞按键/通信：Motor_Stop(0)、Motor_Run(0,-50)等。
         * 不放Delay_ms、阻塞串口打印、Flash擦写；不从另一个中断调用SPI读取。
         * 出现锁存故障后，确认原因再显式Motor_Init并重新装载标定。
         */
    }
}
