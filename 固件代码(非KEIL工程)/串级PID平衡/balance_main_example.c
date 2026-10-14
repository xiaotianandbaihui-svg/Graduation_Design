#include "balance_tim3_example.h"
#include "svpwm.h"
#include "tle5012b.h"

BalancePID balance_pid; /* 全局平衡PID对象 */

int main(void)
{
    int16_t command[2];
    int8_t dir[2] = {1, 1};
    uint16_t modulation[2] = {0u, 0u};
    float pitch, left_dps, right_dps, left_rpm, right_rpm;

    Balance_Init(&balance_pid);
    /* 在这里填写angle、speed[0]、speed[1]各自的kp/ki/kd，默认均为0。 */
    if (!Balance_Start(&balance_pid)) goto stop;

    if (!Board_Init() || !Motor_init() || !Balance_TIM3_Init()) goto stop;

    while (1) {
        if (PID_Refresh_Flag) {
            PID_Refresh_Flag = 0u; /* 开始处理就清除，不在处理结束时清除 */
            if (!Balance_ReadPitch(&pitch) ||
                TLE5012B_ReadAngularSpeed(0u, &left_dps) != TLE_OK ||
                TLE5012B_ReadAngularSpeed(1u, &right_dps) != TLE_OK) goto stop;

            /* 编码器deg/s转机械rpm，以各自电角度增加方向为正；直驱1:1。 */
            left_rpm = left_dps / 6.0f * SVPWM_GetEncoderSign(0u);
            right_rpm = right_dps / 6.0f * SVPWM_GetEncoderSign(1u);
            if (!Balance_Update(&balance_pid, pitch, left_rpm, right_rpm, command))
                goto stop;
            Balance_CommandToDrive(command, dir, modulation);
        }

        /* 放在if外：PID未刷新时，也用上次幅值持续更新电机电角度。 */
        if (!SVPWM_Update(dir[0], dir[1], modulation[0], modulation[1])) goto stop;
    }

stop:
    Balance_TIM3_Stop();
    Balance_Stop(&balance_pid);
    SVPWM_Stop(0u);
    SVPWM_Stop(1u);
    while (1) { } /* 故障后停止，不自动重启 */
}