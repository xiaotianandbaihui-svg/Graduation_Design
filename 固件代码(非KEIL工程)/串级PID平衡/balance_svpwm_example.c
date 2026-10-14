#include "balance_pid.h"
#include "svpwm.h"
#include "tle5012b.h"

/* 电机0=左轮、1=右轮；编码器与轮子1:1，有减速器时另加传动比。 */
void Balance_SVPWM_Stop(BalanceSVPWM *s)
{
    if (s) {
        Balance_Stop(&s->controller);
        s->have_command = 0u;
        s->motor_command[0] = s->motor_command[1] = 0;
        s->command_time_ms = 0u;
    }
    SVPWM_Stop(0u);
    SVPWM_Stop(1u);
}

int Balance_SVPWM_Control(BalanceSVPWM *s, float tilt_deg,
                         uint32_t now_ms)
{
    float left_dps, right_dps;
    int8_t sign[2];
    if (!s || !s->controller.running) goto fail;
    sign[0] = SVPWM_GetEncoderSign(0u);
    sign[1] = SVPWM_GetEncoderSign(1u);
    if ((sign[0] != 1 && sign[0] != -1) ||
        (sign[1] != 1 && sign[1] != -1)) goto fail;
    if (TLE5012B_ReadAngularSpeed(0u, &left_dps) != TLE_OK ||
        TLE5012B_ReadAngularSpeed(1u, &right_dps) != TLE_OK) goto fail;
    /* deg/s除6为机械rpm，以各自电角度增加方向为正。 */
    left_dps *= (float)sign[0] / 6.0f;
    right_dps *= (float)sign[1] / 6.0f;
    if (!Balance_Update(&s->controller, tilt_deg,
                        left_dps, right_dps, s->motor_command)) goto fail;
    s->command_time_ms = now_ms;
    s->have_command = 1u;
    return 1;
fail:
    Balance_SVPWM_Stop(s);
    return 0;
}

int Balance_SVPWM_MotorUpdate(BalanceSVPWM *s, uint32_t now_ms)
{
    int16_t left, right;
    if (!s) goto fail;
    if (!s->controller.running || !s->have_command) {
        s->have_command = 0u;
        s->motor_command[0] = s->motor_command[1] = 0;
        SVPWM_Stop(0u);
        SVPWM_Stop(1u);
        return 1; /* 停止状态或尚无新命令，保持关闭。 */
    }
    if ((uint32_t)(now_ms - s->command_time_ms) >
        BALANCE_COMMAND_TIMEOUT_MS) goto fail;
    left = s->motor_command[0];
    right = s->motor_command[1];
    if (!SVPWM_Update((int8_t)(left < 0 ? -1 : 1),
                      (int8_t)(right < 0 ? -1 : 1),
                      (uint16_t)(left < 0 ? -left : left),
                      (uint16_t)(right < 0 ? -right : right))) goto fail;
    return 1;
fail:
    Balance_SVPWM_Stop(s);
    return 0;
}
