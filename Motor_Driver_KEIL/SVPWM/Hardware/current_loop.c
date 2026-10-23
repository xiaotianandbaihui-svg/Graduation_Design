#include "current_loop.h"
#include <math.h>

#define DEG_TO_RAD 0.017453292519943295f
#define INV_SQRT3  0.577350269189625765f

static float Clamp(float value, float limit)
{
    return value > limit ? limit : (value < -limit ? -limit : value);
}

static void LimitVector(float *x, float *y, float limit)
{
    float magnitude = hypotf(*x, *y);
    if (magnitude > limit) {
        float scale = limit / magnitude;
        *x *= scale;
        *y *= scale;
    }
}

static void ClearState(CurrentLoop *loop)
{
    loop->integral_d = loop->integral_q = 0.0f;
    loop->id_a = loop->iq_a = 0.0f;
    loop->ud_v = loop->uq_v = 0.0f;
}

void CurrentLoop_Init(CurrentLoop *loop, uint8_t motor_id,
                      const CurrentLoop_Config *config)
{
    loop->config = *config;
    loop->motor_id = motor_id;
    loop->running = 0;
    ClearState(loop);
}

int CurrentLoop_Start(CurrentLoop *loop)
{
    const CurrentLoop_Config *c = &loop->config;
    /* 静态参数只在启动时检查，不在每次电流环中重复扫描。 */
    if (!isfinite(c->kp_d) || c->kp_d < 0.0f ||
        !isfinite(c->ki_d) || c->ki_d < 0.0f ||
        !isfinite(c->kp_q) || c->kp_q < 0.0f ||
        !isfinite(c->ki_q) || c->ki_q < 0.0f ||
        !isfinite(c->period_s) || c->period_s <= 0.0f ||
        !isfinite(c->current_limit_a) || c->current_limit_a <= 0.0f ||
        !(c->voltage_ratio > 0.0f && c->voltage_ratio <= 1.0f)) {
        CurrentLoop_Stop(loop);
        return 0;
    }
    ClearState(loop);
    loop->running = 1;
    return 1;
}

void CurrentLoop_Stop(CurrentLoop *loop)
{
    loop->running = 0;
    ClearState(loop);
    CurrentLoop_DisablePWM(loop->motor_id);
}

int CurrentLoop_Update(CurrentLoop *loop, float target_id_a, float target_iq_a)
{
    const CurrentLoop_Config *c = &loop->config;
    CurrentLoop_Feedback feedback;
    float sine, cosine, alpha, beta, ed, eq, voltage_limit;
    float next_d, next_q, raw_d, raw_q, ud, uq;

    if (!loop->running) return 0;
    if (!CurrentLoop_ReadFeedback(loop->motor_id, &feedback) ||
        !isfinite(feedback.ia_a) || !isfinite(feedback.ib_a) ||
        !isfinite(feedback.electrical_deg) ||
        !isfinite(feedback.bus_voltage_v) || feedback.bus_voltage_v <= 0.0f ||
        !isfinite(target_id_a) || !isfinite(target_iq_a)) goto failed;

    LimitVector(&target_id_a, &target_iq_a, c->current_limit_a);
    sine = sinf(feedback.electrical_deg * DEG_TO_RAD);
    cosine = cosf(feedback.electrical_deg * DEG_TO_RAD);

    /* Clarke变换，幅值不变型；假设三线电机ia+ib+ic=0。 */
    alpha = feedback.ia_a;
    beta = (feedback.ia_a + 2.0f * feedback.ib_a) * INV_SQRT3;
    /* Park变换：d轴对准转子磁链，q轴正向超前d轴90度。 */
    loop->id_a = alpha * cosine + beta * sine;
    loop->iq_a = -alpha * sine + beta * cosine;
    ed = target_id_a - loop->id_a;
    eq = target_iq_a - loop->iq_a;

    voltage_limit = feedback.bus_voltage_v * INV_SQRT3 * c->voltage_ratio;
    next_d = Clamp(loop->integral_d + c->ki_d * c->period_s * ed, voltage_limit);
    next_q = Clamp(loop->integral_q + c->ki_q * c->period_s * eq, voltage_limit);
    raw_d = c->kp_d * ed + next_d;
    raw_q = c->kp_q * eq + next_q;
    if (!isfinite(raw_d) || !isfinite(raw_q)) goto failed;

    ud = raw_d;
    uq = raw_q;
    LimitVector(&ud, &uq, voltage_limit); /* 限合成矢量，避免两轴分别限幅后超调制 */

    /* 逆Park变换，输出静止坐标系的电压矢量。 */
    alpha = ud * cosine - uq * sine;
    beta = ud * sine + uq * cosine;
    if (!CurrentLoop_WriteVoltage(loop->motor_id, alpha, beta)) goto failed;

    /* 条件积分：拒绝继续推向饱和的积分变化，允许反向释放。
     * 本拍电压使用候选PI结果，是否保留积分影响下一拍。 */
    if ((raw_d - ud) * (next_d - loop->integral_d) <= 0.0f)
        loop->integral_d = next_d;
    if ((raw_q - uq) * (next_q - loop->integral_q) <= 0.0f)
        loop->integral_q = next_q;
    loop->ud_v = ud;
    loop->uq_v = uq;
    return 1;

failed:
    CurrentLoop_Stop(loop);
    return 0;
}

#ifndef CURRENT_LOOP_EXTERNAL_IO
/* 尚未接通完整的同步反馈/电压输出链路，以下空接口仍返回失败。 */
int CurrentLoop_ReadFeedback(uint8_t motor_id, CurrentLoop_Feedback *feedback)
{
    (void)motor_id;
    (void)feedback;
    /* TODO：取得本周期真实的ia/ib、电角度和母线电压，填写feedback，成功返回1。 */
    return 0;
}

int CurrentLoop_WriteVoltage(uint8_t motor_id, float alpha_v, float beta_v)
{
    (void)motor_id;
    (void)alpha_v;
    (void)beta_v;
    /* TODO：将alpha/beta电压矢量转为三相PWM，成功返回1。 */
    return 0;
}

void CurrentLoop_DisablePWM(uint8_t motor_id)
{
    (void)motor_id;
    /* TODO：关闭指定电机的驱动输出。 */
}
#endif
