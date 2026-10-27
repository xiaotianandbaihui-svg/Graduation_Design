#include "current_loop.h"
#include <math.h>

#define DEG_TO_RAD 0.017453292519943295f
#define INV_SQRT3  0.577350269189625765f

static float Clamp(float value, float limit) /* 取得-limit≤value≤limit */
{
    return value > limit ? limit : (value < -limit ? -limit : value);
}

static int PI_Init(CurrentLoop_PI *pi, float kp, float ki, float period_s)
{
    if (!isfinite(kp) || kp < 0.0f || !isfinite(ki) || ki < 0.0f) return 0;
    pi->kp = kp;
    pi->ki_step = ki * period_s;
    return isfinite(pi->ki_step);
}

/* 先试算本拍积分；合成电压限幅后，再决定是否保留。 */
static float PI_Output(CurrentLoop_PI *pi, float error, float limit)
{
    float next = pi->integral + pi->ki_step * error;
    if (!isfinite(next)) return NAN;
    pi->integral = Clamp(next, limit);
    return pi->kp * error + pi->integral;
}

static void PI_AntiWindup(CurrentLoop_PI *pi, float previous, float raw, float limited)
{
    /* 本拍积分若继续推动输出饱和，就退回上拍积分；反向释放仍允许。 */
    if ((raw - limited) * (pi->integral - previous) > 0.0f)
        pi->integral = previous;
}

void CurrentLoop_Reset(CurrentLoop *loop)
{
    loop->d.integral = loop->q.integral = 0.0f;
    loop->id_a = loop->iq_a = 0.0f;
    loop->ud_v = loop->uq_v = 0.0f;
}

int CurrentLoop_Init(CurrentLoop *loop, const CurrentLoop_Config *config)
{
    CurrentLoop_Reset(loop);
    if (!isfinite(config->period_s) || config->period_s <= 0.0f ||
        !isfinite(config->current_limit_a) || config->current_limit_a <= 0.0f ||
        !(config->voltage_ratio > 0.0f && config->voltage_ratio <= 1.0f) ||
        !PI_Init(&loop->d, config->kp_d, config->ki_d, config->period_s) ||
        !PI_Init(&loop->q, config->kp_q, config->ki_q, config->period_s)) return 0;
    loop->current_limit_a = config->current_limit_a;
    loop->voltage_ratio = config->voltage_ratio;
    return 1;
}

int CurrentLoop_Update(CurrentLoop *loop, const CurrentLoop_Feedback *feedback,
                       float target_iq_a, CurrentLoop_Voltage *voltage)
{
    float sine, cosine, alpha, beta, ed, eq, limit, magnitude, scale;
    float previous_d, previous_q, raw_d, raw_q, ud, uq;
    if (!isfinite(feedback->ia_a) || !isfinite(feedback->ib_a) ||
        !isfinite(feedback->electrical_deg) || !isfinite(feedback->bus_voltage_v) ||
        feedback->bus_voltage_v <= 0.0f || !isfinite(target_iq_a)) goto failed;

    target_iq_a = Clamp(target_iq_a, loop->current_limit_a);
    sine = sinf(feedback->electrical_deg * DEG_TO_RAD);
    cosine = cosf(feedback->electrical_deg * DEG_TO_RAD);

    /* Clarke + Park：把相电流换算到随转子旋转的d/q轴。 */
    alpha = feedback->ia_a;
    beta = (feedback->ia_a + 2.0f * feedback->ib_a) * INV_SQRT3;
    loop->id_a = alpha * cosine + beta * sine;
    loop->iq_a = -alpha * sine + beta * cosine;


    /* 电流星点向外为正、电压相端对星点为正，因此误差取实测-目标。 */
    /*这里算的其实就是电流误差了*/
    ed = loop->id_a; /* Id目标为0，但Ud不能固定为0 */
    eq = loop->iq_a - target_iq_a;


    /* 电压矢量幅值不超过母线电压的1/sqrt(3)，并保留配置的余量，这个裕量就来自电压比例，电压比例的分母是1000 */
    limit = feedback->bus_voltage_v * INV_SQRT3 * loop->voltage_ratio;
    previous_d = loop->d.integral;
    previous_q = loop->q.integral;

    /*这是首次限幅，算是一个初次限幅，免得数量过大，计算后溢出了，数学上，其实是对最终合成的那个电压矢量进行限幅*/
    raw_d = PI_Output(&loop->d, ed, limit);
    raw_q = PI_Output(&loop->q, eq, limit);
    magnitude = hypotf(raw_d, raw_q);/*平方和开根号，也就是欧氏距离的那个公式*/
    if (!isfinite(magnitude)) goto failed;

    /* 限制合成矢量，不能只把Ud、Uq各自限幅。 */
    ud = raw_d; uq = raw_q;
    if (magnitude > limit) {
        scale = limit / magnitude;
        ud *= scale; uq *= scale;
    }/*其实就是超过了，magnitude就规范为limit，(这里是等比例缩小，主要是要保障合成电压矢量的方向不会变)*/

    /*此处会在总的输出合成矢量饱和后防止积分项继续增加，是抗饱和的，但也只是做了个积分抗饱和*/
    PI_AntiWindup(&loop->d, previous_d, raw_d, ud);
    PI_AntiWindup(&loop->q, previous_q, raw_q, uq);

    /* 逆Park：把d/q电压换回静止坐标系，交给SVPWM。*/
    voltage->alpha_v = ud * cosine - uq * sine;
    voltage->beta_v = ud * sine + uq * cosine;
    loop->ud_v = ud; loop->uq_v = uq;
    return 1;

failed:
    CurrentLoop_Reset(loop);
    voltage->alpha_v = voltage->beta_v = 0.0f;
    return 0;
}
