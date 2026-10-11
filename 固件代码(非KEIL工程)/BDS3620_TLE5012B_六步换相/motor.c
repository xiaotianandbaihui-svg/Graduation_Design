#include "motor.h"
#include "motor_port.h"
#include <string.h>

typedef struct {
    MotorCalibration cal;
    uint16_t raw;
    int16_t duty;
    uint8_t calibrated;
    uint8_t angle_valid;
    TLE_Result encoder_error;
} MotorState;

static MotorState motor[2];
static uint8_t active_mask;
static uint8_t initialized;
static uint32_t software_fault;
static uint32_t last_update;

/* 可在调试器Watch窗口查看；实际双编码器最坏执行时间必须小于100us。 */
volatile uint32_t motor_last_update_us;

static int active(uint8_t id)
{
    return initialized && id < 2u && (active_mask & (1u << id));
}

static int32_t wrap_delta(uint16_t now, uint16_t before)
{
    int32_t delta = (int32_t)now - (int32_t)before;
    if (delta >= 16384) delta -= 32768;
    if (delta < -16384) delta += 32768;
    return delta;
}

static uint16_t wrap_count(int32_t value)
{
    value %= 32768;
    if (value < 0) value += 32768;
    return (uint16_t)value;
}

static void latch_fault(uint32_t fault)
{
    software_fault |= fault;
    motor[0].duty = 0;
    motor[1].duty = 0;
    MotorPort_Kill(0u);
}

uint32_t Motor_GetFault(void)
{
    return software_fault | MotorPort_Fault();
}

static int read_encoder(uint8_t id)
{
    uint16_t raw;
    motor[id].encoder_error = TLE5012B_ReadRaw(id, &raw);
    if (motor[id].encoder_error != TLE_OK) {
        motor[id].angle_valid = 0u;
        latch_fault(id == 0u ? MOTOR_FAULT_ENCODER1 : MOTOR_FAULT_ENCODER2);
        return 0;
    }
    motor[id].raw = raw;
    motor[id].angle_valid = 1u;
    return 1;
}

MotorResult Motor_Init(uint8_t mask)
{
    uint8_t id;
    /* 重初始化首先停机，也会清除RAM中的标定；调用者需重新载入标定。 */
    MotorPort_Kill(0u);
    memset(motor, 0, sizeof(motor));
    initialized = 0u;
    active_mask = 0u;
    software_fault = 0u;
    motor_last_update_us = 0u;
    if (mask == 0u || mask > 3u) return MOTOR_BAD_ARGUMENT;
    if (!MotorPort_Init()) {
        latch_fault(MOTOR_FAULT_INIT);
        return MOTOR_NOT_READY;
    }
    active_mask = mask;
    for (id = 0u; id < 2u; ++id) {
        if (!(mask & (1u << id))) continue;
        motor[id].encoder_error = TLE5012B_Init(id);
        if (motor[id].encoder_error != TLE_OK) {
            latch_fault(id == 0u ? MOTOR_FAULT_ENCODER1 : MOTOR_FAULT_ENCODER2);
            return MOTOR_FAULTED;
        }
        if (!read_encoder(id)) return MOTOR_FAULTED;
    }
    if (!MotorPort_Enable()) return MOTOR_FAULTED;
    initialized = 1u;
    last_update = MotorPort_Ticks();
    return MOTOR_OK;
}

uint8_t Motor_AngleToVector(uint16_t raw, const MotorCalibration *cal, int direction)
{
    uint16_t electrical;
    uint32_t target;
    int32_t mechanical;
    if (cal == 0 || cal->pole_pairs == 0u) return 0u;
    mechanical = ((int32_t)(raw & 0x7fffu) - (int32_t)cal->zero_raw) * cal->encoder_sign;
    electrical = wrap_count(mechanical * cal->pole_pairs);
    /* 对齐时矢量0与转子同向。转动时电压矢量相对转子取+90或-90电角度，
     * 再取最近的60度矢量。不能简单把对齐零位后的角度直接除60度！
     * 6*(theta +/-90deg) +30deg*6；加30deg用于四舍五入。
     * 全程整数，避免近似的60度计数造成扇区边界累计误差。 */
    target = ((uint32_t)electrical + (direction >= 0 ? 8192u : 24576u)) & 0x7fffu;
    return (uint8_t)(((target * 6u + 16384u) / 32768u) % 6u);
}

MotorResult Motor_SetCalibration(uint8_t id, const MotorCalibration *calibration)
{
    if (!active(id)) return MOTOR_NOT_READY;
    if (Motor_GetFault()) return MOTOR_FAULTED;
    if (motor[id].duty != 0) return MOTOR_BAD_ARGUMENT;
    if (calibration == 0 || calibration->zero_raw >= 32768u ||
        calibration->pole_pairs != MOTOR_POLE_PAIRS ||
        (calibration->encoder_sign != 1 && calibration->encoder_sign != -1))
        return MOTOR_BAD_ARGUMENT;
    motor[id].cal = *calibration;
    motor[id].calibrated = 1u;
    return MOTOR_OK;
}

MotorResult Motor_Run(uint8_t id, int16_t duty)
{
    if (!active(id)) return MOTOR_NOT_READY;
    if (duty == 0) { Motor_Stop(id); return MOTOR_OK; }
    if (Motor_GetFault()) return MOTOR_FAULTED;
    if (duty > MOTOR_MAX_DUTY || duty < -MOTOR_MAX_DUTY) {
        Motor_Stop(id);
        return MOTOR_BAD_ARGUMENT;
    }
    if (!motor[id].calibrated) return MOTOR_NOT_CALIBRATED;
    motor[id].duty = duty;
    return MOTOR_OK;                 /* 下一次Update读到有效角度后才输出 */
}

void Motor_Stop(uint8_t id)
{
    if (id > 1u) return;
    motor[id].duty = 0;
    MotorPort_Stop(id);
}

void Motor_EmergencyStop(void)
{
    motor[0].duty = 0;
    motor[1].duty = 0;
    MotorPort_Kill(PORT_FAULT_USER);
}

void Motor_Update(void)
{
    uint8_t id;
    uint32_t start;
    uint32_t elapsed;
    int running;
    if (!initialized || Motor_GetFault()) return;
    start = MotorPort_Ticks();
    if ((uint32_t)(start - last_update) < MOTOR_UPDATE_US * MOTOR_CPU_TICKS_PER_US) return;
    last_update = start;
    /* 两路都校验成功后才更新输出，不让另一只坏编码器被忽略。 */
    for (id = 0u; id < 2u; ++id) {
        if ((active_mask & (1u << id)) && !read_encoder(id)) return;
    }
    running = motor[0].duty != 0 || motor[1].duty != 0;
    elapsed = MotorPort_Ticks() - start;
    if (running && elapsed >= MOTOR_UPDATE_US * MOTOR_CPU_TICKS_PER_US) {
        MotorPort_Kill(PORT_FAULT_TIMEOUT);
        return;
    }
    MotorPort_Feed();
    for (id = 0u; id < 2u; ++id) {
        if (!(active_mask & (1u << id)) || motor[id].duty == 0) continue;
        MotorPort_Apply(id, Motor_AngleToVector(motor[id].raw, &motor[id].cal,
                         motor[id].duty > 0 ? 1 : -1),
                         (uint16_t)(motor[id].duty > 0 ? motor[id].duty : -motor[id].duty));
    }
    elapsed = MotorPort_Ticks() - start;
    motor_last_update_us = elapsed / MOTOR_CPU_TICKS_PER_US;
    if (running && elapsed >= MOTOR_UPDATE_US * MOTOR_CPU_TICKS_PER_US)
        MotorPort_Kill(PORT_FAULT_TIMEOUT);
}

TLE_Result Motor_GetEncoderError(uint8_t id)
{
    return id < 2u ? motor[id].encoder_error : TLE_BAD_ARGUMENT;
}

MotorResult Motor_GetAngle(uint8_t id, uint16_t *raw, float *degrees)
{
    if (!active(id) || !motor[id].angle_valid) return MOTOR_NOT_READY;
    if (Motor_GetFault()) return MOTOR_FAULTED;
    if (raw != 0) *raw = motor[id].raw;
    if (degrees != 0) *degrees = (float)motor[id].raw * (360.0f / 32768.0f);
    return MOTOR_OK;
}

/* 保持一个定子矢量；最后20ms取圆周平均，并检查转子是否已基本静止。
 * 静态对齐期间不根据角度换相，但仍持续检查编码器、nFAULT和更新超时。 */
static int align_vector(uint8_t id, uint8_t vector, uint16_t duty,
                        uint32_t duration_ms, uint16_t *angle)
{
    uint32_t start, elapsed;
    uint32_t duration = duration_ms * 1000u * MOTOR_CPU_TICKS_PER_US;
    uint32_t sample_start = (duration_ms - 20u) * 1000u * MOTOR_CPU_TICKS_PER_US;
    uint32_t samples = 0u;
    uint16_t reference = 0u;
    int32_t sum = 0, delta, min_delta = 0, max_delta = 0;
    if (Motor_GetFault()) return 0;
    if (!read_encoder(id)) return 0;
    MotorPort_Feed();
    MotorPort_Apply(id, vector, duty);
    start = MotorPort_Ticks();
    while ((elapsed = (uint32_t)(MotorPort_Ticks() - start)) < duration) {
        if (Motor_GetFault() || !read_encoder(id)) return 0;
        MotorPort_Feed();
        if (elapsed >= sample_start) {
            if (samples == 0u) reference = motor[id].raw;
            delta = wrap_delta(motor[id].raw, reference);
            if (delta < min_delta) min_delta = delta;
            if (delta > max_delta) max_delta = delta;
            sum += delta;
            ++samples;
        }
        MotorPort_DelayUs(100u);
    }
    /* 最后20ms内角度变化不超过10电角度，避免仍在振荡时记为零位。 */
    if (samples == 0u || max_delta - min_delta > (int32_t)(32768u / (MOTOR_POLE_PAIRS * 36u)))
        return 0;
    *angle = wrap_count((int32_t)reference + sum / (int32_t)samples);
    return 1;
}

MotorResult Motor_Calibrate(uint8_t id, uint16_t duty, MotorCalibration *result)
{
    uint16_t before, now = 0u;
    uint8_t step;
    int8_t sign = 0;
    int32_t delta, magnitude;
    const int32_t expected = 32768 / (MOTOR_POLE_PAIRS * 6);
    MotorCalibration cal;
    if (!active(id)) return MOTOR_NOT_READY;
    if (Motor_GetFault()) return MOTOR_FAULTED;
    if (result == 0 || duty == 0u || duty > MOTOR_MAX_ALIGN_DUTY ||
        motor[0].duty != 0 || motor[1].duty != 0) return MOTOR_BAD_ARGUMENT;
    motor[id].calibrated = 0u;
    if (!align_vector(id, 0u, duty, 600u, &before)) goto failed;
    /* 走完整一个电周期：0->1->2->3->4->5->0，自动识别编码器方向。
     * 每一步应机械转动约360/(7*6)=8.57度，错误极对数/堵转不接受。 */
    for (step = 1u; step <= 6u; ++step) {
        if (!align_vector(id, (uint8_t)(step % 6u), duty, 300u, &now)) goto failed;
        delta = wrap_delta(now, before);
        if (step == 1u) sign = delta >= 0 ? 1 : -1;
        magnitude = delta * sign;
        if (magnitude < expected / 2 || magnitude > expected * 3 / 2) goto failed;
        before = now;
    }
    MotorPort_Stop(id);
    if (Motor_GetFault()) goto failed;
    cal.zero_raw = now;
    cal.encoder_sign = sign;
    cal.pole_pairs = (uint8_t)MOTOR_POLE_PAIRS;
    motor[id].cal = cal;
    motor[id].calibrated = 1u;
    *result = cal;
    last_update = MotorPort_Ticks();
    return MOTOR_OK;
failed:
    MotorPort_Stop(id);
    latch_fault(MOTOR_FAULT_CALIBRATION);
    return MOTOR_CALIBRATION_FAILED;
}
