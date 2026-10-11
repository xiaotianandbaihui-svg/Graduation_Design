/* 主机上的逻辑测试。实际SPI波形、门极波形和电机电流仍须实测。 */
#include "motor.h"
#include "motor_port.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint32_t ticks, feed, fault;
static uint8_t output_mask, killed;
static uint16_t raw[2] = {1234u, 5678u};
static uint8_t vector_out[2];
static uint16_t duty_out[2];
static uint32_t applications;
static int crc_bad_id = -1;
static int bus_bad_id = -1;
static int calibration_motion;
static int calibration_direction = 1;
static unsigned phase_steps;
static uint32_t read_us = 12u;
static uint16_t status_bits = 0xfe00u;

static void advance(uint32_t us)
{
    ticks += us * MOTOR_CPU_TICKS_PER_US;
    if (output_mask && (uint32_t)(ticks - feed) > MOTOR_TIMEOUT_US * MOTOR_CPU_TICKS_PER_US)
        MotorPort_Kill(PORT_FAULT_TIMEOUT);
}

int MotorPort_Init(void)
{
    ticks = feed = fault = applications = 0u;
    killed = 1u;
    output_mask = 0u;
    vector_out[0] = vector_out[1] = 255u;
    duty_out[0] = duty_out[1] = 0u;
    return 1;
}
int MotorPort_Enable(void) { killed = 0u; return 1; }
uint32_t MotorPort_Ticks(void) { return ticks; }
void MotorPort_DelayUs(uint32_t us) { advance(us); }
void MotorPort_Feed(void) { feed = ticks; }
uint32_t MotorPort_Fault(void) { return fault; }
void MotorPort_Stop(uint8_t id) { output_mask &= (uint8_t)~(1u << id); duty_out[id] = 0u; }
void MotorPort_Kill(uint32_t why)
{
    fault |= why;
    killed = 1u;
    output_mask = 0u;
    duty_out[0] = duty_out[1] = 0u;
}

void MotorPort_Apply(uint8_t id, uint8_t vector, uint16_t duty)
{
    int32_t position;
    if (killed) return;
    assert(id < 2u && vector < 6u && duty <= MOTOR_MAX_DUTY);
    if (calibration_motion) {
        if (vector_out[id] == 255u) phase_steps = 0u;
        else if (vector_out[id] != vector) {
            assert(vector == (vector_out[id] + 1u) % 6u);
            ++phase_steps;
        }
        position = 31000 + calibration_direction * (int32_t)(phase_steps * 32768u / 42u);
        raw[id] = (uint16_t)((uint32_t)position & 0x7fffu);
    }
    vector_out[id] = vector;
    duty_out[id] = duty;
    output_mask |= (uint8_t)(1u << id);
    ++applications;
}

int MotorPort_SSCRead(uint8_t id, uint16_t cmd, uint16_t *data, uint16_t *safety)
{
    uint8_t bytes[4];
    assert(cmd == 0x8001u || cmd == 0x8021u);
    advance(read_us);
    if ((int)id == bus_bad_id) return 0;
    *data = cmd == 0x8001u ? 0u : (uint16_t)(raw[id] | 0x8000u);
    bytes[0] = (uint8_t)(cmd >> 8);
    bytes[1] = (uint8_t)cmd;
    bytes[2] = (uint8_t)(*data >> 8);
    bytes[3] = (uint8_t)*data;
    *safety = status_bits | TLE5012B_CRC8(bytes, 4u);
    if ((int)id == crc_bad_id) *safety ^= 1u;
    return 1;
}

static uint16_t safety_for(uint16_t command, uint16_t data)
{
    uint8_t bytes[4] = {(uint8_t)(command >> 8), (uint8_t)command,
                         (uint8_t)(data >> 8), (uint8_t)data};
    return (uint16_t)(0xfe00u | TLE5012B_CRC8(bytes, 4u));
}

static void test_crc_and_status(void)
{
    uint16_t safety = safety_for(0x8021u, 0x9234u);
    /* 公认的CRC-8/SAE-J1850 check值，独立于构造的模拟帧。 */
    assert(TLE5012B_CRC8((const uint8_t *)"123456789", 9u) == 0x4bu);
    assert(TLE5012B_CheckFrame(0x8021u, 0x9234u, safety, 1) == TLE_OK);
    assert(TLE5012B_CheckFrame(0x8021u, 0x9235u, safety, 1) == TLE_CRC_ERROR);
    assert(TLE5012B_CheckFrame(0x8021u, 0x9234u, safety ^ 0x8000u, 1) == TLE_RESET_ERROR);
    assert(TLE5012B_CheckFrame(0x8021u, 0x9234u, safety ^ 0x4000u, 1) == TLE_SYSTEM_ERROR);
    assert(TLE5012B_CheckFrame(0x8021u, 0x9234u, safety ^ 0x2000u, 1) == TLE_INTERFACE_ERROR);
    assert(TLE5012B_CheckFrame(0x8021u, 0x9234u, safety ^ 0x1000u, 1) == TLE_ANGLE_ERROR);
    assert(TLE5012B_CheckFrame(0x8021u, 0x9234u, safety | 0x0100u, 1) == TLE_RESPONSE_ERROR);
    assert(TLE5012B_CheckFrame(0x8021u, 0x9234u, safety ^ 0x8000u, 0) == TLE_OK);
}

static void test_angles(void)
{
    MotorCalibration cal = {32000u, 1, 7u};
    uint32_t i, sign_index, transitions, previous;
    uint8_t positive, negative;
    int32_t electrical, difference;
    for (sign_index = 0u; sign_index < 2u; ++sign_index) {
        cal.encoder_sign = sign_index == 0u ? 1 : -1;
        previous = Motor_AngleToVector(32767u, &cal, 1);
        transitions = 0u;
        for (i = 0u; i < 32768u; ++i) {
            positive = Motor_AngleToVector((uint16_t)i, &cal, 1);
            negative = Motor_AngleToVector((uint16_t)i, &cal, -1);
            assert(positive < 6u && negative < 6u);
            assert((positive + 3u) % 6u == negative);
            electrical = (((int32_t)i - cal.zero_raw) * cal.encoder_sign * 7) % 32768;
            if (electrical < 0) electrical += 32768;
            /* 以1/6编码器计数为单位：正转矢量领先60..120电角度。 */
            difference = ((int32_t)positive * 32768 - electrical * 6) % (32768 * 6);
            if (difference < 0) difference += 32768 * 6;
            assert(difference >= 32768 && difference <= 65536);
            if (positive != previous) ++transitions;
            previous = positive;
        }
        assert(transitions == 42u); /* 每机械圈7电周期、每电周期6扇区 */
    }
}

static void update_once(void) { advance(100u); Motor_Update(); }

static void test_run_and_faults(void)
{
    MotorCalibration cal = {1234u, 1, 7u};
    uint16_t angle;
    float degrees;
    uint32_t old_applications;
    assert(Motor_Init(3u) == MOTOR_OK);
    assert(output_mask == 0u && applications == 0u);
    assert(Motor_Run(0u, 50) == MOTOR_NOT_CALIBRATED);
    assert(Motor_SetCalibration(0u, &cal) == MOTOR_OK);
    assert(Motor_SetCalibration(1u, &cal) == MOTOR_OK);
    assert(Motor_Run(0u, 50) == MOTOR_OK);
    assert(Motor_Run(1u, -60) == MOTOR_OK);
    update_once();
    assert(output_mask == 3u && duty_out[0] == 50u && duty_out[1] == 60u);
    assert(Motor_GetAngle(0u, &angle, &degrees) == MOTOR_OK && angle == 1234u);
    assert(degrees > 13.5f && degrees < 13.6f);
    Motor_Stop(0u);
    assert(output_mask == 2u);
    crc_bad_id = 1;
    update_once();
    assert(output_mask == 0u && (Motor_GetFault() & MOTOR_FAULT_ENCODER2));
    old_applications = applications;
    crc_bad_id = -1;
    update_once();
    assert(applications == old_applications); /* 通信恢复也不能自动重启 */
    assert(Motor_Run(0u, 50) == MOTOR_FAULTED);

    assert(Motor_Init(1u) == MOTOR_OK);
    assert(Motor_Run(0u, 50) == MOTOR_NOT_CALIBRATED); /* 重启后需加载标定 */
    assert(Motor_SetCalibration(0u, &cal) == MOTOR_OK);
    assert(Motor_Run(0u, 50) == MOTOR_OK);
    update_once();
    read_us = 101u;
    update_once();
    assert(output_mask == 0u && (Motor_GetFault() & MOTOR_FAULT_TIMEOUT));
    read_us = 12u;

    assert(Motor_Init(1u) == MOTOR_OK);
    assert(Motor_SetCalibration(0u, &cal) == MOTOR_OK);
    assert(Motor_Run(0u, 50) == MOTOR_OK);
    update_once();
    advance(MOTOR_TIMEOUT_US + 1u);
    assert(output_mask == 0u && (Motor_GetFault() & MOTOR_FAULT_TIMEOUT));

    assert(Motor_Init(1u) == MOTOR_OK);
    assert(Motor_SetCalibration(0u, &cal) == MOTOR_OK);
    assert(Motor_Run(0u, 50) == MOTOR_OK);
    update_once();
    MotorPort_Kill(PORT_FAULT_DRV1);
    update_once();
    assert(output_mask == 0u && Motor_Run(0u, 50) == MOTOR_FAULTED);

    assert(Motor_Init(1u) == MOTOR_OK);
    assert(Motor_SetCalibration(0u, &cal) == MOTOR_OK);
    assert(Motor_Run(0u, MOTOR_MAX_DUTY + 1) == MOTOR_BAD_ARGUMENT);
    bus_bad_id = 0;
    update_once();
    assert(Motor_GetEncoderError(0u) == TLE_BUS_ERROR && output_mask == 0u);
    bus_bad_id = -1;

    status_bits = 0x7e00u; /* 持续复位：初次读STAT允许，但下一帧必须报错 */
    assert(Motor_Init(1u) == MOTOR_FAULTED);
    status_bits = 0xfe00u;
    assert(Motor_Init(1u) == MOTOR_OK);
    ticks = 0xfffff000u;
    Motor_Update();
    ticks += 100u * MOTOR_CPU_TICKS_PER_US; /* DWT回绕仍能正常读取 */
    Motor_Update();
    assert(Motor_GetFault() == 0u);
}

static void test_calibration(void)
{
    MotorCalibration cal;
    int sign;
    for (sign = -1; sign <= 1; sign += 2) {
        assert(Motor_Init(1u) == MOTOR_OK);
        calibration_motion = 1;
        calibration_direction = sign;
        assert(Motor_Calibrate(0u, 50u, &cal) == MOTOR_OK);
        assert(cal.encoder_sign == sign && cal.pole_pairs == 7u);
        assert(output_mask == 0u);
        assert(cal.zero_raw == raw[0]);
        calibration_motion = 0;
        assert(Motor_Run(0u, 50) == MOTOR_OK);
        update_once();
        assert(output_mask == 1u);
        Motor_Stop(0u);
    }
    assert(Motor_Init(1u) == MOTOR_OK);
    /* 转子不动：不能误认为标定成功。 */
    assert(Motor_Calibrate(0u, 50u, &cal) == MOTOR_CALIBRATION_FAILED);
    assert(output_mask == 0u && (Motor_GetFault() & MOTOR_FAULT_CALIBRATION));
}

int main(void)
{
    test_crc_and_status();
    test_angles();
    test_run_and_faults();
    test_calibration();
    puts("PASS: CRC/status; 65536 signed-angle cases; 42 sectors/rev; dual-motor stop/faults; timing/wrap; calibration +/- and stalled rotor.");
    return 0;
}
