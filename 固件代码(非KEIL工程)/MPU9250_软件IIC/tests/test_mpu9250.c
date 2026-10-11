/* 以寄存器模型验证初始化、ID、读回校验、错误传播及字节序。 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "mpu9250.h"
#include "iic_port.h"

static uint8_t regs[128][256];
static unsigned int writes, reads, elapsed_us;
static int fail_next, corrupt_reg, last_addr, last_reg, last_len;

static void reset(void)
{
    memset(regs, 0, sizeof(regs));
    regs[0x68][0x75] = regs[0x69][0x75] = 0x71;
    regs[0x0C][0x00] = 0x48;
    regs[0x0C][0x10] = 128;
    regs[0x0C][0x11] = 160;
    regs[0x0C][0x12] = 96;
    writes = reads = elapsed_us = 0;
    fail_next = 0; corrupt_reg = -1;
    last_addr = last_reg = last_len = -1;
}

IIC_Status IIC_Init(void) { return IIC_OK; }
void IIC_DelayUs(uint32_t us) { elapsed_us += us; }

IIC_Status IIC_WriteReg(uint8_t addr, uint8_t reg, uint8_t value)
{
    ++writes;
    if (fail_next) { fail_next = 0; return IIC_ERR_NACK; }
    regs[addr][reg] = (int)reg == corrupt_reg ? (uint8_t)(value ^ 1u) : value;
    return IIC_OK;
}

IIC_Status IIC_ReadRegs(uint8_t addr, uint8_t reg, uint8_t *data, size_t len)
{
    ++reads;
    last_addr = addr; last_reg = reg; last_len = (int)len;
    if (fail_next) { fail_next = 0; return IIC_ERR_SCL_TIMEOUT; }
    assert((size_t)reg + len <= 256u);
    memcpy(data, &regs[addr][reg], len);
    return IIC_OK;
}

int main(void)
{
    MPU9250 imu = {0};
    MPU9250_Raw raw, before;
    float mag[3] = {100.0f, 200.0f, 300.0f};
    static const uint8_t sample[14] = {
        0x40, 0x00, 0xC0, 0x00, 0x7F, 0xFF,
        0x80, 0x00,
        0x00, 0x01, 0xFF, 0xFF, 0x80, 0x00
    };
    static const uint8_t magnetic[7] = {100, 0, 0x9C, 0xFF, 0, 0x80, 0x10};

    reset();
    assert(MPU9250_Init(&imu, 0x68) == MPU9250_OK);
    assert(imu.initialized && !imu.mag_initialized);
    assert(regs[0x68][0x6B] == 1 && regs[0x68][0x6C] == 0);
    assert(regs[0x68][0x1B] == 0x18 && regs[0x68][0x1C] == 0);
    assert(regs[0x68][0x1A] == 3 && regs[0x68][0x1D] == 3);
    assert(regs[0x68][0x19] == 4 && regs[0x68][0x38] == 1);
    assert(elapsed_us >= 400000u);

    memset(&raw, 0x55, sizeof(raw)); before = raw;
    assert(MPU9250_ReadRaw(&imu, &raw) == MPU9250_NO_NEW_DATA);
    assert(memcmp(&raw, &before, sizeof(raw)) == 0);
    regs[0x68][0x3A] = 1;
    memcpy(&regs[0x68][0x3B], sample, sizeof(sample));
    assert(MPU9250_ReadRaw(&imu, &raw) == MPU9250_OK);
    assert(last_reg == 0x3B && last_len == 14);
    assert(raw.accel[0] == 16384 && raw.accel[1] == -16384);
    assert(raw.accel[2] == 32767 && raw.temperature == -32768);
    assert(raw.gyro[0] == 1 && raw.gyro[1] == -1 && raw.gyro[2] == -32768);
    before = raw; fail_next = 1;
    assert(MPU9250_ReadRaw(&imu, &raw) == MPU9250_ERR_IIC);
    assert(imu.last_iic_status == IIC_ERR_SCL_TIMEOUT);
    assert(memcmp(&raw, &before, sizeof(raw)) == 0);

    assert(MPU9250_InitMag(&imu) == MPU9250_OK && imu.mag_initialized);
    assert(regs[0x68][0x37] == 2 && regs[0x68][0x6A] == 0);
    assert(regs[0x0C][0x0A] == 0x16);
    assert(imu.mag_adjust[0] == 1.0f && imu.mag_adjust[1] == 1.125f);
    assert(imu.mag_adjust[2] == 0.875f);
    assert(MPU9250_ReadMag(&imu, mag) == MPU9250_NO_NEW_DATA);
    assert(mag[0] == 100.0f && mag[1] == 200.0f && mag[2] == 300.0f);
    regs[0x0C][2] = 1;
    memcpy(&regs[0x0C][3], magnetic, sizeof(magnetic));
    assert(MPU9250_ReadMag(&imu, mag) == MPU9250_OK);
    assert(last_addr == 0x0C && last_reg == 3 && last_len == 7); /* 包含ST2。 */
    assert(fabsf(mag[0] - 15.0f) < 0.001f);
    assert(fabsf(mag[1] + 16.875f) < 0.001f);
    assert(fabsf(mag[2] + 4300.8f) < 0.01f);
    regs[0x0C][9] |= 8;
    mag[0] = 99;
    assert(MPU9250_ReadMag(&imu, mag) == MPU9250_ERR_MAG_OVERFLOW);
    assert(mag[0] == 99);

    reset(); regs[0x68][0x75] = 0x70;
    assert(MPU9250_Init(&imu, 0x68) == MPU9250_ERR_ID);
    assert(!imu.initialized && !imu.mag_initialized && writes == 0);
    assert(MPU9250_ReadRaw(&imu, &raw) == MPU9250_ERR_NOT_INITIALIZED);
    reset(); corrupt_reg = 0x1C;
    /* bit0是保留位，校验应忽略；改用量程设置中的有效位测试。 */
    corrupt_reg = 0x19;
    assert(MPU9250_Init(&imu, 0x68) == MPU9250_ERR_VERIFY);
    assert(!imu.initialized);
    reset(); fail_next = 1;
    assert(MPU9250_Init(&imu, 0x68) == MPU9250_ERR_IIC);
    assert(!imu.initialized && imu.last_iic_status == IIC_ERR_SCL_TIMEOUT);
    reset();
    assert(MPU9250_Init(&imu, 0x69) == MPU9250_OK && imu.addr7 == 0x69);
    assert(MPU9250_Init(&imu, 0xD0) == MPU9250_ERR_ARGUMENT);
    assert(!imu.initialized);
    assert(MPU9250_Init(NULL, 0x68) == MPU9250_ERR_ARGUMENT);
    puts("mpu9250: init, readback, IDs, endian, signed limits and ST2 checks PASS");
    return 0;
}
