#include "mpu6500_spi.h"
#include "stm32f10x.h"
#include "Delay.h"
#include <math.h>
#include <stddef.h>

#define SMPLRT_DIV      0x19u
#define CONFIG          0x1Au
#define GYRO_CONFIG     0x1Bu
#define ACCEL_CONFIG    0x1Cu
#define ACCEL_CONFIG2   0x1Du
#define FIFO_EN         0x23u
#define INT_PIN_CFG     0x37u
#define INT_ENABLE      0x38u
#define INT_STATUS      0x3Au
#define USER_CTRL       0x6Au
#define PWR_MGMT_1      0x6Bu
#define PWR_MGMT_2      0x6Cu
#define WHO_AM_I        0x75u
#define CS_PIN          GPIO_Pin_12
#define WAIT_LIMIT      100000u /* 有限轮询次数，不是微秒数 */
#define RAD_PER_DEG     0.01745329252f

#if MPU6500_READ_MAX_HZ < 140625u || MPU6500_READ_MAX_HZ > 18000000u
#error MPU6500_READ_MAX_HZ_out_of_range
#endif

static uint8_t ready, angle_ready;
static uint8_t sample_time_ready;
static uint32_t last_angle_us;
static uint8_t tx[16];
static volatile uint8_t rx[16]; /* DMA写入 */
static float q[4];
static float gyro_bias[3], integral[3]; /* 原始值零偏；rad/s残余修正 */
static uint8_t CalibrateGyro(void);

/* SPI时钟来自PCLK1；只修改SPI分频，不修改系统时钟。 */
static void SetSPIClock(uint32_t max_hz)
{
    RCC_ClocksTypeDef clocks;
    SPI_InitTypeDef spi;
    uint16_t divider = 2, prescaler = SPI_BaudRatePrescaler_2;
    RCC_GetClocksFreq(&clocks);
    while (clocks.PCLK1_Frequency / divider > max_hz && divider < 256u) {
        divider *= 2u;
        prescaler += 8u; /* BR字段每增加1，分频再乘2 */
    }
    SPI_Cmd(SPI2, DISABLE);
    SPI_StructInit(&spi);
    spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    spi.SPI_Mode = SPI_Mode_Master;
    spi.SPI_DataSize = SPI_DataSize_8b;
    spi.SPI_CPOL = SPI_CPOL_High;
    spi.SPI_CPHA = SPI_CPHA_2Edge;
    spi.SPI_NSS = SPI_NSS_Soft;
    spi.SPI_BaudRatePrescaler = prescaler;
    spi.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_Init(SPI2, &spi);
    SPI_Cmd(SPI2, ENABLE);
}

static void PortInit(void)
{
    GPIO_InitTypeDef gpio;
    DMA_InitTypeDef dma;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    SPI_I2S_DeInit(SPI2);
    DMA_DeInit(DMA1_Channel4);
    DMA_DeInit(DMA1_Channel5);

    /* 先预置ODR，再切输出，避免CS在初始化时出现低脉冲。 */
    GPIO_SetBits(GPIOB, CS_PIN | GPIO_Pin_13);
    gpio.GPIO_Pin = CS_PIN;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);
    gpio.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &gpio);
    gpio.GPIO_Pin = GPIO_Pin_14;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &gpio);
    SetSPIClock(1000000u);

    DMA_StructInit(&dma);
    dma.DMA_PeripheralBaseAddr = (uintptr_t)&SPI2->DR;
    dma.DMA_MemoryBaseAddr = (uintptr_t)rx;
    dma.DMA_DIR = DMA_DIR_PeripheralSRC;
    dma.DMA_BufferSize = sizeof(rx);
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_Init(DMA1_Channel4, &dma);
    dma.DMA_MemoryBaseAddr = (uintptr_t)tx;
    dma.DMA_DIR = DMA_DIR_PeripheralDST;
    dma.DMA_Priority = DMA_Priority_High;
    DMA_Init(DMA1_Channel5, &dma);
}

/* 至少约1us；不使用共享SysTick，避免与其他延时函数争用。
 * 每轮至少一个NOP；循环开销会使实际等待略长，不用于精密计时。 */
static void CSWait(void)
{
    uint32_t cycles = SystemCoreClock/1000000u+1u;
    while (cycles--) __NOP();
}

/* 同一组DMA用于短寄存器事务和高速连续读取；不需要DMA中断。 */
static uint8_t Transfer(uint16_t count)
{
    uint32_t wait;
    uint8_t ok = 0;
    DMA_ClearFlag(DMA1_FLAG_GL4 | DMA1_FLAG_GL5);
    DMA_SetCurrDataCounter(DMA1_Channel4, count);
    DMA_SetCurrDataCounter(DMA1_Channel5, count);
    GPIO_ResetBits(GPIOB, CS_PIN);
    CSWait(); /* 满足片选建立时间 */
    DMA_Cmd(DMA1_Channel4, ENABLE); /* 先准备接收，后启动发送 */
    DMA_Cmd(DMA1_Channel5, ENABLE);
    SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx, ENABLE);

    for (wait = WAIT_LIMIT; wait != 0; --wait) {
        if (DMA_GetFlagStatus(DMA1_FLAG_TE4) || DMA_GetFlagStatus(DMA1_FLAG_TE5))
            break;
        if (DMA_GetFlagStatus(DMA1_FLAG_TC4)) { ok = 1; break; }
    }
    /* RX完成还要确认最后一位已发完，才能释放CS。 */
    if (ok) {
        for (wait = WAIT_LIMIT; wait != 0; --wait)
            if (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY) == RESET) break;
        if (wait == 0 || SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_OVR)) ok = 0;
    }
    SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx, DISABLE);
    DMA_Cmd(DMA1_Channel5, DISABLE);
    DMA_Cmd(DMA1_Channel4, DISABLE);
    if (!ok) { ready = 0; SPI_Cmd(SPI2, DISABLE); }
    CSWait(); /* 低速配置事务的CS保持时间也满足 */
    GPIO_SetBits(GPIOB, CS_PIN);
    CSWait(); /* 两次事务之间的CS高电平时间 */
    return ok;
}

static uint8_t ReadReg(uint8_t reg, uint8_t *value)
{
    tx[0] = reg | 0x80u;
    tx[1] = 0;
    if (!Transfer(2)) return 0;
    *value = rx[1];
    return 1;
}

static uint8_t WriteReg(uint8_t reg, uint8_t value)
{
    tx[0] = reg;
    tx[1] = value;
    return Transfer(2);
}

uint8_t MPU6500_Init(void)
{
    /* GYRO_CONFIG.FCHOICE_B=0；ACCEL_CONFIG2.ACCEL_FCHOICE_B=0，启用低通。
     * 低通约41Hz，输出率1kHz。所有配置读写均在<=1MHz进行。 */
    static const uint8_t config[][2] = {
        {USER_CTRL,     0x10}, /* 禁止IIC接口，不启用内部IIC主机 */
        {PWR_MGMT_1,    0x01}, /* 唤醒，自动选择最佳时钟 */
        {PWR_MGMT_2,    0x00}, /* 六轴全部开启 */
        {SMPLRT_DIV,    0x00},
        {CONFIG,        0x03},
        {GYRO_CONFIG,   0x18}, /* ±2000°/s */
        {ACCEL_CONFIG,  0x10}, /* ±8g */
        {ACCEL_CONFIG2, 0x03},
        {FIFO_EN,       0x00},
        {INT_PIN_CFG,   0x00}, /* 只有读INT_STATUS才清状态 */
        {INT_ENABLE,    0x01}  /* 数据就绪状态；无需连接INT引脚 */
    };
    uint8_t i, value;
    ready = angle_ready = 0;
    sample_time_ready = 0;
    last_angle_us = 0;
    PortInit();
    Delay_ms(100);
    if (!ReadReg(WHO_AM_I, &value) || value != 0x70u) return 0;
    if (!WriteReg(PWR_MGMT_1, 0x80u)) return 0;
    Delay_ms(100);
    for (i = 0; i < sizeof(config) / sizeof(config[0]); ++i) {
        if (!WriteReg(config[i][0], config[i][1])) return 0;
        if (config[i][0] == PWR_MGMT_1) Delay_ms(100);
        if (!ReadReg(config[i][0], &value) || value != config[i][1]) return 0;
    }
    Delay_ms(50); /* 传感器和滤波器稳定 */
    if (!ReadReg(INT_STATUS, &value)) return 0;
    for (i = 1; i < sizeof(tx); ++i) tx[i] = 0;
    SetSPIClock(MPU6500_READ_MAX_HZ); /* 此后只高速读取状态/传感器寄存器 */
    ready = 1;
    ready = CalibrateGyro();
    return ready;
}

static int16_t Signed16(uint8_t high, uint8_t low)
{
    int32_t value = ((uint16_t)high << 8) | low;
    if (value & 0x8000) value -= 65536;
    return (int16_t)value;
}

int8_t MPU6500_ReadRaw(MPU6500_Raw *raw)
{
    uint8_t i;
    MPU6500_Raw frame;
    if (!ready || raw == NULL) return -1;
    tx[0] = INT_STATUS | 0x80u;
    if (!Transfer(sizeof(tx))) return -1;
    /* 未定义状态位应为0；可识别MISO悬空拉高，不能代替CRC。 */
    if (rx[1] & 0xA4u) { ready = 0; return -1; }
    if (!(rx[1] & 0x01u)) return 0;
    for (i = 0; i < 3; ++i) {
        frame.accel[i] = Signed16(rx[2+2*i], rx[3+2*i]);
        frame.gyro[i]  = Signed16(rx[10+2*i], rx[11+2*i]);
    }
    frame.temperature = Signed16(rx[8], rx[9]);
    *raw = frame;
    return 1;
}

int8_t MPU6500_ReadAngles(uint32_t now_us, float angle_deg[3])
{
    MPU6500_Raw raw;
    float dt_ms;
    int8_t result;
    if (angle_deg == NULL) return -1;
    result = MPU6500_ReadRaw(&raw);
    if (result != 1) return result;
    dt_ms = sample_time_ready ? (float)(uint32_t)(now_us-last_angle_us)*0.001f : 0.0f;
    last_angle_us = now_us; /* 只有读到新帧才更新时间；不重复积分旧数据 */
    sample_time_ready = 1;
    return (int8_t)MPU6500_CalcAngles(&raw, dt_ms, angle_deg);
}

/* 200个不同的新样本；动作明显时拒绝标定，不将运动当作零偏。
 * 缓慢匀速绕重力轴旋转无法由此可靠识别，使用者仍必须保持静止。 */
static uint8_t CalibrateGyro(void)
{
    MPU6500_Raw raw;
    int32_t sum[3] = {0};
    int16_t gmin[3] = {32767,32767,32767}, gmax[3] = {-32768,-32768,-32768};
    int16_t amin[3] = {32767,32767,32767}, amax[3] = {-32768,-32768,-32768};
    uint16_t n = 0, attempts;
    uint8_t i;
    int8_t result;
    float ax, ay, az, norm2;
    for (attempts = 0; attempts < 600u && n < 200u; ++attempts) {
        Delay_ms(5);
        result = MPU6500_ReadRaw(&raw);
        if (result < 0) return 0;
        if (result == 0) continue;
        ax = raw.accel[0]/MPU6500_ACCEL_LSB_G;
        ay = raw.accel[1]/MPU6500_ACCEL_LSB_G;
        az = raw.accel[2]/MPU6500_ACCEL_LSB_G;
        norm2 = ax*ax+ay*ay+az*az;
        if (norm2 < 0.85f*0.85f || norm2 > 1.15f*1.15f) return 0;
        for (i = 0; i < 3; ++i) {
            if (fabsf((float)raw.gyro[i]) > 30.0f*MPU6500_GYRO_LSB_DPS) return 0;
            if (raw.gyro[i] < gmin[i]) gmin[i] = raw.gyro[i];
            if (raw.gyro[i] > gmax[i]) gmax[i] = raw.gyro[i];
            if (raw.accel[i] < amin[i]) amin[i] = raw.accel[i];
            if (raw.accel[i] > amax[i]) amax[i] = raw.accel[i];
            if (gmax[i]-gmin[i] > 2.0f*MPU6500_GYRO_LSB_DPS ||
                amax[i]-amin[i] > 0.10f*MPU6500_ACCEL_LSB_G) return 0;
            sum[i] += raw.gyro[i];
        }
        ++n;
    }
    if (n != 200u) return 0;
    for (i = 0; i < 3; ++i) {
        gyro_bias[i] = (float)sum[i]/n;
        integral[i] = 0.0f;
    }
    return 1;
}

/* 将机体系旋转增量(弧度)右乘到四元数；使用同一份旧q。 */
static void Rotate(float x, float y, float z)
{
    float q0=q[0], q1=q[1], q2=q[2], q3=q[3];
    float a=sqrtf(x*x+y*y+z*z), c, s;
    if (a > 1.0e-6f) { s=sinf(0.5f*a)/a; c=cosf(0.5f*a); }
    else             { s=0.5f; c=1.0f; }
    x*=s; y*=s; z*=s;
    q[0]=q0*c-q1*x-q2*y-q3*z;
    q[1]=q0*x+q1*c+q2*z-q3*y;
    q[2]=q0*y-q1*z+q2*c+q3*x;
    q[3]=q0*z+q1*y-q2*x+q3*c;
}

uint8_t MPU6500_CalcAngles(const MPU6500_Raw *raw, float dt_ms, float angle_deg[3])
{
    float ax, ay, az, norm, dt, roll, pitch, gain;
    float gx, gy, gz, vx, vy, vz, e[3];
    float q0, q1, q2, q3, s;
    uint8_t accel_ok, i;
    if (!ready || raw == NULL || angle_deg == NULL) return 0;
    if (!isfinite(dt_ms) || dt_ms < 0 || (angle_ready && dt_ms == 0)) return 0;
    if (angle_ready && dt_ms > 100.0f) { angle_ready = 0; return 0; }
    dt = dt_ms * 0.001f;
    ax = raw->accel[0]; ay = raw->accel[1]; az = raw->accel[2];
    norm = sqrtf(ax*ax + ay*ay + az*az);
    /* 起跳/自由落体/撞击时不把明显的运动加速度当作重力。 */
    accel_ok = norm > 0.8f*MPU6500_ACCEL_LSB_G && norm < 1.2f*MPU6500_ACCEL_LSB_G;
    if (accel_ok) { ax /= norm; ay /= norm; az /= norm; }
    if (!angle_ready) {
        if (!accel_ok) return 0;
        roll = atan2f(ay, az);
        pitch = atan2f(-ax, sqrtf(ay*ay + az*az));
        q[0] = cosf(roll*0.5f) * cosf(pitch*0.5f);
        q[1] = sinf(roll*0.5f) * cosf(pitch*0.5f);
        q[2] = cosf(roll*0.5f) * sinf(pitch*0.5f);
        q[3] = -sinf(roll*0.5f) * sinf(pitch*0.5f);
        for (i = 0; i < 3; ++i) integral[i] = 0.0f;
        angle_ready = 1;
    } else {
        gx = (raw->gyro[0]-gyro_bias[0]) * (RAD_PER_DEG/MPU6500_GYRO_LSB_DPS);
        gy = (raw->gyro[1]-gyro_bias[1]) * (RAD_PER_DEG/MPU6500_GYRO_LSB_DPS);
        gz = (raw->gyro[2]-gyro_bias[2]) * (RAD_PER_DEG/MPU6500_GYRO_LSB_DPS);
        /* 先用陀螺预测到本帧，再用本帧重力校正，转动不靠加速度慢慢追。 */
        Rotate((gx+integral[0])*dt, (gy+integral[1])*dt, (gz+integral[2])*dt);
        if (accel_ok) {
            q0 = q[0]; q1 = q[1]; q2 = q[2]; q3 = q[3];
            vx = 2.0f*(q1*q3-q0*q2);
            vy = 2.0f*(q0*q1+q2*q3);
            vz = q0*q0-q1*q1-q2*q2+q3*q3;
            e[0] = ay*vz-az*vy;
            e[1] = az*vx-ax*vz;
            e[2] = ax*vy-ay*vx;
            /* 大倾角误差先由P修正；接近重力方向后才估计残余零偏。 */
            if (ax*vx+ay*vy+az*vz > 0.985f) {
                for (i = 0; i < 3; ++i) {
                    integral[i] += MPU6500_TILT_KI*e[i]*dt;
                    if (integral[i] > MPU6500_I_LIMIT_DPS*RAD_PER_DEG)
                        integral[i] = MPU6500_I_LIMIT_DPS*RAD_PER_DEG;
                    if (integral[i] < -MPU6500_I_LIMIT_DPS*RAD_PER_DEG)
                        integral[i] = -MPU6500_I_LIMIT_DPS*RAD_PER_DEG;
                }
            }
            /* 离散校正比例随dt变化，20ms时约0.077；不是给最终角度叠低通。 */
            gain = 1.0f-expf(-MPU6500_TILT_KP*dt);
            Rotate(gain*e[0], gain*e[1], gain*e[2]);
        }
        norm = sqrtf(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
        if (!isfinite(norm) || norm < 1.0e-6f) { angle_ready = 0; return 0; }
        for (i = 0; i < 4; ++i) q[i] /= norm;
    }
    q0 = q[0]; q1 = q[1]; q2 = q[2]; q3 = q[3];
    s = 2.0f*(q0*q2-q1*q3);
    if (s > 1.0f) s = 1.0f;
    if (s < -1.0f) s = -1.0f;
    angle_deg[0] = asinf(s)/RAD_PER_DEG;
    angle_deg[1] = atan2f(2.0f*(q0*q1+q2*q3), 1.0f-2.0f*(q1*q1+q2*q2))/RAD_PER_DEG;
    angle_deg[2] = atan2f(2.0f*(q0*q3+q1*q2), 1.0f-2.0f*(q2*q2+q3*q3))/RAD_PER_DEG;
    return 1;
}
