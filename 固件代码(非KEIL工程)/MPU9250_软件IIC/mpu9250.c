#include "mpu9250.h"
#include "iic_port.h"

#define REG_SMPLRT_DIV       0x19u
#define REG_CONFIG           0x1Au
#define REG_GYRO_CONFIG      0x1Bu
#define REG_ACCEL_CONFIG     0x1Cu
#define REG_ACCEL_CONFIG2    0x1Du
#define REG_FIFO_EN          0x23u
#define REG_INT_PIN_CFG      0x37u
#define REG_INT_ENABLE       0x38u
#define REG_INT_STATUS       0x3Au
#define REG_ACCEL_XOUT_H     0x3Bu
#define REG_USER_CTRL        0x6Au
#define REG_PWR_MGMT_1       0x6Bu
#define REG_PWR_MGMT_2       0x6Cu
#define REG_WHO_AM_I         0x75u

#define MAG_WIA              0x00u
#define MAG_ST1              0x02u
#define MAG_HXL              0x03u
#define MAG_CNTL1            0x0Au
#define MAG_CNTL2            0x0Bu
#define MAG_ASAX             0x10u

static void delay_ms(uint32_t ms)
{
    while (ms-- > 0u) {
        IIC_DelayUs(1000);
    }
}

static MPU9250_Status write_reg(MPU9250 *dev, uint8_t addr,
                                uint8_t reg, uint8_t value)
{
    dev->last_iic_status = IIC_WriteReg(addr, reg, value);
    return dev->last_iic_status == IIC_OK ? MPU9250_OK : MPU9250_ERR_IIC;
}

static MPU9250_Status read_regs(MPU9250 *dev, uint8_t addr,
                                uint8_t reg, uint8_t *data, size_t len)
{
    dev->last_iic_status = IIC_ReadRegs(addr, reg, data, len);
    return dev->last_iic_status == IIC_OK ? MPU9250_OK : MPU9250_ERR_IIC;
}

static MPU9250_Status write_verify(MPU9250 *dev, uint8_t addr,
                                   uint8_t reg, uint8_t value, uint8_t mask)
{
    uint8_t readback;
    MPU9250_Status status = write_reg(dev, addr, reg, value);
    if (status != MPU9250_OK) {
        return status;
    }
    status = read_regs(dev, addr, reg, &readback, 1);
    if (status != MPU9250_OK) {
        return status;
    }
    return (readback & mask) == (value & mask)
               ? MPU9250_OK : MPU9250_ERR_VERIFY;
}

/* 使用有符号范围内的转换，正确处理0x8000等负数边界。 */
static int16_t signed16(uint8_t high, uint8_t low)
{
    uint16_t value = (uint16_t)(((uint16_t)high << 8) | low);
    int32_t result = value;
    if (value & 0x8000u) {
        result -= 65536;
    }
    return (int16_t)result;
}

MPU9250_Status MPU9250_Init(MPU9250 *dev, uint8_t addr7)
{
    static const struct {
        uint8_t reg, value, mask;
    } config[] = {
        {REG_PWR_MGMT_2,    0x00, 0x3F}, /* 打开三轴加速度和三轴陀螺仪。 */
        {REG_USER_CTRL,     0x00, 0x70}, /* 不用FIFO及内部I2C主机，保留外部I2C接口。 */
        {REG_FIFO_EN,       0x00, 0xFF}, /* 不向FIFO写传感器数据。 */
        {REG_CONFIG,        0x03, 0x3F}, /* 陀螺仪DLPF档位3，约41Hz。 */
        {REG_GYRO_CONFIG,   0x18, 0xFB}, /* ±2000deg/s；FCHOICE_B=00，启用DLPF。 */
        {REG_ACCEL_CONFIG,  0x00, 0xF8}, /* ±2g，关闭自检，保留位写0。 */
        {REG_ACCEL_CONFIG2, 0x03, 0x0F}, /* 加速度DLPF档位3，约40Hz。 */
        {REG_SMPLRT_DIV,    0x04, 0xFF}, /* 当前滤波配置下1000/(1+4)=200Hz。 */
        {REG_INT_PIN_CFG,   0x00, 0xF2}, /* 默认中断行为；磁力计初始化时再开旁路。 */
        {REG_INT_ENABLE,    0x01, 0x59}  /* 开启原始数据就绪；可轮询INT_STATUS。 */
    };
    size_t i;
    uint8_t id;
    MPU9250_Status status;

    if (dev == NULL) {
        return MPU9250_ERR_ARGUMENT;
    }
    dev->initialized = 0;
    dev->mag_initialized = 0;
    dev->last_iic_status = IIC_OK;
    for (i = 0; i < 3u; ++i) {
        dev->mag_adjust[i] = 1.0f;
    }
    if (addr7 != MPU9250_ADDR_AD0_LOW && addr7 != MPU9250_ADDR_AD0_HIGH) {
        return MPU9250_ERR_ARGUMENT;
    }
    dev->addr7 = addr7;

    /* 1. 等待上电稳定，释放总线并检查实际电平。 */
    delay_ms(100);
    dev->last_iic_status = IIC_Init();
    if (dev->last_iic_status != IIC_OK) {
        return MPU9250_ERR_IIC;
    }

    /* 2. 确认访问的是MPU9250，避免把其他同地址器件直接复位。 */
    status = read_regs(dev, addr7, REG_WHO_AM_I, &id, 1);
    if (status != MPU9250_OK) {
        return status;
    }
    if (id != 0x71u) {
        return MPU9250_ERR_ID;
    }

    /* 3. 软件复位。复位位自动清除，所以这里不做立即写回校验。 */
    status = write_reg(dev, addr7, REG_PWR_MGMT_1, 0x80);
    if (status != MPU9250_OK) {
        return status;
    }
    delay_ms(100);

    /* 4. 解除休眠，CLKSEL=1：PLL可用时选PLL，否则使用内部时钟。 */
    status = write_verify(dev, addr7, REG_PWR_MGMT_1, 0x01, 0xFF);
    if (status != MPU9250_OK) {
        return status;
    }
    delay_ms(100);

    /* 5. 设置各轴、量程、滤波及采样分频，逐项检查ACK和写回值。 */
    for (i = 0; i < sizeof(config) / sizeof(config[0]); ++i) {
        status = write_verify(dev, addr7, config[i].reg,
                              config[i].value, config[i].mask);
        if (status != MPU9250_OK) {
            return status;
        }
    }
    delay_ms(100); /* 等待滤波与传感器输出稳定，启动延时取保守值。 */
    dev->initialized = 1;
    return MPU9250_OK;
}

MPU9250_Status MPU9250_InitMag(MPU9250 *dev)
{
    unsigned int i;
    uint8_t id, asa[3];
    MPU9250_Status status;
    if (dev == NULL) {
        return MPU9250_ERR_ARGUMENT;
    }
    if (!dev->initialized) {
        return MPU9250_ERR_NOT_INITIALIZED;
    }
    dev->mag_initialized = 0;

    /* 1. 关闭MPU内部I2C主机，打开旁路，让STM32直接访问0x0C。 */
    status = write_verify(dev, dev->addr7, REG_USER_CTRL, 0x00, 0x70);
    if (status != MPU9250_OK) return status;
    status = write_verify(dev, dev->addr7, REG_INT_PIN_CFG, 0x02, 0xF2);
    if (status != MPU9250_OK) return status;
    delay_ms(1);

    /* 2. AK8963的芯片ID应为0x48。 */
    status = read_regs(dev, AK8963_ADDR, MAG_WIA, &id, 1);
    if (status != MPU9250_OK) return status;
    if (id != 0x48u) return MPU9250_ERR_ID;
    status = write_reg(dev, AK8963_ADDR, MAG_CNTL2, 0x01);
    if (status != MPU9250_OK) return status;
    delay_ms(1);

    /* 3. 进入Fuse ROM访问模式，读出厂灵敏度修正系数。 */
    status = write_reg(dev, AK8963_ADDR, MAG_CNTL1, 0x00);
    if (status != MPU9250_OK) return status;
    delay_ms(1);
    status = write_reg(dev, AK8963_ADDR, MAG_CNTL1, 0x0F);
    if (status != MPU9250_OK) return status;
    delay_ms(1);
    status = read_regs(dev, AK8963_ADDR, MAG_ASAX, asa, 3);
    if (status != MPU9250_OK) return status;
    for (i = 0; i < 3u; ++i) {
        dev->mag_adjust[i] = ((float)asa[i] - 128.0f) / 256.0f + 1.0f;
    }

    /* 4. 切换测量模式前先回到掉电模式，再设16位、连续模式2。 */
    status = write_reg(dev, AK8963_ADDR, MAG_CNTL1, 0x00);
    if (status != MPU9250_OK) return status;
    delay_ms(1);
    status = write_verify(dev, AK8963_ADDR, MAG_CNTL1, 0x16, 0x1F);
    if (status != MPU9250_OK) return status;
    delay_ms(1);
    dev->mag_initialized = 1;
    return MPU9250_OK;
}

MPU9250_Status MPU9250_ReadRaw(MPU9250 *dev, MPU9250_Raw *out)
{
    uint8_t ready, data[14];
    unsigned int i;
    MPU9250_Raw sample;
    MPU9250_Status status;
    if (dev == NULL || out == NULL) return MPU9250_ERR_ARGUMENT;
    if (!dev->initialized) return MPU9250_ERR_NOT_INITIALIZED;

    status = read_regs(dev, dev->addr7, REG_INT_STATUS, &ready, 1);
    if (status != MPU9250_OK) return status;
    if (!(ready & 0x01u)) return MPU9250_NO_NEW_DATA;

    /* 从0x3B连续读14字节：加速度6、温度2、角速度6，均为高字节在前。 */
    status = read_regs(dev, dev->addr7, REG_ACCEL_XOUT_H, data, sizeof(data));
    if (status != MPU9250_OK) return status;
    for (i = 0; i < 3u; ++i) {
        sample.accel[i] = signed16(data[2u * i], data[2u * i + 1u]);
        sample.gyro[i] = signed16(data[8u + 2u * i], data[9u + 2u * i]);
    }
    sample.temperature = signed16(data[6], data[7]);
    *out = sample;
    return MPU9250_OK;
}

MPU9250_Status MPU9250_ReadMag(MPU9250 *dev, float magnetic_uT[3])
{
    uint8_t st1, data[7];
    unsigned int i;
    MPU9250_Status status;
    if (dev == NULL || magnetic_uT == NULL) return MPU9250_ERR_ARGUMENT;
    if (!dev->mag_initialized) return MPU9250_ERR_NOT_INITIALIZED;

    status = read_regs(dev, AK8963_ADDR, MAG_ST1, &st1, 1);
    if (status != MPU9250_OK) return status;
    if (!(st1 & 0x01u)) return MPU9250_NO_NEW_DATA;

    /* 必须把ST2也读走，结束本次读取，才能正常更新后续磁场数据。 */
    status = read_regs(dev, AK8963_ADDR, MAG_HXL, data, sizeof(data));
    if (status != MPU9250_OK) return status;
    if (data[6] & 0x08u) return MPU9250_ERR_MAG_OVERFLOW;
    for (i = 0; i < 3u; ++i) {
        /* AK8963与加速度/陀螺仪不同：低字节在前。 */
        int16_t raw = signed16(data[2u * i + 1u], data[2u * i]);
        magnetic_uT[i] = (float)raw * 0.15f * dev->mag_adjust[i];
    }
    return MPU9250_OK;
}
