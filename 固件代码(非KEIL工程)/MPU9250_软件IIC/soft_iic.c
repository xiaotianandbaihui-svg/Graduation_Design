#include "soft_iic.h"
#include "iic_port.h"

/* 理想位周期约10us，实际频率还受GPIO函数和中断开销影响，通常低于100kHz。 */
#define IIC_HALF_US          5u
#define IIC_SCL_TIMEOUT_US   1000u

static void half_delay(void)
{
    IIC_DelayUs(IIC_HALF_US);
}

/* SCK(1)只是释放时钟线，必须确认线上的真实电平已经变高。 */
static IIC_Status scl_high(void)
{
    uint32_t waited;
    SCK(1);
    for (waited = 0; waited < IIC_SCL_TIMEOUT_US; ++waited) {
        if (SCK_Read()) {
            return IIC_OK;
        }
        IIC_DelayUs(1);
    }
    return IIC_ERR_SCL_TIMEOUT;
}

IIC_Status IIC_Init(void)
{
    IIC_Status status;
    SDA(1);
    status = scl_high();
    if (status != IIC_OK) {
        return status;
    }
    half_delay();
    return SDA_Read() ? IIC_OK : IIC_ERR_BUS_BUSY;
}

/* START：SCL保持高电平时，SDA由高变低。
 * repeated=1用于读寄存器时的重复起始，不先发STOP。
 */
static IIC_Status start(uint8_t repeated)
{
    IIC_Status status;
    if (repeated) {
        SCK(0);
    }
    SDA(1);
    half_delay();
    status = scl_high();
    if (status != IIC_OK) {
        return status;
    }
    half_delay();
    if (!SDA_Read()) {
        return IIC_ERR_BUS_BUSY;
    }
    SDA(0);
    half_delay();
    SCK(0);
    half_delay();
    return IIC_OK;
}

/* STOP：SCL保持高电平时，SDA由低变高。 */
static IIC_Status stop(void)
{
    IIC_Status status;
    SCK(0);
    SDA(0);
    half_delay();
    status = scl_high();
    if (status != IIC_OK) {
        SDA(1);              /* 无法生成STOP时仍释放总线，避免主机持续拉低。 */
        return status;
    }
    half_delay();
    SDA(1);
    half_delay();
    return SDA_Read() ? IIC_OK : IIC_ERR_STOP;
}

/* 发送8位，最高位在前；第9个时钟读取从机ACK（SDA=0）。 */
static IIC_Status send_byte(uint8_t value)
{
    unsigned int bit;
    uint8_t nack;
    IIC_Status status;

    for (bit = 0; bit < 8u; ++bit) {
        SCK(0);
        SDA((value & 0x80u) ? 1u : 0u);
        half_delay();
        status = scl_high();
        if (status != IIC_OK) {
            return status;
        }
        half_delay();
        SCK(0);
        value = (uint8_t)(value << 1);
    }

    SDA(1);                  /* 主机释放SDA，让从机驱动ACK。 */
    half_delay();
    status = scl_high();
    if (status != IIC_OK) {
        return status;
    }
    half_delay();
    nack = SDA_Read();
    SCK(0);
    half_delay();
    return nack ? IIC_ERR_NACK : IIC_OK;
}

/* 读取8位后，由主机发送第9位：还有数据则ACK，最后一字节则NACK。 */
static IIC_Status read_byte(uint8_t *value, uint8_t ack)
{
    unsigned int bit;
    uint8_t result = 0;
    IIC_Status status;

    SDA(1);
    for (bit = 0; bit < 8u; ++bit) {
        SCK(0);
        half_delay();
        status = scl_high();
        if (status != IIC_OK) {
            return status;
        }
        half_delay();
        result = (uint8_t)((result << 1) | (SDA_Read() ? 1u : 0u));
        SCK(0);
    }

    SDA(ack ? 0u : 1u);
    half_delay();
    status = scl_high();
    if (status != IIC_OK) {
        return status;
    }
    half_delay();
    SCK(0);
    half_delay();
    SDA(1);
    *value = result;
    return IIC_OK;
}

static IIC_Status finish(IIC_Status status)
{
    IIC_Status stop_status = stop();
    /* 已有通信错误时保留原始原因；成功通信也必须检查STOP是否完成。 */
    return status != IIC_OK ? status : stop_status;
}

IIC_Status IIC_WriteRegs(uint8_t addr7, uint8_t reg,
                         const uint8_t *data, size_t len)
{
    size_t i;
    IIC_Status status;
    if (addr7 > 0x7Fu || data == NULL || len == 0u) {
        return IIC_ERR_ARGUMENT;
    }

    status = start(0);
    if (status != IIC_OK) {
        return status;       /* 未取得总线，不强行发STOP。 */
    }
    status = send_byte((uint8_t)(addr7 << 1));
    if (status == IIC_OK) {
        status = send_byte(reg);
    }
    for (i = 0; i < len && status == IIC_OK; ++i) {
        status = send_byte(data[i]);
    }
    return finish(status);
}

IIC_Status IIC_ReadRegs(uint8_t addr7, uint8_t reg,
                        uint8_t *data, size_t len)
{
    size_t i;
    IIC_Status status;
    if (addr7 > 0x7Fu || data == NULL || len == 0u) {
        return IIC_ERR_ARGUMENT;
    }

    status = start(0);
    if (status != IIC_OK) {
        return status;
    }
    status = send_byte((uint8_t)(addr7 << 1)); /* 先写寄存器地址。 */
    if (status == IIC_OK) {
        status = send_byte(reg);
    }
    if (status == IIC_OK) {
        status = start(1);                  /* 重复START。 */
    }
    if (status == IIC_OK) {
        status = send_byte((uint8_t)((addr7 << 1) | 1u));
    }
    for (i = 0; i < len && status == IIC_OK; ++i) {
        status = read_byte(&data[i], (uint8_t)(i + 1u < len));
    }
    return finish(status);
}

IIC_Status IIC_WriteReg(uint8_t addr7, uint8_t reg, uint8_t data)
{
    return IIC_WriteRegs(addr7, reg, &data, 1);
}

IIC_Status IIC_ReadReg(uint8_t addr7, uint8_t reg, uint8_t *data)
{
    return IIC_ReadRegs(addr7, reg, data, 1);
}
