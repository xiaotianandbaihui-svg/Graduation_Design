#ifndef SOFT_IIC_H
#define SOFT_IIC_H

#include <stddef.h>
#include <stdint.h>

typedef enum {
    IIC_OK = 0,
    IIC_ERR_ARGUMENT,
    IIC_ERR_BUS_BUSY,
    IIC_ERR_SCL_TIMEOUT,
    IIC_ERR_NACK,
    IIC_ERR_STOP
} IIC_Status;

/* 本实现用于单主机，不能被不同任务/中断同时调用。
 * 调用前自行使能GPIO时钟，将SDA/SCK配置成开漏输出。
 */
IIC_Status IIC_Init(void);

/* addr7一律传7位地址，例如MPU9250传0x68，不是0xD0。
 * reg为8位寄存器地址；连续读写要求从设备支持寄存器地址自动递增。
 * 失败时返回错误码，不能使用读缓冲区中可能只更新了一部分的数据。
 */
IIC_Status IIC_WriteRegs(uint8_t addr7, uint8_t reg,
                         const uint8_t *data, size_t len);
IIC_Status IIC_ReadRegs(uint8_t addr7, uint8_t reg,
                        uint8_t *data, size_t len);
IIC_Status IIC_WriteReg(uint8_t addr7, uint8_t reg, uint8_t data);
IIC_Status IIC_ReadReg(uint8_t addr7, uint8_t reg, uint8_t *data);

#endif
