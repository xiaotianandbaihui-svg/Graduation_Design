#ifndef MPU9250_H
#define MPU9250_H

#include <stdint.h>
#include "soft_iic.h"

#define MPU9250_ADDR_AD0_LOW     0x68u
#define MPU9250_ADDR_AD0_HIGH    0x69u
#define AK8963_ADDR              0x0Cu

typedef enum {
    MPU9250_OK = 0,
    MPU9250_NO_NEW_DATA,
    MPU9250_ERR_ARGUMENT,
    MPU9250_ERR_IIC,
    MPU9250_ERR_ID,
    MPU9250_ERR_VERIFY,
    MPU9250_ERR_NOT_INITIALIZED,
    MPU9250_ERR_MAG_OVERFLOW
} MPU9250_Status;

typedef struct {
    uint8_t addr7;
    uint8_t initialized;
    uint8_t mag_initialized;
    float mag_adjust[3];      /* 磁力计出厂灵敏度修正系数，不是整机磁场标定。 */
    IIC_Status last_iic_status;
} MPU9250;

typedef struct {
    int16_t accel[3];         /* X/Y/Z；本初始化下除以16384.0f得到g。 */
    int16_t gyro[3];          /* X/Y/Z；本初始化下除以16.4f得到deg/s。 */
    int16_t temperature;
} MPU9250_Raw;

/* dev先用 {0} 初始化。仅支持真正的MPU9250（WHO_AM_I=0x71）。
 * 设置：±2g、±2000deg/s、200Hz采样分频、DLPF档位3（约40Hz）。
 * 本函数不配置STM32时钟/GPIO/串口，不做姿态解算。
 */
MPU9250_Status MPU9250_Init(MPU9250 *dev, uint8_t addr7);

/* 可选：在Init成功后调用，配置AK8963为16位、连续测量模式2（100Hz）。
 * 旁路模式下，总线上只能有一个地址为0x0C的磁力计；不支持双模块同时旁路。
 */
MPU9250_Status MPU9250_InitMag(MPU9250 *dev);

/* 先检查数据就绪；没有新数据返回NO_NEW_DATA。
 * 失败或无新数据时，输出结构保持不变。
 */
MPU9250_Status MPU9250_ReadRaw(MPU9250 *dev, MPU9250_Raw *out);
MPU9250_Status MPU9250_ReadMag(MPU9250 *dev, float magnetic_uT[3]);

#endif
