#ifndef MPU6500_SPI_H
#define MPU6500_SPI_H

#include <stdint.h>

/* STM32F103C8T6 标准库。PB12=CS、PB13=SCK、PB14=MISO、PB15=MOSI。
 * 独占 SPI2、DMA1_Channel4(RX)/Channel5(TX)。初始化在启动阶段调用；
 * 读取/解算在主循环调用，禁止主循环与中断并发访问本驱动。 */
#define MPU6500_READ_MAX_HZ    18000000u /* 长线/转接板可降到9000000或4500000 */
#define MPU6500_ACCEL_LSB_G    4096.0f   /* ±8g */
#define MPU6500_GYRO_LSB_DPS   16.4f     /* ±2000°/s */
#define MPU6500_TILT_KP        4.0f      /* 重力校正增益，1/s */
#define MPU6500_TILT_KI        0.5f      /* 残余零偏校正增益，1/s^2 */
#define MPU6500_I_LIMIT_DPS    5.0f      /* 积分修正上限，°/s */

typedef struct {
    int16_t accel[3];       /* 芯片XYZ，已经过芯片内部低通 */
    int16_t gyro[3];        /* 芯片XYZ，已经过芯片内部低通 */
    int16_t temperature;
} MPU6500_Raw;

/* 初始化GPIO/SPI/DMA和MPU6500；不改系统时钟。1成功，0失败。
 * 上电后保持模块静止：内部约1秒采样标定陀螺零偏，初始姿态无需水平。
 * 检测到明显运动、量程异常或通信失败时返回0，请静止后重试。
 * 芯片ID必须为0x70；初始化同时清除姿态历史。 */
uint8_t MPU6500_Init(void);

/* 连续读取一帧六轴+温度。1有新数据，0暂无新数据，-1出错。
 * 返回0/-1不改输出；出错后需重新初始化。不会等待下一次采样。 */
int8_t MPU6500_ReadRaw(MPU6500_Raw *raw);


#endif
