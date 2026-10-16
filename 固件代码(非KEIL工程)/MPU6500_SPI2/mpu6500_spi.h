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

/* 一次完成读取和解算。now_us为本次开始读取前的单调微秒时间戳。
 * 内部计算相邻新帧的时间间隔，uint32_t时间戳允许回绕。
 * 输出[Pitch,Roll,Yaw]，单位度。1成功、0暂无有效角度、-1通信/参数错误；
 * 0/-1不改输出，通信错误后需重新初始化。只在主循环调用。
 * 与手动ReadRaw+CalcAngles二选一，不要交替更新同一份姿态历史。 */
int8_t MPU6500_ReadAngles(uint32_t now_us, float angle_deg[3]);

/* 输入一帧新数据，dt_ms为相邻送入样本的实际时间间隔(毫秒)。
 * 20ms调用一次就传20.0f，最好用实测间隔；不要把Delay_ms当成整个周期。
 * 首帧允许dt_ms=0，以重力建立倾角，Yaw从0开始。
 * 输出[Pitch,Roll,Yaw]，单位度，范围分别[-90,90]/[-180,180]/[-180,180]。
 * 1成功，0失败且不改输出。间隔>100ms会清除历史，下一帧重新建姿态。
 * 无磁力计：Yaw是相对角度，长期漂移；芯片坐标需与车体坐标对应。 */
uint8_t MPU6500_CalcAngles(const MPU6500_Raw *raw, float dt_ms,
                           float angle_deg[3]);

#endif
