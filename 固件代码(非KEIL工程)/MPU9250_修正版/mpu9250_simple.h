#ifndef MPU9250_SIMPLE_H
#define MPU9250_SIMPLE_H

#include <stdint.h>

/* 软件一阶低通的截止频率(Hz)，0表示旁路；在CalcAngles内按实际dt执行。
 * 芯片内部加速度/陀螺低通约41Hz，这里是额外的软件滤波。 */
#define MPU9250_ACCEL_LPF_HZ 30.0f
#define MPU9250_GYRO_LPF_HZ  30.0f
#define MPU9250_MAG_LPF_HZ   0.0f

/* STM32软件IIC；GPIO和Delay_ms/Delay_us由工程提供。
 * 只有这5个对外函数。1成功，0失败/暂无有效数据，失败不改输出数组。
 * 全部由同一主循环调用，不支持并发；Init会重置全部姿态状态。
 */
uint8_t MPU9250_Init(void);
uint8_t MPU9250_ReadAccel(int16_t xyz[3]); /* MPU XYZ原始值，16384 LSB/g */
uint8_t MPU9250_ReadGyro(int16_t xyz[3]);  /* MPU XYZ原始值，16.4 LSB/(度/秒) */
uint8_t MPU9250_ReadMag(int16_t xyz[3]);   /* AK8963自身XYZ原始值 */

/* 输入未经单位换算/坐标变换的原始数组，无新磁场时mag可为NULL。
 * 内部对九轴做软件低通，滤波保留浮点精度；三个读取接口本身不滤波。
 * dt_ms是相邻送入本函数的样本间隔，单位毫秒，例如5.0f代表5ms。
 * 首次可传0；后续dt必须为正，调用方无论成功失败均推进采样时间戳。
 * 已建立姿态时若间隔>100ms：本次返回0并清除状态；下一帧重新建立，
 * 不拿当前角速度补算丢失的整段运动。首次/重建时要求加速度接近1g。
 * 首次无有效磁场也能开始，Yaw从0起，后来有磁场时逐步校正。
 * angle_deg顺序为[Pitch, Roll, Yaw]，单位度；Pitch为[-90,90]，
 * Roll和Yaw为[-180,180]。这与旧版[Roll,Pitch,Yaw]及0..360航向不同。
 * 这不是车辆轴向安装标定；需确认芯片坐标与车体坐标一致。
 */
uint8_t MPU9250_CalcAngles(const int16_t accel[3], const int16_t gyro[3],
                         const int16_t mag[3], float dt_ms, float angle_deg[3]);

#endif
