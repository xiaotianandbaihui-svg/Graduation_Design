#ifndef MPU9250_SIMPLE_H
#define MPU9250_SIMPLE_H

#include <stdint.h>

/* 只有这 5 个驱动接口。返回 1 表示成功，0 表示失败/暂无有效数据。
 * GPIO 必须由你先配置好；失败时输出数组不变。
 */
uint8_t MPU9250_Init(void);
uint8_t MPU9250_ReadAccel(int16_t xyz[3]);
uint8_t MPU9250_ReadGyro(int16_t xyz[3]);
uint8_t MPU9250_ReadMag(int16_t xyz[3]);

/* 输入是上面读到的原始 XYZ；无新磁场数据时 mag 可传 0。
 * dt_ms：距上次成功计算的实际毫秒数，例如 10 表示 10ms。
 * 初始化后的首次计算可传 0；首次需要接近 1g 的加速度来建立姿态。
 * angle_deg：[横滚 roll，俯仰 pitch，航向 yaw]，单位度。
 * 四元数融合；无磁场时航向依赖陀螺积分。初始无磁场则航向从 0 开始。
 * 内部保存滤波状态，MPU9250_Init() 会清除状态；不支持并发调用。
 */
uint8_t MPU9250_CalcAngles(const int16_t accel[3], const int16_t gyro[3],
                           const int16_t mag[3], float dt_ms, float angle_deg[3]);

#endif
