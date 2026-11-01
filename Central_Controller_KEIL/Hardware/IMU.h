#ifndef __IMU_H
#define __IMU_H

#include <stdint.h>

#define RAD_TO_DEG (180.0f/3.1415926535f)
#define DEG_TO_RAD (3.1415926535f/180.0f)

void IMU_InitQuaternion(float ax, float ay, float az);
/* 用有效重力/磁场重建姿态并清9轴积分；磁场传入未校准的G值，内部校准一次。 */
int IMU_ResetAttitude(float ax, float ay, float az, float mx, float my, float mz,
                      float *roll, float *pitch, float *yaw);

void IMU_3Axis_Acc(float ax, float ay, float az, float *roll, float *pitch);
void IMU_3Axis_Gyro(float gx, float gy, float gz, float *roll, float *pitch, float *yaw, float dt);
void IMU_6Axis_Gyro_Acc(float ax, float ay, float az, 
						float gx, float gy, float gz, 
						float *roll, float *pitch, float *yaw, float dt);
void IMU_6Axis_Gyro_Acc_v(float ax, float ay, float az, 
							 float gx, float gy, float gz, 
							 float *roll, float *pitch, float *yaw, float *a_down, float dt);
void IMU_6Axis_Gyro_Acc_3v(float ax, float ay, float az, 
							 float gx, float gy, float gz, 
							 float *roll, float *pitch, float *yaw,
							 float *a_forward, float *a_right, float *a_down, float dt);
/* 1=已更新，0=输入/计算异常且保留原姿态；零加速度/磁场时跳过对应校正。 */
int IMU_9Axis_Gyro_Acc_Mag(float ax, float ay, float az, 
							float gx, float gy, float gz, 
							float mx, float my, float mz, 
							float *roll, float *pitch, float *yaw, float dt);
/* Formal==2调试模式：0=上电采集约30秒；1=使用下方已实测的参数。
 * 成功后串口会打印可复制的宏。正式运行前填好参数并改为1；不自动写Flash。 */
#ifndef MAG_USE_PRESET_CALIBRATION
#define MAG_USE_PRESET_CALIBRATION 0
#endif
#define MAG_BIAS_X_G  0.0f
#define MAG_BIAS_Y_G  0.0f
#define MAG_BIAS_Z_G  0.0f
#define MAG_SCALE_X   1.0f
#define MAG_SCALE_Y   1.0f
#define MAG_SCALE_Z   1.0f

#define MAG_CAL_SAMPLE_MS       20u
#define MAG_CAL_SAMPLES         1500u /* 约30秒，缓慢向各方向翻转，覆盖三轴最大/最小值 */
#define MAG_CAL_MIN_SAMPLES     600u
#define MAG_CAL_MIN_SPAN_G      0.20f /* 每轴最大值-最小值至少0.20G，拒绝静止/只平转 */
#define MAG_CAL_MAX_RADIUS_RATIO 3.0f /* 最大半径/最小半径，过大时提示重新采样 */

typedef struct {
    float bias[3];       /* 硬铁偏置，单位G */
    float scale[3];      /* 各轴比例修正，无单位；不是完整软铁矩阵 */
    uint8_t calibrated;
} MagCalib_t;
extern MagCalib_t mag_calib;

/* Begin重置本轮采样；Collect只收集，不在运行时自动改参数。
 * 输入必须是未经校准的G值。Finish返回1成功，0覆盖不足；失败保留旧参数。 */
void Mag_Calibrate_Begin(void);
void Mag_Calibrate_Collect(float mx, float my, float mz);
int Mag_Calibrate_Finish(void);
void Mag_Apply_Calibration(float *mx, float *my, float *mz);
/* 未校准磁场(G)+Roll/Pitch(度)，内部校准并做倾斜补偿；1成功，0无有效航向。 */
int Mag_GetHeading(float mx, float my, float mz, float roll_deg, float pitch_deg,
                   float *heading_deg);

#endif
