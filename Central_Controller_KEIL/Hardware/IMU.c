#include <stdint.h>
#include <math.h>
#include "IMU.h"

/*用于存储姿态的四元数*/
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float exInt9, eyInt9, ezInt9;

int IMU_ResetAttitude(float ax, float ay, float az, float mx, float my, float mz,
                      float *roll, float *pitch, float *yaw)
{
    float r, p, y, hx, hy, cr, sr, cp, sp, cy, sy;
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (!isfinite(norm) || norm < 0.000001f) return 0;
    Mag_Apply_Calibration(&mx, &my, &mz);
    r = atan2f(ay, az);
    p = atan2f(-ax, hypotf(ay, az));
    hx = mx*cosf(p) + (my*sinf(r) + mz*cosf(r))*sinf(p);
    hy = my*cosf(r) - mz*sinf(r);
    norm = hypotf(hx, hy);
    if (!isfinite(norm) || norm < 0.000001f) return 0;
    y = atan2f(-hy, hx);
    cr = cosf(r*0.5f); sr = sinf(r*0.5f);
    cp = cosf(p*0.5f); sp = sinf(p*0.5f);
    cy = cosf(y*0.5f); sy = sinf(y*0.5f);
    q0 = cy*cp*cr + sy*sp*sr;
    q1 = cy*cp*sr - sy*sp*cr;
    q2 = cy*sp*cr + sy*cp*sr;
    q3 = sy*cp*cr - cy*sp*sr;
    exInt9 = eyInt9 = ezInt9 = 0;
    *roll = r*RAD_TO_DEG; *pitch = p*RAD_TO_DEG; *yaw = y*RAD_TO_DEG;
    return 1;
}

/**
  * 函    数：姿态四元数初始化
  * 参    数：ax ay az 加速度值
  * 返 回 值：无
  * 说    明：使用加速度值初始化全局变量的姿态四元数，
  *           将加速度计解算出来的姿态，同步到四元数，缩短互补滤波器的建立时间
  */
void IMU_InitQuaternion(float ax, float ay, float az)
{
	/*使用加速度值，计算得到横滚角和俯仰角*/
	float theta = atan2(ay, az);
	float phi = atan2(-ax, sqrt(az * az + ay * ay));
	float psi = 0;
	
	/*重复的计算放在前面统一执行*/
	float a = psi * 0.5f, b = phi * 0.5f, g = theta * 0.5f;
	float ca = cosf(a), sa = sinf(a);
	float cb = cosf(b), sb = sinf(b);
	float cg = cosf(g), sg = sinf(g);
	
	/*初始化四元数*/
	q0 = ca*cb*cg + sa*sb*sg;
	q1 = ca*cb*sg - sa*sb*cg;
	q2 = ca*sb*cg + sa*cb*sg;
	q3 = sa*cb*cg - ca*sb*sg;
	
	/*四元数归一化*/
	float norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 /= norm;
	q1 /= norm;
	q2 /= norm;
	q3 /= norm;
}

/**
  * 函    数：3轴姿态解算（仅使用加速度计）
  * 参    数：ax ay az 加速度值，单位：任意
  * 参    数：roll pitch 解算出来的横滚角和俯仰角，输出参数，单位：度
  * 返 回 值：无
  * 说    明：只在静止状态下准确，运动时会受运动加速度影响
  *           加速度计无法解算出偏航角
  */
void IMU_3Axis_Acc(float ax, float ay, float az, float *roll, float *pitch)
{
	/*使用加速度值，计算得到横滚角和俯仰角*/
	*roll = atan2(ay, az) * RAD_TO_DEG;
	*pitch = atan2(-ax, sqrt(az * az + ay * ay)) * RAD_TO_DEG;
}

/**
  * 函    数：3轴姿态解算（仅使用陀螺仪）
  * 参    数：gx gy gz 陀螺仪角速度值，单位：度每秒
  * 参    数：roll pitch yaw 解算出来的横滚角、俯仰角和偏航角，输出参数，单位：度
  * 参    数：dt 调用此函数的周期，单位：秒
  * 返 回 值：无
  * 说    明：使用陀螺仪更新全局变量的姿态四元数，输出此四元数对应的欧拉角
  *           输出的欧拉角会因为陀螺仪零偏而逐渐漂移
  */
void IMU_3Axis_Gyro(float gx, float gy, float gz, float *roll, float *pitch, float *yaw, float dt)
{
	/*角度制转弧度制*/
	/*若输入参数已经是弧度制，则不需要此转换*/
	gx *= DEG_TO_RAD;
	gy *= DEG_TO_RAD;
	gz *= DEG_TO_RAD;
	
	/*四元数更新*/
    q0 += 0.5f * (-q1*gx - q2*gy - q3*gz) * dt;
    q1 += 0.5f * ( q0*gx - q3*gy + q2*gz) * dt;
    q2 += 0.5f * ( q3*gx + q0*gy - q1*gz) * dt;
    q3 += 0.5f * (-q2*gx + q1*gy + q0*gz) * dt;
    
	/*四元数归一化*/
    float norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 /= norm;
    q1 /= norm;
    q2 /= norm;
    q3 /= norm;
	
	/*四元数转欧拉角*/
    *roll = atan2f(2.0f*(q2*q3 + q0*q1), 1.0f - 2.0f*(q1*q1 + q2*q2)) * RAD_TO_DEG;
	*pitch = asinf(2.0f * (q0*q2 - q1*q3)) * RAD_TO_DEG;
    *yaw = atan2f(2.0f*(q1*q2 + q0*q3), 1.0f - 2.0f*(q2*q2 + q3*q3)) * RAD_TO_DEG;
}

/**
  * 函    数：6轴姿态解算（使用加速度计和陀螺仪互补滤波融合）
  * 参    数：ax ay az 加速度值，单位：任意
  * 参    数：gx gy gz 陀螺仪角速度值，单位：度每秒
  * 参    数：roll pitch yaw 解算出来的横滚角、俯仰角和偏航角，输出参数，单位：度
  * 参    数：dt 调用此函数的周期，单位：秒
  * 返 回 值：无
  * 说    明：使用加速度计和陀螺仪互补滤波融合，更新全局变量的姿态四元数，输出此四元数对应的欧拉角
  *           输出的横滚角和俯仰角准确无漂移，但偏航角会因为陀螺仪零偏而逐渐漂移
  */
void IMU_6Axis_Gyro_Acc(float ax, float ay, float az, 
						float gx, float gy, float gz, 
						float *roll, float *pitch, float *yaw, float dt)
{
	/*角度制转弧度制*/
	/*若输入参数已经是弧度制，则不需要此转换*/
	gx *= DEG_TO_RAD;
	gy *= DEG_TO_RAD;
	gz *= DEG_TO_RAD;
	
	/*如果加速度全为0，则退出此函数，避免加速度归一化时产生除零错误*/
	if((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)) return;
	
	/*加速度归一化，对应实际重力方向[ax, ay, az]*/
    float norm = sqrtf(ax*ax + ay*ay + az*az);
	ax /= norm;
	ay /= norm;
	az /= norm;
	
	/*提取重力分量*/
	/*四元数提取的重力分量，对应理论重力方向[vx, vy, vz]*/
	float vx = 2.0f*(q1*q3 - q0*q2);
	float vy = 2.0f*(q0*q1 + q2*q3);
	float vz = 1.0f - 2.0f*(q1*q1 + q2*q2);
	
	/*实际重力方向和理论重力方向，叉乘得到误差*/
	/*向量a = [ax, ay, az]，向量v = [vx, vy, vz]，向量e = [ex, ey, ez]*/
	/*向量e = 向量a 叉乘 向量v*/
	/*e的模长 = |a||v|sinθ，e的方向垂直于a和v的平面*/
	/*又因为|a| = 1，|v| = 1，小角度下sinθ约等于θ，因此e的模长可以表示两个向量的夹角θ，即误差*/
	/*e的方向表示a到v旋转的轴，即减小误差的方向*/
	float ex = ay * vz - az * vy;
	float ey = az * vx - ax * vz;
	float ez = ax * vy - ay * vx;
	
	/*误差补偿*/
	float Kp = 0.2f;		//PI互补滤波器的Kp系数，决定了四元数姿态贴近加速度计姿态的快慢
	float Ki = 0.1f;		//PI互补滤波器的Ki系数，用于消除稳态误差，此值过大可能导致不稳定
	static float exInt, eyInt, ezInt;	//积分变量
	
	/*误差逐渐累加到积分变量中，进行误差积分*/
	exInt += ex * dt;
	eyInt += ey * dt;
	ezInt += ez * dt;
	
	/*误差[ex, ey, ez]经过PI互补滤波器，补偿给陀螺仪角速度，进而使四元数姿态逐渐贴近加速度计姿态*/
	gx += Kp * ex + Ki * exInt;
	gy += Kp * ey + Ki * eyInt;
	gz += Kp * ez + Ki * ezInt;
	
	/*四元数更新*/
	/*此时的[gx, gy, gz]，包含了真实转动和补偿转动，一起用于姿态更新*/
    q0 += 0.5f * (-q1*gx - q2*gy - q3*gz) * dt;
    q1 += 0.5f * ( q0*gx - q3*gy + q2*gz) * dt;
    q2 += 0.5f * ( q3*gx + q0*gy - q1*gz) * dt;
    q3 += 0.5f * (-q2*gx + q1*gy + q0*gz) * dt;
    
	/*四元数归一化*/
    norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 /= norm;
    q1 /= norm;
    q2 /= norm;
    q3 /= norm;
	
	/*四元数转欧拉角*/
    *roll = atan2f(2.0f*(q2*q3 + q0*q1), 1.0f - 2.0f*(q1*q1 + q2*q2)) * RAD_TO_DEG;
	*pitch = asinf(2.0f * (q0*q2 - q1*q3)) * RAD_TO_DEG;
    *yaw = atan2f(2.0f*(q1*q2 + q0*q3), 1.0f - 2.0f*(q2*q2 + q3*q3)) * RAD_TO_DEG;
}
void Mag_Apply_Calibration(float *mx, float *my, float *mz);

/**
  * 函    数：9轴姿态解算（使用加速度计、陀螺仪和磁力计互补滤波融合）
  * 参    数：ax ay az 加速度值，单位：任意
  * 参    数：gx gy gz 陀螺仪角速度值，单位：度每秒
  * 参    数：mx my mz 磁力值，单位与磁校准采样一致；内部应用校准
  * 参    数：roll pitch yaw 解算出来的横滚角、俯仰角和偏航角，输出参数，单位：度
  * 参    数：dt 调用此函数的周期，单位：秒
  * 返 回 值：1成功，0输入或计算异常
  * 说    明：使用加速度计、陀螺仪和磁力计互补滤波融合，更新全局变量的姿态四元数，输出此四元数对应的欧拉角
  *           返回1表示更新成功，0表示保留原姿态；精度受传感器标定和磁干扰影响
  */
int IMU_9Axis_Gyro_Acc_Mag(float ax, float ay, float az,
                          float gx, float gy, float gz,
                          float mx, float my, float mz,
                          float *roll, float *pitch, float *yaw, float dt)
{
    const float Kp = 4.0f, Ki = 0.01f;
    float norm, ex = 0.0f, ey = 0.0f, ez = 0.0f;
    float ix, iy, iz, n0, n1, n2, n3, sin_pitch;
    float q0q1 = q0*q1, q0q2 = q0*q2, q0q3 = q0*q3;
    float q1q1 = q1*q1, q1q2 = q1*q2, q1q3 = q1*q3;
    float q2q2 = q2*q2, q2q3 = q2*q3, q3q3 = q3*q3;

    if (!isfinite(gx) || !isfinite(gy) || !isfinite(gz) ||
        !isfinite(dt) || dt <= 0.0f) return 0;

    norm = sqrtf(ax*ax + ay*ay + az*az);
    if (!isfinite(norm)) return 0;
    /* 自由落体可能接近0g：不除以零，暂时只用陀螺仪/有效磁场。 */
    if (norm > 0.000001f) {
        float vx = 2.0f*(q1q3 - q0q2);
        float vy = 2.0f*(q0q1 + q2q3);
        float vz = 1.0f - 2.0f*(q1q1 + q2q2);
        ax /= norm; ay /= norm; az /= norm;
        ex = ay*vz - az*vy;
        ey = az*vx - ax*vz;
        ez = ax*vy - ay*vx;
    }

    norm = sqrtf(mx*mx + my*my + mz*mz);
    if (!isfinite(norm)) return 0;
    /* 磁力计首帧未就绪时可以为零，跳过本次磁场校正。 */
    if (norm > 0.000001f) {
        //Mag_Apply_Calibration(&mx, &my, &mz);
        norm = sqrtf(mx*mx + my*my + mz*mz);
        if (!isfinite(norm)) return 0;
    }
    if (norm > 0.000001f) {
        float hx, hy, bx, bz, wx, wy, wz;
        mx /= norm; my /= norm; mz /= norm;
        hx = 2.0f*(mx*(0.5f-q2q2-q3q3) + my*(q1q2-q0q3) + mz*(q1q3+q0q2));
        hy = 2.0f*(mx*(q1q2+q0q3) + my*(0.5f-q1q1-q3q3) + mz*(q2q3-q0q1));
        bx = sqrtf(hx*hx + hy*hy);
        bz = 2.0f*(mx*(q1q3-q0q2) + my*(q2q3+q0q1) + mz*(0.5f-q1q1-q2q2));
        wx = 2.0f*bx*(0.5f-q2q2-q3q3) + 2.0f*bz*(q1q3-q0q2);
        wy = 2.0f*bx*(q1q2-q0q3) + 2.0f*bz*(q0q1+q2q3);
        wz = 2.0f*bx*(q0q2+q1q3) + 2.0f*bz*(0.5f-q1q1-q2q2);
        ex += my*wz - mz*wy;
        ey += mz*wx - mx*wz;
        ez += mx*wy - my*wx;
    }

    ix = exInt9 + ex*dt; iy = eyInt9 + ey*dt; iz = ezInt9 + ez*dt;
    gx = gx*DEG_TO_RAD + Kp*ex + Ki*ix;
    gy = gy*DEG_TO_RAD + Kp*ey + Ki*iy;
    gz = gz*DEG_TO_RAD + Kp*ez + Ki*iz;

    /* 四个分量均使用更新前的四元数，计算成功后才一起提交。 */
    n0 = q0 + 0.5f*(-q1*gx - q2*gy - q3*gz)*dt;
    n1 = q1 + 0.5f*( q0*gx - q3*gy + q2*gz)*dt;
    n2 = q2 + 0.5f*( q3*gx + q0*gy - q1*gz)*dt;
    n3 = q3 + 0.5f*(-q2*gx + q1*gy + q0*gz)*dt;
    norm = sqrtf(n0*n0 + n1*n1 + n2*n2 + n3*n3);
    if (!isfinite(norm) || norm <= 0.000001f) return 0;
    q0 = n0/norm; q1 = n1/norm; q2 = n2/norm; q3 = n3/norm;
    exInt9 = ix; eyInt9 = iy; ezInt9 = iz;

    sin_pitch = 2.0f*(q0*q2 - q1*q3);
    sin_pitch = fmaxf(-1.0f, fminf(1.0f, sin_pitch));
    *roll = atan2f(2.0f*(q2*q3 + q0*q1), 1.0f-2.0f*(q1*q1 + q2*q2))*RAD_TO_DEG;
    *pitch = asinf(sin_pitch)*RAD_TO_DEG;
    *yaw = atan2f(2.0f*(q1*q2 + q0*q3), 1.0f-2.0f*(q2*q2 + q3*q3))*RAD_TO_DEG;
    return 1;
}

/*磁力计校准程序*/
typedef struct {
    float bias[3];      // 硬铁偏置 (offset)
    float scale[3];     // 各轴缩放系数
    uint8_t calibrated; // 是否已校准
} MagCalib_t;

MagCalib_t mag_calib = {
    .bias  = {0.0f, 0.0f, 0.0f},
    .scale = {1.0f, 1.0f, 1.0f},
    .calibrated = 0
};
void Mag_Calibrate_Collect(float mx, float my, float mz)
{
    static float mag_min[3] = { 9999.0f,  9999.0f,  9999.0f};
    static float mag_max[3] = {-9999.0f, -9999.0f, -9999.0f};
    static uint32_t sample_cnt = 0;

    // 更新最大最小值
    if (mx < mag_min[0]) mag_min[0] = mx;
    if (mx > mag_max[0]) mag_max[0] = mx;
    if (my < mag_min[1]) mag_min[1] = my;
    if (my > mag_max[1]) mag_max[1] = my;
    if (mz < mag_min[2]) mag_min[2] = mz;
    if (mz > mag_max[2]) mag_max[2] = mz;

    sample_cnt++;

    // 建议采集至少 800~1500 个点再计算
    if (sample_cnt >= 1200)
    {
        // 计算硬铁偏置（椭圆中心）
        mag_calib.bias[0] = (mag_max[0] + mag_min[0]) * 0.5f;
        mag_calib.bias[1] = (mag_max[1] + mag_min[1]) * 0.5f;
        mag_calib.bias[2] = (mag_max[2] + mag_min[2]) * 0.5f;

        // 计算各轴半径
        float radius[3];
        radius[0] = (mag_max[0] - mag_min[0]) * 0.5f;
        radius[1] = (mag_max[1] - mag_min[1]) * 0.5f;
        radius[2] = (mag_max[2] - mag_min[2]) * 0.5f;

        // 取平均半径作为参考
        float avg_radius = (radius[0] + radius[1] + radius[2]) / 3.0f;

        // 计算缩放系数（把各轴拉成接近圆形）
        mag_calib.scale[0] = avg_radius / radius[0];
        mag_calib.scale[1] = avg_radius / radius[1];
        mag_calib.scale[2] = avg_radius / radius[2];

        mag_calib.calibrated = 1;
        sample_cnt = 0;   // 可选：重置，方便多次校准

        // 这里可以打印或保存校准结果
        // printf("Mag Calib Done!\n");
        // printf("Bias: %.2f, %.2f, %.2f\n", mag_calib.bias[0], mag_calib.bias[1], mag_calib.bias[2]);
        // printf("Scale: %.3f, %.3f, %.3f\n", mag_calib.scale[0], mag_calib.scale[1], mag_calib.scale[2]);
    }
}
void Mag_Apply_Calibration(float *mx, float *my, float *mz)
{
    if (!mag_calib.calibrated) return;

    // 先减偏置（硬铁）
    float x = *mx - mag_calib.bias[0];
    float y = *my - mag_calib.bias[1];
    float z = *mz - mag_calib.bias[2];

    // 再乘缩放（简单软铁）
    *mx = x * mag_calib.scale[0];
    *my = y * mag_calib.scale[1];
    *mz = z * mag_calib.scale[2];
}
