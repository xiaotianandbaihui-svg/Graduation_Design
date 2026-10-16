#include "mpu9250_simple.h"
#include <math.h>
#include <stddef.h>
#include "Delay.h"
#include "stm32f10x_gpio.h"

/* ---------- 地址、寄存器和可修改参数 ---------- */
#define MPU_ADDR          0x68u  /* 7 位地址；AD0 接高则改成 0x69，不填 0xD0 */
#define MAG_ADDR          0x0Cu
#define SMPLRT_DIV        0x19u
#define CONFIG            0x1Au
#define GYRO_CONFIG       0x1Bu
#define ACCEL_CONFIG      0x1Cu
#define ACCEL_CONFIG2     0x1Du
#define FIFO_EN           0x23u
#define INT_PIN_CFG       0x37u
#define INT_ENABLE        0x38u
#define ACCEL_XOUT_H      0x3Bu
#define GYRO_XOUT_H       0x43u
#define USER_CTRL         0x6Au
#define PWR_MGMT_1        0x6Bu
#define PWR_MGMT_2        0x6Cu
#define WHO_AM_I          0x75u
#define MAG_WIA           0x00u
#define MAG_ST1           0x02u
#define MAG_HXL           0x03u
#define MAG_CNTL1         0x0Au
#define MAG_CNTL2         0x0Bu
#define MAG_ASAX          0x10u

#define PI_F              3.14159265359f
#define DEG_TO_RAD        (PI_F / 180.0f)
#define GYRO_LSB_PER_DPS   16.4f       /* 对应下面的 ±2000 度/秒设置 */
#define ACCEL_LSB_PER_G    16384.0f    /* 对应下面的 ±2g 设置 */
#define TILT_TAU_S        0.5f        /* 互补滤波示例时间常数，非手册固定值 */
#define YAW_TAU_S         1.0f

/* 标定结果填在这里；默认值不代表已经完成标定。
 * 陀螺偏置：静止时原始读数的平均值。磁场偏置/比例：各磁轴原始坐标下。
 */
static const float gyro_bias[3] = {0.0f, 0.0f, 0.0f};
static const float mag_bias[3]  = {0.0f, 0.0f, 0.0f};
static const float mag_scale[3] = {1.0f, 1.0f, 1.0f};
static float mag_adjust[3];           /* AK8963 出厂灵敏度系数 */
static float quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
static float eint[3];
static uint8_t ready, quat_ready;
static float filtered[3][3]; /* [加速度/角速度/磁场][XYZ]，单位仍是原始LSB */
static uint8_t filter_ready[3];
static float mag_elapsed_ms;

void SDA(uint8_t level)
{
    if (level) GPIO_SetBits(MPU_GPIO, MPU_SDA_PIN);
    else       GPIO_ResetBits(MPU_GPIO, MPU_SDA_PIN);
}

void SCK(uint8_t level)
{
    if (level) GPIO_SetBits(MPU_GPIO, MPU_SCL_PIN);
    else       GPIO_ResetBits(MPU_GPIO, MPU_SCL_PIN);
}

uint8_t SDA_Read(void)
{
    return (GPIO_ReadInputDataBit(MPU_GPIO, MPU_SDA_PIN) != Bit_RESET) ? 1u : 0u;
}

static void ResetAttitude(void)
{
    quat[0] = 1.0f;
    quat[1] = quat[2] = quat[3] = 0.0f;
    eint[0] = eint[1] = eint[2] = 0.0f;
    quat_ready = 0;
    filter_ready[0] = filter_ready[1] = filter_ready[2] = 0;
    mag_elapsed_ms = 0.0f;
}

/* 一阶RC低通的后向欧拉离散式。首个样本直接采用，避免从0缓慢爬升。 */
static void LowPass3(uint8_t group, const int16_t raw[3], float dt_ms, float hz)
{
    float alpha = hz > 0.0f ? dt_ms / (dt_ms + 1000.0f/(2.0f*PI_F*hz)) : 1.0f;
    uint8_t i;
    for (i = 0; i < 3; ++i) {
        if (!filter_ready[group]) filtered[group][i] = raw[i];
        else filtered[group][i] += alpha * (raw[i] - filtered[group][i]);
    }
    filter_ready[group] = 1;
}

/* ---------- 软件 IIC 完整实现：都是内部函数 ---------- */
static void IIC_Delay(void)
{
    Delay_us(3);                     /* 每个半周期至少3us，实际速率还受GPIO调用影响 */
}

static uint8_t IIC_Start(void)
{
    SDA(1); IIC_Delay();
    SCK(1); IIC_Delay();
    if (!SDA_Read()) return 0;        /* SDA 未释放，总线忙 */
    SDA(0); IIC_Delay();             /* SCL 高时，SDA 下降沿 = START */
    SCK(0);
    return 1;
}

static void IIC_Stop(void)
{
    SCK(0); SDA(0); IIC_Delay();
    SCK(1); IIC_Delay();
    SDA(1); IIC_Delay();             /* SCL 高时，SDA 上升沿 = STOP */
}

static uint8_t IIC_SendByte(uint8_t value)
{
    uint8_t i, ack;
    for (i = 0; i < 8; ++i) {
        SDA((value & 0x80u) != 0);   /* 最高位先发，进入本函数时 SCL 为低 */
        IIC_Delay(); SCK(1); IIC_Delay(); SCK(0);
        value = (uint8_t)(value << 1);
    }
    SDA(1); IIC_Delay();             /* 第 9 个时钟，让从机给 ACK */
    SCK(1); IIC_Delay();
    ack = (uint8_t)!SDA_Read();      /* 低电平 ACK 表示成功 */
    SCK(0);
    return ack;
}

static uint8_t IIC_ReadByte(uint8_t ack)
{
    uint8_t i, value = 0;
    SDA(1);                         /* 释放 SDA，由从机输出数据 */
    for (i = 0; i < 8; ++i) {
        IIC_Delay(); SCK(1); IIC_Delay();
        value = (uint8_t)((value << 1) | (SDA_Read() != 0));
        SCK(0);
    }
    SDA(ack ? 0 : 1);               /* 继续读：ACK；最后一个字节：NACK */
    IIC_Delay(); SCK(1); IIC_Delay(); SCK(0);
    SDA(1);
    return value;
}

static uint8_t WriteReg(uint8_t addr, uint8_t reg, uint8_t value)
{
    uint8_t ok;
    if (!IIC_Start()) return 0;
    ok = IIC_SendByte((uint8_t)(addr << 1)) &&
         IIC_SendByte(reg) && IIC_SendByte(value);
    IIC_Stop();                     /* 任意一字节未应答，也会执行 STOP */
    return ok;
}

static uint8_t ReadRegs(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t i, ok;
    if (!IIC_Start()) return 0;
    ok = IIC_SendByte((uint8_t)(addr << 1)) && IIC_SendByte(reg) &&
         IIC_Start() && IIC_SendByte((uint8_t)((addr << 1) | 1u));
    if (ok) {
        for (i = 0; i < len; ++i) data[i] = IIC_ReadByte(i + 1u < len);
    }
    IIC_Stop();
    return ok;
}

static int16_t Signed16(uint8_t high, uint8_t low)
{
    int32_t value = ((int32_t)high << 8) | low;
    if (value >= 32768) value -= 65536;
    return (int16_t)value;
}

/* ---------- 接口 1：一次初始化加速度计、陀螺仪和磁力计 ---------- */
__weak uint8_t MPU9250_Init(void)
{
    uint8_t id, asa[3], i;
    ready = 0;
	/*--------------------------------------------------------------------*/
	/*------------------------先把硬件给我配了-----------------------------*/
	/*--------------------------------------------------------------------*/
		GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    /* 先把输出寄存器写 1，再切成输出，避免切换瞬间把总线或 CS 拉低 */
    GPIO_SetBits(MPU_GPIO, MPU_SCL_PIN | MPU_SDA_PIN | MPU_CS_PIN);

    gpio.GPIO_Pin   = MPU_SCL_PIN | MPU_SDA_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_Out_OD;      /* 开漏：0 拉低，1 释放 */
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(MPU_GPIO, &gpio);

    gpio.GPIO_Pin   = MPU_CS_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;      /* CS 恒为高，选择 IIC 模式 */
    GPIO_Init(MPU_GPIO, &gpio);
	/*--------------------------------------------------------------------*/
    ResetAttitude();
    SDA(1); SCK(1); Delay_ms(100);
		int8_t a=ReadRegs(MPU_ADDR, WHO_AM_I, &id, 1);
    if (!a|| (id != 0x71 && id != 0x70)) return 0;

    if (!WriteReg(MPU_ADDR, PWR_MGMT_1, 0x80)) return 0; /* 复位 */
    Delay_ms(100);
    if (!WriteReg(MPU_ADDR, PWR_MGMT_1, 0x01)) return 0; /* 唤醒、选时钟 */
    Delay_ms(100);
    if (!WriteReg(MPU_ADDR, PWR_MGMT_2,  0x00)) return 0; /* 打开所有轴 */
    if (!WriteReg(MPU_ADDR, USER_CTRL,   0x00)) return 0; /* 关闭内部 IIC 主机 */
    if (!WriteReg(MPU_ADDR, FIFO_EN,     0x00)) return 0;
    if (!WriteReg(MPU_ADDR, CONFIG,      0x03)) return 0; /* 陀螺低通约 41Hz */
    if (!WriteReg(MPU_ADDR, GYRO_CONFIG, 0x18)) return 0; /* ±2000 度/秒 */
    if (!WriteReg(MPU_ADDR, ACCEL_CONFIG,0x00)) return 0; /* ±2g */
    if (!WriteReg(MPU_ADDR, ACCEL_CONFIG2,0x03)) return 0;/* 加速度低通约 40Hz */
    if (!WriteReg(MPU_ADDR, SMPLRT_DIV,  0x04)) return 0; /* 内部采样 200Hz */
    if (!WriteReg(MPU_ADDR, INT_ENABLE,  0x00)) return 0; /* 不使用中断引脚 */
    if (!WriteReg(MPU_ADDR, INT_PIN_CFG, 0x02)) return 0; /* 磁力计旁路访问 */
		int8_t b=ReadRegs(MAG_ADDR, MAG_WIA, &id, 1);
    if (!b || id != 0x48) return 0;
    if (!WriteReg(MAG_ADDR, MAG_CNTL2, 0x01)) return 0;   /* 磁力计复位 */
    Delay_ms(1);
    if (!WriteReg(MAG_ADDR, MAG_CNTL1, 0x00)) return 0;
    Delay_ms(1);
    if (!WriteReg(MAG_ADDR, MAG_CNTL1, 0x0F)) return 0;   /* 读出厂校准数据 */
    Delay_ms(1);
    if (!ReadRegs(MAG_ADDR, MAG_ASAX, asa, 3)) return 0;
    for (i = 0; i < 3; ++i) mag_adjust[i] = (asa[i] - 128) / 256.0f + 1.0f;
    if (!WriteReg(MAG_ADDR, MAG_CNTL1, 0x00)) return 0;
    Delay_ms(1);
    if (!WriteReg(MAG_ADDR, MAG_CNTL1, 0x16)) return 0;   /* 16 位，连续 100Hz */
    Delay_ms(100);
    ready = 1;
    return 1;
}

static uint8_t ReadXYZ(uint8_t reg, int16_t xyz[3])
{
    uint8_t data[6], i;
    if (!ready || xyz == NULL || !ReadRegs(MPU_ADDR, reg, data, 6)) return 0;
    for (i = 0; i < 3; ++i) xyz[i] = Signed16(data[2*i], data[2*i+1]);
    return 1;
}

/* ---------- 接口 2、3：原始加速度、原始角速度，均为高字节在前 ---------- */
uint8_t MPU9250_ReadAccel(int16_t xyz[3]) { return ReadXYZ(ACCEL_XOUT_H, xyz); }
uint8_t MPU9250_ReadGyro(int16_t xyz[3])  { return ReadXYZ(GYRO_XOUT_H, xyz); }

/* ---------- 接口 4：原始磁场，低字节在前，必须读到 ST2 ---------- */
uint8_t MPU9250_ReadMag(int16_t xyz[3])
{
    uint8_t st1, data[7], i;
    if (!ready || xyz == NULL) return 0;
    if (!ReadRegs(MAG_ADDR, MAG_ST1, &st1, 1) || !(st1 & 0x01u)) return 0;
    if (!ReadRegs(MAG_ADDR, MAG_HXL, data, 7) || (data[6] & 0x08u)) return 0;
    for (i = 0; i < 3; ++i) xyz[i] = Signed16(data[2*i+1], data[2*i]);
    return 1;                       /* 不换单位、不调换 XYZ，保留原始读数 */
}

uint8_t MPU9250_CalcAngles(const int16_t accel[3], const int16_t gyro[3],
                           const int16_t mag[3], float dt_ms, float angle_deg[3])
{
    const float kp_tilt   = 1.0f / TILT_TAU_S;   /* 1/s */
    const float kp_yaw    = 1.0f / YAW_TAU_S;    /* 1/s */
    const float ki_gain   = 0.02f;               /* 1/s^2, 置0关闭积分 */
    const float ki_limit  = 0.05f;               /* rad/s, 陀螺零偏估计上限 */
    const float dt_max_ms = 100.0f;              /* 超过此间隔则丢弃历史姿态 */
    const float mag_h_min = 0.1f;                /* 归一化后水平分量下限 */

    float ax, ay, az, nr, norm, m[3], dt_s;
    float mx = 0.0f, my = 0.0f, mz = 0.0f;
    float gx, gy, gz;
    float q0, q1, q2, q3, vx, vy, vz;
    float eax = 0.0f, eay = 0.0f, eaz = 0.0f;   /* 加速度误差 */
    float emx = 0.0f, emy = 0.0f, emz = 0.0f;   /* 磁力计误差(仅绕竖直轴) */
    float wn, half, c, s, d0, d1, d2, d3;
    float r, p, y, sr, cr, sp, cp;
    float hx, hy, heading_error, mag_integral = 0.0f;
    float hsr, hcr, hsp, hcp, hsy, hcy;
    uint8_t i, accel_ok, mag_ok = 0;

    if (!ready || accel == NULL || gyro == NULL || angle_deg == NULL) return 0;
    if (!isfinite(dt_ms) || dt_ms < 0.0f ||
        (quat_ready && dt_ms == 0.0f)) return 0;
    if (quat_ready && dt_ms > dt_max_ms) {
        ResetAttitude();
        return 0; /* 本帧不输出；下一帧根据当时的重力/磁场重新建立姿态 */
    }
    dt_s = dt_ms * 0.001f; /* 对外统一毫秒，积分内部使用秒 */
    LowPass3(1, gyro, dt_ms, MPU9250_GYRO_LPF_HZ);
    /* 磁力计只有新数据才更新滤波，间隔累计至该次新样本；上限避免无限累积。 */
    mag_elapsed_ms = fminf(mag_elapsed_ms + dt_ms, 1000.0f);

    /* ---- 加速度计 ---- */
    ax = accel[0]; ay = accel[1]; az = accel[2];
    nr = sqrtf(ax*ax + ay*ay + az*az);
    accel_ok = (nr / ACCEL_LSB_PER_G > 0.8f) && (nr / ACCEL_LSB_PER_G < 1.2f);
    if (accel_ok) {
        LowPass3(0, accel, dt_ms, MPU9250_ACCEL_LPF_HZ);
        ax = filtered[0][0]; ay = filtered[0][1]; az = filtered[0][2];
        nr = sqrtf(ax*ax + ay*ay + az*az);
        accel_ok = nr > 0.8f*ACCEL_LSB_PER_G && nr < 1.2f*ACCEL_LSB_PER_G;
        if (accel_ok) { ax /= nr; ay /= nr; az /= nr; }
    } else {
        /* 先检查原始模长；冲击/自由落体不进入重力低通，恢复时重新起滤。 */
        filter_ready[0] = 0;
    }

    /* ---- 磁力计:变到加速度坐标 (My, Mx, -Mz) 并归一化 ---- */
    if (mag != NULL && (mag[0] != 0 || mag[1] != 0 || mag[2] != 0)) {
        if (mag_elapsed_ms >= 1000.0f) filter_ready[2] = 0;
        LowPass3(2, mag, mag_elapsed_ms, MPU9250_MAG_LPF_HZ);
        mag_elapsed_ms = 0.0f;
        for (i = 0; i < 3; ++i)
            m[i] = (filtered[2][i] - mag_bias[i]) * mag_adjust[i] * mag_scale[i];
        mx = m[1]; my = m[0]; mz = -m[2];
        norm = sqrtf(mx*mx + my*my + mz*mz);
        if (isfinite(norm) && norm > 1.0e-6f) {
            mx /= norm; my /= norm; mz /= norm;
            mag_ok = 1;
        }
    }

    if (!quat_ready) {
        /* ---- 首次或超时恢复:加速度建立倾角，磁场可选 ---- */
        if (!accel_ok) return 0;
        r = atan2f(ay, az);
        p = atan2f(-ax, sqrtf(ay*ay + az*az));
        sr = sinf(r); cr = cosf(r); sp = sinf(p); cp = cosf(p);
        y = 0.0f;
        if (mag_ok) {
            hx = mx*cp + my*sr*sp + mz*cr*sp;
            hy = my*cr - mz*sr;
            if (hx*hx + hy*hy >= mag_h_min*mag_h_min)
                y = atan2f(-hy, hx);
        }

        hsr = sinf(0.5f*r); hcr = cosf(0.5f*r);
        hsp = sinf(0.5f*p); hcp = cosf(0.5f*p);
        hsy = sinf(0.5f*y); hcy = cosf(0.5f*y);
        quat[0] = hcr*hcp*hcy + hsr*hsp*hsy;
        quat[1] = hsr*hcp*hcy - hcr*hsp*hsy;
        quat[2] = hcr*hsp*hcy + hsr*hcp*hsy;
        quat[3] = hcr*hcp*hsy - hsr*hsp*hcy;
        eint[0] = eint[1] = eint[2] = 0.0f;
        quat_ready = 1;
    } else {
        q0 = quat[0]; q1 = quat[1]; q2 = quat[2]; q3 = quat[3];

        /* 估计重力方向(旋转矩阵第三行,单位向量) */
        vx = 2.0f * (q1*q3 - q0*q2);
        vy = 2.0f * (q0*q1 + q2*q3);
        vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

        /* 加速度误差 a × v;不可信时置0,陀螺照常积分 */
        if (accel_ok) {
            eax = ay*vz - az*vy;
            eay = az*vx - ax*vz;
            eaz = ax*vy - ay*vx;
        }

        /* 磁场旋转到世界坐标，水平朝向误差只绕世界竖直轴修正。
         * atan2保留完整角差，避免首次无磁场且后来相差180度时叉积为0。 */
        if (mag_ok) {
            hx = 2.0f*(mx*(0.5f - q2*q2 - q3*q3) + my*(q1*q2 - q0*q3) + mz*(q1*q3 + q0*q2));
            hy = 2.0f*(mx*(q1*q2 + q0*q3) + my*(0.5f - q1*q1 - q3*q3) + mz*(q2*q3 - q0*q1));
            if (hx*hx + hy*hy >= mag_h_min*mag_h_min) {
                heading_error = -atan2f(hy, hx);
                emx = heading_error*vx;
                emy = heading_error*vy;
                emz = heading_error*vz;
                /* 大角差先用P项找航向，不把初始朝向误差累积成陀螺零偏。 */
                if (fabsf(heading_error) < 10.0f*DEG_TO_RAD) mag_integral = 1.0f;
            }
        }

        /* ---- 陀螺 (rad/s) ---- */
        gx = (filtered[1][0] - gyro_bias[0]) / GYRO_LSB_PER_DPS * DEG_TO_RAD;
        gy = (filtered[1][1] - gyro_bias[1]) / GYRO_LSB_PER_DPS * DEG_TO_RAD;
        gz = (filtered[1][2] - gyro_bias[2]) / GYRO_LSB_PER_DPS * DEG_TO_RAD;

        /* 积分项(带限幅) */
        eint[0] += ki_gain * (eax + mag_integral*emx) * dt_s;
        eint[1] += ki_gain * (eay + mag_integral*emy) * dt_s;
        eint[2] += ki_gain * (eaz + mag_integral*emz) * dt_s;
        for (i = 0; i < 3; ++i) {
            if (eint[i] >  ki_limit) eint[i] =  ki_limit;
            if (eint[i] < -ki_limit) eint[i] = -ki_limit;
        }

        gx += kp_tilt*eax + kp_yaw*emx + eint[0];
        gy += kp_tilt*eay + kp_yaw*emy + eint[1];
        gz += kp_tilt*eaz + kp_yaw*emz + eint[2];

        /* ---- 四元数积分:q <- q (x) dq,dq 为本周期旋转增量(闭式,保范数) ---- */
        wn = sqrtf(gx*gx + gy*gy + gz*gz);
        half = 0.5f * wn * dt_s;
        if (wn > 1.0e-6f) { s = sinf(half) / wn; c = cosf(half); }
        else              { s = 0.5f * dt_s;     c = 1.0f; }
        d0 = c; d1 = s*gx; d2 = s*gy; d3 = s*gz;

        quat[0] = q0*d0 - q1*d1 - q2*d2 - q3*d3;
        quat[1] = q0*d1 + q1*d0 + q2*d3 - q3*d2;
        quat[2] = q0*d2 - q1*d3 + q2*d0 + q3*d1;
        quat[3] = q0*d3 + q1*d2 - q2*d1 + q3*d0;

        norm = sqrtf(quat[0]*quat[0] + quat[1]*quat[1] + quat[2]*quat[2] + quat[3]*quat[3]);
        if (!isfinite(norm) || norm < 1.0e-6f) { ResetAttitude(); return 0; }
        for (i = 0; i < 4; ++i) quat[i] /= norm;
    }

    /* ---- 四元数 -> ZYX欧拉角，输出[Pitch, Roll, Yaw]，单位度 ---- */
    q0 = quat[0]; q1 = quat[1]; q2 = quat[2]; q3 = quat[3];
    s = 2.0f * (q0*q2 - q1*q3);
    if (s >  1.0f) s =  1.0f;
    if (s < -1.0f) s = -1.0f;
    angle_deg[1] = atan2f(2.0f*(q0*q1 + q2*q3), q0*q0 - q1*q1 - q2*q2 + q3*q3) / DEG_TO_RAD;
    angle_deg[0] = asinf(s) / DEG_TO_RAD;
    angle_deg[2] = atan2f(2.0f*(q0*q3 + q1*q2), q0*q0 + q1*q1 - q2*q2 - q3*q3) / DEG_TO_RAD;
    return 1;
}
