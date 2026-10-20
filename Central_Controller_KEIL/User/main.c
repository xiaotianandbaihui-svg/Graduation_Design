#include "stm32f10x.h"
#include "Serial.h"
#include "Delay.h"
#include "can_pitch.h"
#include "zf_device_imu963ra.h"
#include "IMU.h"
#include "leg_ik.h"
#include "servo.h"
#include "balance_pid.h"
#include <math.h>

#define IMU_PERIOD_MS FOOT_PID_PERIOD_MS
#if IMU_PERIOD_MS < 1u || IMU_PERIOD_MS > 50u
#error IMU_PERIOD_MS_must_be_1_to_50
#endif

/* 转换后的惯性传感器数据。 */
float imu963ra_gyro_x_transition, imu963ra_gyro_y_transition, imu963ra_gyro_z_transition;
float imu963ra_acc_x_transition, imu963ra_acc_y_transition, imu963ra_acc_z_transition;
float imu963ra_mag_x_transition, imu963ra_mag_y_transition, imu963ra_mag_z_transition;

volatile uint8_t Its_Time_to_PID = 0; /* CAN中断：一帧左右轮速已更新 */
volatile float Speed[2] = {0.0f, 0.0f};
volatile uint8_t flag_to_update = 0;
float angle[3]; /* [0]=Pitch，[1]=Yaw，[2]=Roll；仅主循环更新 */
/* 下列坐标是成功下发的指令位置，不是实测位置；不能用(0,0)作为初始足端。 */
volatile float current_leg_height[2] = {FOOT_INITIAL_Y_MM, FOOT_INITIAL_Y_MM};
volatile float current_leg_x[2] = {FOOT_INITIAL_X_MM, FOOT_INITIAL_X_MM};
volatile float current_target_angle_pitch_yaw[2] = {0.0f, 0.0f};
/* 轮足的当前状态：保留站立、跳跃、运动的状态定义。 */
enum Global_State
{
    Stand_Stop,
    Jumping,
    Running,
};
FootPID for_foot_balance;
/* 调试观察：0成功；-10计算输入异常；-11 CAN初始化失败；-12 IMU数据无效；
 * -13 CAN邮箱提交失败；-14舵机初始化失败；其余负数见LegIK_Result。 */
volatile int foot_control_status = 0;

static void IMU_TimerInit(void)
{
    RCC_ClocksTypeDef clocks;
    TIM_TimeBaseInitTypeDef timer;
    NVIC_InitTypeDef nvic;
    uint32_t tim_hz;
    RCC_GetClocksFreq(&clocks);
    tim_hz = clocks.PCLK1_Frequency;
    if (clocks.PCLK1_Frequency != clocks.HCLK_Frequency) tim_hz *= 2u;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    TIM_DeInit(TIM2);
    TIM_TimeBaseStructInit(&timer);
    timer.TIM_Prescaler = (uint16_t)(tim_hz / 1000000u - 1u);
    timer.TIM_Period = IMU_PERIOD_MS * 1000u - 1u;
    TIM_TimeBaseInit(TIM2, &timer);
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    flag_to_update = 0;
    /* Serial_Init已设置分组2。 */
    nvic.NVIC_IRQChannel = TIM2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
}

void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET) {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        flag_to_update = 1;
    }
}

static int TakeIMUTick(void)
{
    uint32_t primask = __get_PRIMASK();
    int pending;
    __disable_irq();
    pending = flag_to_update != 0;
    flag_to_update = 0;
    __set_PRIMASK(primask);
    return pending;
}

static int ReadAttitude(void)
{
    imu963ra_get_acc();
    if (imu963ra_spi_error) return 0;
    imu963ra_get_gyro();
    if (imu963ra_spi_error) return 0;
    imu963ra_get_mag();
    if (imu963ra_spi_error) return 0;
    /* 原9轴解算会归一化这两个向量，零向量不能送入。 */
    if ((imu963ra_acc_x == 0 && imu963ra_acc_y == 0 && imu963ra_acc_z == 0) ||
        (imu963ra_mag_x == 0 && imu963ra_mag_y == 0 && imu963ra_mag_z == 0)) return 0;
    imu963ra_acc_x_transition = imu963ra_acc_transition(imu963ra_acc_x);
    imu963ra_acc_y_transition = imu963ra_acc_transition(imu963ra_acc_y);
    imu963ra_acc_z_transition = imu963ra_acc_transition(imu963ra_acc_z);
    imu963ra_gyro_x_transition = imu963ra_gyro_transition(imu963ra_gyro_x);
    imu963ra_gyro_y_transition = imu963ra_gyro_transition(imu963ra_gyro_y);
    imu963ra_gyro_z_transition = imu963ra_gyro_transition(imu963ra_gyro_z);
    imu963ra_mag_x_transition = imu963ra_mag_transition(imu963ra_mag_x);
    imu963ra_mag_y_transition = imu963ra_mag_transition(imu963ra_mag_y);
    imu963ra_mag_z_transition = imu963ra_mag_transition(imu963ra_mag_z);
    IMU_9Axis_Gyro_Acc_Mag(imu963ra_acc_x_transition, imu963ra_acc_y_transition,
        imu963ra_acc_z_transition, imu963ra_gyro_x_transition, imu963ra_gyro_y_transition,
        imu963ra_gyro_z_transition, imu963ra_mag_x_transition, imu963ra_mag_y_transition,
        imu963ra_mag_z_transition, &angle[2], &angle[0], &angle[1], IMU_PERIOD_MS * 0.001f);
    return isfinite(angle[0]) && isfinite(angle[1]) && isfinite(angle[2]);
}

int main(void)
{
    float wheel_rpm[2], old_x[2], old_y[2], next_x[2], next_y[2];
    FootPID proposed;
    LegIK_Result result;
    unsigned i;
    uint8_t control_due = 0;

    Serial_Init();
    FootPID_Init(&for_foot_balance);
    if (Servo_Init(SERVO_PERIOD_US) != SERVO_OK) {
        foot_control_status = -14;
        while (1) {}
    }
    Leg_ResetFootPositionHistory();
    for (i = 0; i < 2; ++i) {
        next_x[i] = current_leg_x[i];
        next_y[i] = current_leg_height[i];
    }
    result = Leg_SetFootPositions(next_x, next_y);
    if (result != LEG_IK_OK) {
        /* 先标定leg_ik.h的四个Servo_Bias；不把非法角度直接截成0/180度。 */
        foot_control_status = result;
        while (1) {}
    }
    Delay_ms(FOOT_INITIAL_SETTLE_MS);
    while (imu963ra_init()) {}
    if (!CAN_Init_Together()) {
        foot_control_status = -11;
        while (1) {}
    }
    IMU_TimerInit();

    while (1) {
        if (TakeIMUTick()) {
            control_due = 0; /* 未收到上周期回包时，不补算多次PID。 */
            if (ReadAttitude()) {
                (void)CAN_TakeWheelSpeed(wheel_rpm); /* 丢弃已缓存的旧通知 */
                /* 先姿态/腿高，后目标角；邮箱忙则跳过本次，避免堵住主循环。 */
                if (CAN_Angle_and_LegHeight_Send_First_Package(angle[0], angle[1],
                        current_leg_height[0], current_leg_height[1]) &&
                    CAN_Angle_and_LegHeight_Send_Second_Package(
                        current_target_angle_pitch_yaw[0], current_target_angle_pitch_yaw[1])) {
                    control_due = 1;
                } else {
                    foot_control_status = -13;
                }
            } else {
                foot_control_status = -12;
            }
        }

        /* 定时器决定节拍，CAN回包触发足端控制；每个节拍至多执行一次。
         * 当前协议没有序号，不能据此声称已严格匹配跨板的同周期报文。 */
        if (control_due && CAN_TakeWheelSpeed(wheel_rpm)) {
            control_due = 0;
            for (i = 0; i < 2; ++i) {
                old_x[i] = current_leg_x[i];
                old_y[i] = current_leg_height[i];
            }
            proposed = for_foot_balance;
            if (!FootPID_Update(&proposed, angle[2], wheel_rpm,
                                old_x, old_y, next_x, next_y)) {
                foot_control_status = -10;
                continue;
            }
            /* Roll和两个独立轮速环全部算完，只从这里统一设置两条腿。 */
            result = Leg_SetFootPositions(next_x, next_y);
            foot_control_status = result;
            if (result == LEG_IK_OK) {
                for_foot_balance = proposed;
                for (i = 0; i < 2; ++i) {
                    current_leg_x[i] = next_x[i];
                    current_leg_height[i] = next_y[i];
                }
            }
        }
    }
}
