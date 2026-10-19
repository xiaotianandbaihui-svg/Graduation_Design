#include "stm32f10x.h"
#include "Serial.h"
#include "mpu6500_spi.h"
#include <stdio.h>
#include "can_pitch.h"
#include "zf_device_imu963ra.h"
#include "IMU.h"
#include "leg_ik.h"
#include "servo.h"
#define IMU_PERIOD_MS  20u       /* 定时通知主循环；需要20ms就改为20u */
#if IMU_PERIOD_MS < 1u || IMU_PERIOD_MS > 50u
#error IMU_PERIOD_MS_must_be_1_to_50
#endif

/*转化前的原始数据*/
extern int16_t imu963ra_gyro_x, imu963ra_gyro_y, imu963ra_gyro_z ;       // 三轴陀螺仪数据      GYRO (陀螺仪)
extern int16_t imu963ra_acc_x,  imu963ra_acc_y,  imu963ra_acc_z;        // 三轴加速度计数据     ACC  (accelerometer 加速度计)
extern int16_t imu963ra_mag_x,  imu963ra_mag_y,  imu963ra_mag_z;        // 三轴磁力计数据      MAG  (magnetometer 磁力计)
/*转化后的数据*/
float imu963ra_gyro_x_transition, imu963ra_gyro_y_transition, imu963ra_gyro_z_transition;
float imu963ra_acc_x_transition,  imu963ra_acc_y_transition,  imu963ra_acc_z_transition;
float imu963ra_mag_x_transition,  imu963ra_mag_y_transition,  imu963ra_mag_z_transition;

volatile uint8_t Its_Time_to_PID=0;/*定时器触发发送消息到驱动后，驱动回传角度数值，之后快速主控进行PID调控*/


#define Central_Driver 1
#if Central_Driver
float angle[3];                      /* Pitch、Roll、Yaw，度；仅主循环写 */
float Speed[2]={0};//双轮转速
volatile uint8_t flag_to_update;/*到时间就发送角度*/
volatile float current_leg_height[2]={0};
volatile float current_leg_x[2]={0};
volatile float current_target_angle_pitch_yaw[2]={0};/*当前目标角度，第一个是pitch 第二个是yaw*/
extern LegIK_Geometry My_LegIK_Geometry;
extern LegIK_Angles Left_LegIK_Angles ;
extern LegIK_Angles Right_LegIK_Angles;
#else
volatile float the_angles[2]={0};//应当依次为Pitch、Yaw角,接收主控传来的角度信息
volatile  float leg_height[2]={0};//标记腿高也就是y坐标的，由上位机发送而来
volatile float target_angle[2]={0};
#endif



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
    timer.TIM_Prescaler = (uint16_t)(tim_hz/1000000u-1u); /* 1us/计数 */
    timer.TIM_Period = IMU_PERIOD_MS*1000u-1u;
    TIM_TimeBaseInit(TIM2, &timer);
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update); /* 清除初始化UG产生的UIF */
    flag_to_update = 0;
    /* Serial_Init已设置分组2，TIM2优先于USART1接收中断。 */
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

int main(void)
{
	Servo_Init(SERVO_PERIOD_US); /* TIM3: no pulses until Servo_SetPulse/Servo_SetAngle. */
	Serial_Init();/*来个串口，我板子上应该花了接口，先打开*/
	IMU_TimerInit();/*来个定时老弟*/
	while(imu963ra_init()){};/*陀螺仪初始化*/
    while (1) {
        if (flag_to_update) {
            flag_to_update = 0; /* 先清零，保留读取/解算期间新到的请求 */
					/*这里发送角度和腿长(腿长用于调整PID参数)，我们的驱动板主要是控制轮子转动实现车的转弯和平衡*/
					/*解算并读取一次角度*/
					imu963ra_get_acc();
					imu963ra_get_gyro();
					imu963ra_get_mag();
					imu963ra_acc_x_transition=imu963ra_acc_transition(imu963ra_acc_x);
					imu963ra_acc_y_transition=imu963ra_acc_transition(imu963ra_acc_y);
					imu963ra_acc_z_transition=imu963ra_acc_transition(imu963ra_acc_z);
					imu963ra_gyro_x_transition=imu963ra_gyro_transition(imu963ra_gyro_x);
					imu963ra_gyro_y_transition=imu963ra_gyro_transition(imu963ra_gyro_y);
					imu963ra_gyro_z_transition=imu963ra_gyro_transition(imu963ra_gyro_z);
					imu963ra_mag_x_transition=imu963ra_mag_transition(imu963ra_mag_x);
					imu963ra_mag_y_transition=imu963ra_mag_transition(imu963ra_mag_y);
					imu963ra_mag_z_transition=imu963ra_mag_transition(imu963ra_mag_z);
					IMU_9Axis_Gyro_Acc_Mag(imu963ra_acc_x_transition, imu963ra_acc_y_transition, imu963ra_acc_z_transition, 
					imu963ra_gyro_x_transition, imu963ra_gyro_y_transition, imu963ra_gyro_z_transition, 
					imu963ra_mag_x_transition, imu963ra_mag_y_transition, imu963ra_mag_z_transition, 
					&angle[2], &angle[0], &angle[1], IMU_PERIOD_MS*0.001);
					
					while(!CAN_Angle_and_LegHeight_Send_First_Package(angle[0],angle[1],current_leg_height[0],current_leg_height[1]));/*发送第一包数据*/
					while(!CAN_Angle_and_LegHeight_Send_Second_Package(current_target_angle_pitch_yaw[0],current_target_angle_pitch_yaw[1]));/*发送第二包数据*/
        }
				
				
				if(Its_Time_to_PID)
				{
					Its_Time_to_PID=0;
					/*根据回传的轮速，立刻进行PID控制，这个是控制足端x方向的*/
					/*此处为PID代码，PID+其他代码在此处填写，得到应当输出的足端位置，emm，很朴素了。。*/
					/*主要的，要根据轮速，控制腿的x坐标变化；其次，通过横滚角控制左右轮腿的高度*/
					/*---------------------------------------------------------------------------*/
					/*---------------------------PID代码至此开始----------------------------------*/
					/*---------------------------------------------------------------------------*/

					
					/*---------------------------------------------------------------------------*/
					/*---------------------------PID代码至此结束----------------------------------*/
					/*---------------------------------------------------------------------------*/
					
					/*左腿足端位置设置*/
					LegIK_Solve(&My_LegIK_Geometry,
                         current_leg_x[0],current_leg_height[0] , &Left_LegIK_Angles);
					Servo_SetAngle(0,Servo_Bias_1-Left_LegIK_Angles.q1_deg);
					Servo_SetAngle(1,Servo_Bias_2-Left_LegIK_Angles.q4_deg);
					
					/*右腿足端位置设置*/
					LegIK_Solve(&My_LegIK_Geometry,
                         current_leg_x[1],current_leg_height[1] , &Right_LegIK_Angles);
					Servo_SetAngle(2,Servo_Bias_3-Left_LegIK_Angles.q1_deg);
					Servo_SetAngle(3,Servo_Bias_4-Left_LegIK_Angles.q4_deg);
					/*这里再次强调，我老是马虎接错线，这里一定要根据注释严格接线*/
				}
    }
}
