#include "stm32f10x.h"
#include "Serial.h"
#include "mpu6500_spi.h"
#include <stdio.h>
#include "can_pitch.h"
#include "zf_device_imu963ra.h"
#include "IMU.h"
#include "leg_ik.h"
#include "servo.h"
#include "Delay.h"
#include "balance_pid.h"
#include "jump.h"

#define IMU_PERIOD_MS  20u       /* 定时通知主循环；需要20ms就改为20u */
#if IMU_PERIOD_MS < 1u || IMU_PERIOD_MS > 50u
#error IMU_PERIOD_MS_must_be_1_to_50
#endif

/*转化前的原始数据*/
extern int16_t imu963ra_gyro_x, imu963ra_gyro_y, imu963ra_gyro_z ;       // 三轴陀螺仪数据      GYRO (陀螺仪)
extern int16_t imu963ra_acc_x,  imu963ra_acc_y,  imu963ra_acc_z;        // 三轴加速度计数据     ACC  (accelerometer 加速度计)
extern int16_t imu963ra_mag_x,  imu963ra_mag_y,  imu963ra_mag_z;        // 三轴磁力计数据      MAG  (magnetometer 磁力计)

/*滤波后的的原始数据*/
 int16_t imu963ra_gyro_x_out, imu963ra_gyro_y_out, imu963ra_gyro_z_out ;       // 三轴陀螺仪数据      GYRO (陀螺仪)
 int16_t imu963ra_acc_x_out,  imu963ra_acc_y_out,  imu963ra_acc_z_out;        // 三轴加速度计数据     ACC  (accelerometer 加速度计)
 int16_t imu963ra_mag_x_out,  imu963ra_mag_y_out,  imu963ra_mag_z_out;        // 三轴磁力计数据      MAG  (magnetometer 磁力计)
uint8_t flag_filter_run=0;//滤波器首次启动，用于去除缓拟合的
float alpha_for_acc=0.45,alpha_for_gyro=0.4,alpha_for_mag=0.45;
/*转化后的数据*/
float imu963ra_gyro_x_transition, imu963ra_gyro_y_transition, imu963ra_gyro_z_transition;
float imu963ra_acc_x_transition,  imu963ra_acc_y_transition,  imu963ra_acc_z_transition;
float imu963ra_mag_x_transition,  imu963ra_mag_y_transition,  imu963ra_mag_z_transition;

extern volatile uint8_t if_We_Should_Infrom_Main_Function_to_PID;/*定时器触发发送消息到驱动后，驱动立刻回传轮子转速数值，之后进入主控进行PID得到对应的目标Pitch，再把目标角度回传回去，给驱动做PID*/
extern volatile uint32_t CAN_Time_Out_Counter;/*CAN超时计数器*/

#define Formal 1
#define Central_Driver 1


#if Central_Driver
float angle[3];                      /* Pitch、Yaw、Roll，度；仅主循环写 */
volatile float Speed[2]={0};//双轮转速,单位是：rpm
/* TIM2累计毫秒，可自然回绕；跳跃按无符号差值计时，不清零这个公共时钟。 */
static volatile uint32_t control_time_ms;
volatile uint8_t flag_to_update;/*到时间就发送角度*/
/* 初始目标采用合法工作区内的宏；首次下发成功后表示最近一次指令位置。 */
 float current_leg_height[2]={FOOT_INITIAL_Y_MM, FOOT_INITIAL_Y_MM};/*足端Y位置初始化*/
 float current_leg_x[2]={FOOT_INITIAL_X_MM, FOOT_INITIAL_X_MM};/*足端X位置初始化*/
 float current_target_angle_pitch_yaw[2]={0};/*当前目标角度，第一个是pitch 第二个是yaw*/
extern LegIK_Geometry My_LegIK_Geometry;
extern LegIK_Angles Left_LegIK_Angles ;
extern LegIK_Angles Right_LegIK_Angles;
PID Speed_PID;/*给电机驱动板的PID，根据目标速度和当前速度求出需要的倾角pitch*/
FootPID Foot_PID;/*足端位置的PID*/
volatile float target_Speed=0.0f;/*目标速度，单位是：rpm*/
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
    TIM_Cmd(TIM2, DISABLE);
}

void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET) {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        control_time_ms += IMU_PERIOD_MS;
        flag_to_update = 1;
		CAN_Time_Out_Counter++;//计数自增
    }
}

static void Control_Fault(const char *message)
{
    Robot_State = ROBOT_FAULT;
    /* 停止发送控制数据；驱动板由自己的CAN超时路径停止PWM。 */
    while (1) {
        printf("%s\n", message);
        Delay_ms(500);
    }
}

#if (Formal==1)  /*正式函数*/
int main(void)
{
    Servo_Init(SERVO_PERIOD_US);
    Leg_ResetFootPositionHistory();/*首次设置腿高，不限制范围*/
    Serial_Init();
    if (!CAN_Init_Together()) Control_Fault("CAN init failed");
    while (imu963ra_init()) {}
    /* 仍由你在调试启动处开启TIM2；时间基准和CAN超时均依赖它。 */
    IMU_TimerInit();
    SpeedPID_Init(&Speed_PID);
    FootPID_Init(&Foot_PID);
    if (Leg_SetFootPositions(current_leg_x, current_leg_height) != LEG_IK_OK)
        Control_Fault("Initial foot position rejected");

    while (1) {
        if (CAN_Time_Out_Counter > 3) Control_Fault("CAN Time Out");
        if (flag_to_update) {
            float wheels[2], next_x[2], next_y[2], accel_g;
            uint32_t now_ms = control_time_ms;
            LegIK_Result foot_result;
            flag_to_update = 0;


            /*现在开始，更新角度*/
            imu963ra_get_acc();
            imu963ra_get_gyro();
            imu963ra_get_mag();
            if (imu963ra_spi_error) Control_Fault("IMU SPI error");/*imu963ra_spi_error只有把SPI初始化后被设置为0*/
            imu963ra_acc_x_transition = imu963ra_acc_transition(imu963ra_acc_x);
            imu963ra_acc_y_transition = imu963ra_acc_transition(imu963ra_acc_y);
            imu963ra_acc_z_transition = imu963ra_acc_transition(imu963ra_acc_z);
            imu963ra_gyro_x_transition = imu963ra_gyro_transition(imu963ra_gyro_x);
            imu963ra_gyro_y_transition = imu963ra_gyro_transition(imu963ra_gyro_y);
            imu963ra_gyro_z_transition = imu963ra_gyro_transition(imu963ra_gyro_z);
            imu963ra_mag_x_transition = imu963ra_mag_transition(imu963ra_mag_x);
            imu963ra_mag_y_transition = imu963ra_mag_transition(imu963ra_mag_y);
            imu963ra_mag_z_transition = imu963ra_mag_transition(imu963ra_mag_z);
            if (!IMU_9Axis_Gyro_Acc_Mag(
                    imu963ra_acc_x_transition, imu963ra_acc_y_transition, imu963ra_acc_z_transition,
                    imu963ra_gyro_x_transition, imu963ra_gyro_y_transition, imu963ra_gyro_z_transition,
                    imu963ra_mag_x_transition, imu963ra_mag_y_transition, imu963ra_mag_z_transition,
                    &angle[2], &angle[0], &angle[1], IMU_PERIOD_MS * 0.001f))
                Control_Fault("IMU attitude invalid");
            /*角度更新完成*/

            
            if (Jump_Request) {
                Jump_Request = 0; /* 请求只消费一次，跳跃中不重复启动。 */
                if (Robot_State == ROBOT_NORMAL && Jump_Start(now_ms) == JUMP_RUNNING) {
                    target_Speed = 0.0f;
                    PID_Reset(&Speed_PID);
                    PID_Reset(&Foot_PID.roll);
                    PID_Reset(&Foot_PID.x[0]);
                    PID_Reset(&Foot_PID.x[1]);
                    current_target_angle_pitch_yaw[0] = 0.0f;
                    current_target_angle_pitch_yaw[1] = angle[1];
                } else {
                    printf("Jump request rejected\n");
                }
            }
            if (Robot_State == ROBOT_FAULT) Control_Fault("Jump command failed");
            if (Robot_State == ROBOT_JUMPING) {
                accel_g = sqrtf(imu963ra_acc_x_transition * imu963ra_acc_x_transition +
                                imu963ra_acc_y_transition * imu963ra_acc_y_transition +
                                imu963ra_acc_z_transition * imu963ra_acc_z_transition);
                int8_t jump_result = Jump_Update(now_ms, accel_g);
                if (jump_result == JUMP_ERROR) Control_Fault("Jump failed");
                if (jump_result == JUMP_LANDED) target_Speed = 0.0f;
            }

            /* 姿态/腿高 -> 收轮速 -> 算目标Pitch -> 发目标角度。 */
            while (!CAN_Angle_and_LegHeight_Send_First_Package(
                       angle[0], angle[1], current_leg_height[0], current_leg_height[1])) {
                if (CAN_Time_Out_Counter > 3) Control_Fault("CAN Time Out");
            }
            while (if_We_Should_Infrom_Main_Function_to_PID == 0) {
                if (CAN_Time_Out_Counter > 3) Control_Fault("CAN Time Out");
            }
            /* 一次取齐两轮速度并消费通知，两个PID使用同一帧。 */
            NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
            wheels[0] = Speed[0]; wheels[1] = Speed[1];
            if_We_Should_Infrom_Main_Function_to_PID = 0;
            NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);

            if (Robot_State == ROBOT_NORMAL) {
                if (!SpeedPID_Update(&Speed_PID, target_Speed, wheels,
                                     &current_target_angle_pitch_yaw[0]))
                    Control_Fault("Speed PID invalid");
            } else {
                current_target_angle_pitch_yaw[0] = 0.0f;
            }
            while (!CAN_Angle_and_LegHeight_Send_Second_Package(
                       current_target_angle_pitch_yaw[0], current_target_angle_pitch_yaw[1])) {
                if (CAN_Time_Out_Counter > 3) Control_Fault("CAN Time Out");
            }

            /* 跳跃独占足端输出；普通模式统一由足端PID生成坐标并下发。 */
            if (Robot_State == ROBOT_NORMAL && FootPID_Update(&Foot_PID, angle[2], wheels,
                    current_leg_x, current_leg_height, next_x, next_y)) {
                foot_result = Leg_SetFootPositions(next_x, next_y);
                if (foot_result == LEG_IK_OK) {
                    for (unsigned i = 0; i < 2; ++i) {
                        current_leg_x[i] = next_x[i];
                        current_leg_height[i] = next_y[i];
                    }
                } else {
                    printf("Foot position rejected: %d\n", (int)foot_result);
                }
            }
        }
    }
}
#elif (Formal==2) /*调试用*/
int main(void)
{
	Servo_Init(SERVO_PERIOD_US); /* TIM3: no pulses until Servo_SetPulse/Servo_SetAngle. */
	Serial_Init();/*来个串口，我板子上应该花了接口，先打开*/
	IMU_TimerInit();/*来个定时老弟*/
	Servo_Init(SERVO_PERIOD_US);/*20000us也就是20ms*/
	CAN_Init_Together();/*CAN初始化*/
	while(imu963ra_init()){};/*陀螺仪初始化*/
		printf("OK\n");
		for(int i=0;i<1200;i++)
		{
			imu963ra_get_mag();
			Mag_Calibrate_Collect(imu963ra_mag_x,imu963ra_mag_y,imu963ra_mag_z);
			Delay_ms(10);
		}
    while (1) {
				if(flag_to_update==1)
				{
					flag_to_update=0;
					
					imu963ra_get_acc();
					imu963ra_get_gyro();
					imu963ra_get_mag();
                    if (imu963ra_spi_error) Control_Fault("IMU SPI error");
					if(!flag_filter_run)
					{
						imu963ra_acc_x_out=imu963ra_acc_x;
						imu963ra_acc_y_out=imu963ra_acc_y;
						imu963ra_acc_z_out=imu963ra_acc_z;
						imu963ra_gyro_x_out=imu963ra_gyro_x;
						imu963ra_gyro_y_out=imu963ra_gyro_y;
						imu963ra_gyro_z_out=imu963ra_gyro_z;
						imu963ra_mag_x_out=imu963ra_mag_x;
						imu963ra_mag_y_out=imu963ra_mag_y;
						imu963ra_mag_z_out=imu963ra_mag_z;
						flag_filter_run=1;
					}
					imu963ra_acc_x_out+=alpha_for_acc*(imu963ra_acc_x-imu963ra_acc_x_out);
					imu963ra_acc_y_out+=alpha_for_acc*(imu963ra_acc_y-imu963ra_acc_y_out);
					imu963ra_acc_z_out+=alpha_for_acc*(imu963ra_acc_z-imu963ra_acc_z_out);
					imu963ra_gyro_x_out+=alpha_for_gyro*(imu963ra_gyro_x-imu963ra_gyro_x_out);
					imu963ra_gyro_y_out+=alpha_for_gyro*(imu963ra_gyro_y-imu963ra_gyro_y_out);
					imu963ra_gyro_z_out+=alpha_for_gyro*(imu963ra_gyro_z-imu963ra_gyro_z_out);
					imu963ra_mag_x_out+=alpha_for_mag*(imu963ra_mag_x-imu963ra_mag_x_out);
					imu963ra_mag_y_out+=alpha_for_mag*(imu963ra_mag_y-imu963ra_mag_y_out);
					imu963ra_mag_z_out+=alpha_for_mag*(imu963ra_mag_z-imu963ra_mag_z_out);
				
					
					
					imu963ra_acc_x_transition=imu963ra_acc_transition(imu963ra_acc_x_out);
					imu963ra_acc_y_transition=imu963ra_acc_transition(imu963ra_acc_y_out);
					imu963ra_acc_z_transition=imu963ra_acc_transition(imu963ra_acc_z_out);
					imu963ra_gyro_x_transition=imu963ra_gyro_transition(imu963ra_gyro_x_out);
					imu963ra_gyro_y_transition=imu963ra_gyro_transition(imu963ra_gyro_y_out);
					imu963ra_gyro_z_transition=imu963ra_gyro_transition(imu963ra_gyro_z_out);
					imu963ra_mag_x_transition=imu963ra_mag_transition(imu963ra_mag_x_out);
					imu963ra_mag_y_transition=imu963ra_mag_transition(imu963ra_mag_y_out);
					imu963ra_mag_z_transition=imu963ra_mag_transition(imu963ra_mag_z_out);
					if (!IMU_9Axis_Gyro_Acc_Mag(imu963ra_acc_x_transition, imu963ra_acc_y_transition, imu963ra_acc_z_transition, 
					imu963ra_gyro_x_transition, imu963ra_gyro_y_transition, imu963ra_gyro_z_transition, 
					imu963ra_mag_x_transition, imu963ra_mag_y_transition, imu963ra_mag_z_transition, 
					&angle[2], &angle[0], &angle[1], IMU_PERIOD_MS*0.001f))
                        Control_Fault("IMU attitude invalid");
//					printf("%d,%d,%d,%d,%d,%d\n",imu963ra_gyro_x_out,imu963ra_gyro_y_out,imu963ra_gyro_z_out,imu963ra_gyro_x,imu963ra_gyro_y,imu963ra_gyro_z);				
			
					printf("%f,%f,%f\n",angle[0],angle[1],angle[2]);				
				}

//					Delay_ms(IMU_PERIOD_MS);
//					data=spi_read_write_byte(SPI2,0XA5);
//					printf("%c\n",data);

    }
}
#endif

