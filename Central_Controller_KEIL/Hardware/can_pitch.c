#include "can_pitch.h"
#include "stm32f10x.h"
#include "stm32f10x_can.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "misc.h"
#include <math.h>


#define Central_Driver 1


#if Central_Driver
extern volatile float Speed[2];
#else
extern float the_angles[2]; /* [0]=Pitch, [1]=Yaw */
extern float leg_height[2]; /* [0]=Left_Leg_Height, [1]=Right_Leg_Height */
#endif
extern volatile uint8_t Its_Time_to_PID;

int CAN_Init_Together(void)
{
    RCC_ClocksTypeDef clocks;
    GPIO_InitTypeDef gpio;
    CAN_InitTypeDef can;
    CAN_FilterInitTypeDef filter;
    NVIC_InitTypeDef irq;
    uint32_t prescaler;

    RCC_GetClocksFreq(&clocks);
    /* 500k * (1 + BS1=9 + BS2=2) = 6MHz；CAN直接使用PCLK1，不倍频。 */
    prescaler = clocks.PCLK1_Frequency / 6000000u;
    if (clocks.PCLK1_Frequency % 6000000u ||
        prescaler < 1u || prescaler > 1024u) return 0;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
    NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
    GPIO_PinRemapConfig(GPIO_Remap1_CAN1, DISABLE); /* 默认PA11/PA12 */

    gpio.GPIO_Pin = GPIO_Pin_11;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);
    gpio.GPIO_Pin = GPIO_Pin_12;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio);

    CAN_DeInit(CAN1);
    CAN_StructInit(&can);
    can.CAN_TXFP = ENABLE; /* 按请求顺序发送：先姿态/腿高，再目标角度。 */
    can.CAN_Mode = CAN_Mode_Normal;
    can.CAN_SJW = CAN_SJW_1tq;
    can.CAN_BS1 = CAN_BS1_9tq;
    can.CAN_BS2 = CAN_BS2_2tq;
    can.CAN_Prescaler = (uint16_t)prescaler;
    can.CAN_ABOM = ENABLE; /* 总线关闭后允许硬件恢复 */
    can.CAN_NART = ENABLE; /* 实时角度不自动重发旧帧，下一周期发送最新值，也就是说，现在不自动重传*/
    if (CAN_Init(CAN1, &can) != CAN_InitStatus_Success) return 0;

    filter.CAN_FilterNumber = 0;
    filter.CAN_FilterMode = CAN_FilterMode_IdMask;
    filter.CAN_FilterScale = CAN_FilterScale_32bit;
#if Central_Driver
    filter.CAN_FilterIdHigh = (uint16_t)(CAN_SPEED_ID << 5);
#else
    filter.CAN_FilterIdHigh = (uint16_t)(CAN_ANGLE_LEGHEIGHT_ID << 5);
#endif
    filter.CAN_FilterIdLow = 0;
    filter.CAN_FilterMaskIdHigh = 0xFFE0u; /* 精确匹配11位标准ID */
    filter.CAN_FilterMaskIdLow = 0x0006u; /* IDE=0、RTR=0，只收标准数据帧 */
    filter.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
    filter.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&filter);

    Its_Time_to_PID = 0;
    NVIC_ClearPendingIRQ(USB_LP_CAN1_RX0_IRQn);
    irq.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
    irq.NVIC_IRQChannelPreemptionPriority = 1;
    irq.NVIC_IRQChannelSubPriority = 0;
    irq.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&irq);
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
    return 1;
}
/* 一次取走同一帧的两轮速度，同时清通知，避免中断在复制一半时更新数组。 */
int CAN_TakeWheelSpeed(float rpm[2])
{
#if Central_Driver
    uint32_t primask;
    int fresh;
    if (!rpm) return 0;
    primask = __get_PRIMASK();
    __disable_irq();
    fresh = Its_Time_to_PID != 0;
    if (fresh) {
        rpm[0] = Speed[0];
        rpm[1] = Speed[1];
        Its_Time_to_PID = 0;
    }
    __set_PRIMASK(primask);
    return fresh;
#else
    (void)rpm;
    return 0;
#endif
}

/* 主控发送当前机体角度和两侧腿高。 */
/*return 0:发送失败  1:发送成功*/
int CAN_Angle_and_LegHeight_Send_First_Package(float pitch_deg, float yaw_deg,float left_leg_height,float right_leg_height)
{
    CanTxMsg msg = {0};
    float angles[2] = {pitch_deg, yaw_deg};
		float leg_height[2]={left_leg_height,right_leg_height};
    float scaled;
    uint16_t raw;
    uint8_t i;

    for (i = 0; i < 2; ++i) {
        if (!isfinite(angles[i]) || angles[i] < -327.68f ||
            angles[i] > 327.67f) return 0;
        scaled = angles[i] * 100.0f;
        raw = (uint16_t)(int32_t)(scaled + (scaled >= 0 ? 0.5f : -0.5f));
        msg.Data[2*i] = (uint8_t)raw;
        msg.Data[2*i+1] = (uint8_t)(raw >> 8);
    }
		
    for (i = 0; i < 2; ++i) {
			  if (!isfinite(leg_height[i]) || leg_height[i] < -327.68f ||
					leg_height[i] > 327.67f) return 0;//这么做的一个主要原因还是保障强转的正确，不会导致范围不匹配
        scaled = leg_height[i] * 100.0f;
        raw = (uint16_t)(int32_t)(scaled + (scaled >= 0 ? 0.5f : -0.5f));
        msg.Data[2*(i+2)] = (uint8_t)raw;
        msg.Data[2*(i+2)+1] = (uint8_t)(raw >> 8);
    }
		
		
    msg.StdId = CAN_ANGLE_LEGHEIGHT_ID;
    msg.IDE = CAN_Id_Standard;
    msg.RTR = CAN_RTR_Data;
    msg.DLC = 8;
    return CAN_Transmit(CAN1, &msg) != CAN_TxStatus_NoMailBox;
}


int CAN_Angle_and_LegHeight_Send_Second_Package(float target_pitch_deg, float target_yaw_deg)
{
    CanTxMsg msg = {0};
    float angles[2] = {target_pitch_deg, target_yaw_deg};
    float scaled;
    uint16_t raw;
    uint8_t i;

    for (i = 0; i < 2; ++i) {
        if (!isfinite(angles[i]) || angles[i] < -327.68f ||
            angles[i] > 327.67f) return 0;
        scaled = angles[i] * 100.0f;
        raw = (uint16_t)(int32_t)(scaled + (scaled >= 0 ? 0.5f : -0.5f));
        msg.Data[2*i] = (uint8_t)raw;
        msg.Data[2*i+1] = (uint8_t)(raw >> 8);
    }
				
    msg.StdId = CAN_TARGET_ANGLE_ID;
    msg.IDE = CAN_Id_Standard;
    msg.RTR = CAN_RTR_Data;
    msg.DLC = 4;
    return CAN_Transmit(CAN1, &msg) != CAN_TxStatus_NoMailBox;
}


/* 电驱端回传轮速的复用接口；主控接收后据此控制足端x。 */
int CAN_Send_The_Wheels_Speed(float* left_rpm ,float* right_rpm)
{
	  CanTxMsg msg = {0};
		msg.StdId =CAN_SPEED_ID;
		msg.IDE = CAN_Id_Standard;
    msg.RTR = CAN_RTR_Data;
    msg.DLC = 4;
		uint16_t raw;
		float scaled;
		scaled = (*left_rpm) * 100.0f;
		raw = (uint16_t)(int32_t)(scaled + (scaled >= 0 ? 0.5f : -0.5f));
		msg.Data[0] = (uint8_t)raw;
		msg.Data[1] = (uint8_t)(raw >> 8);
		scaled = (*right_rpm) * 100.0f;
		raw = (uint16_t)(int32_t)(scaled + (scaled >= 0 ? 0.5f : -0.5f));
		msg.Data[2] = (uint8_t)raw;
		msg.Data[3] = (uint8_t)(raw >> 8);
    return CAN_Transmit(CAN1, &msg) != CAN_TxStatus_NoMailBox;
}

static uint8_t if_We_Should_Infrom_Main_Function_to_PID=0;/*通知确认PID*/
/*接收数据中断，*/
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    CanRxMsg msg;
    int32_t raw;
    uint8_t i;
    uint8_t count = CAN_MessagePending(CAN1, CAN_FIFO0);
    /* FIFO最多3帧：本次处理入口时已有的帧，新到的留给下一次中断。 */
    while (count--) {
        CAN_Receive(CAN1, CAN_FIFO0, &msg); /* 读取并释放FIFO项 */
			#if Central_Driver
				if(msg.StdId == CAN_SPEED_ID && msg.IDE == CAN_Id_Standard &&
            msg.RTR == CAN_RTR_Data && msg.DLC == 4)
				{
						for (i = 0; i < 2; ++i) {
								raw = (int32_t)((uint16_t)msg.Data[2*i] |
															 ((uint16_t)msg.Data[2*i+1] << 8));
								if (raw & 0x8000)/*这里因为是把uint16_t转换为int32_t ，高位自动补的都是0*/
										 raw |= 0xFFFF0000u;  /* 只把高16位置1，保留低16位 *//* 还原16位有符号补码 */
								Speed[i] = (float)raw / 100.0f;
						}
					if_We_Should_Infrom_Main_Function_to_PID|=0x01;
				}
				#else 
        if (msg.StdId == CAN_ANGLE_LEGHEIGHT_ID && msg.IDE == CAN_Id_Standard &&
            msg.RTR == CAN_RTR_Data && msg.DLC == 8)
				{
						/* Pitch、Yaw角度更新到缓存区 */
						for (i = 0; i < 2; ++i) {
								raw = (int32_t)((uint16_t)msg.Data[2*i] |
															 ((uint16_t)msg.Data[2*i+1] << 8));
								if (raw & 0x8000)/*这里因为是把uint16_t转换为int32_t ，高位自动补的都是0*/
										 raw |= 0xFFFF0000u;  /* 只把高16位置1，保留低16位 *//* 还原16位有符号补码 */
								the_angles[i] = (float)raw / 100.0f;
						}
						
						
						/* 更新左右两边的腿高 */
						 for (i = 0; i < 2; ++i) {
								raw = (int32_t)((uint16_t)msg.Data[2*(i+2)] |
															 ((uint16_t)msg.Data[2*(i+2)+1] << 8));
								if (raw & 0x8000)/*这里因为是把uint16_t转换为int32_t ，高位自动补的都是0*/
										 raw |= 0xFFFF0000u;  /* 只把高16位置1，保留低16位 *//* 还原16位有符号补码 */
								leg_height[i] = (float)raw / 100.0f;
						}
						 if_We_Should_Infrom_Main_Function_to_PID|=0x01;
				}
				/*这里接收目标的目标的角度*/
				if (msg.StdId == CAN_TARGET_ANGLE_ID && msg.IDE == CAN_Id_Standard &&
            msg.RTR == CAN_RTR_Data && msg.DLC == 4)
				{
						/* Pitch、Yaw角度更新到缓存区 */
						for (i = 0; i < 2; ++i) {
								raw = (int32_t)((uint16_t)msg.Data[2*i] |
															 ((uint16_t)msg.Data[2*i+1] << 8));
								if (raw & 0x8000)/*这里因为是把uint16_t转换为int32_t ，高位自动补的都是0*/
										 raw |= 0xFFFF0000u;  /* 只把高16位置1，保留低16位 *//* 还原16位有符号补码 */
								target_angle[i] = (float)raw / 100.0f;
						}
						if_We_Should_Infrom_Main_Function_to_PID|=0x02;
						
				}
				#endif
		}
		if(if_We_Should_Infrom_Main_Function_to_PID==0x01)
		{
				Its_Time_to_PID = 1; /* 主控已收到同一帧的左右轮速，通知主循环处理。 */
				if_We_Should_Infrom_Main_Function_to_PID=0;
		}
}
