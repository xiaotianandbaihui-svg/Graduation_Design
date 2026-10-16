#include "can_pitch.h"
#include "stm32f10x.h"
#include "stm32f10x_can.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "misc.h"
#include <math.h>

volatile float the_angles[2] = {0.0f, 0.0f}; /* [0]=Pitch, [1]=Yaw */
volatile uint8_t Its_Time_to_PID = 0;

int CANPitch_Init(void)
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
    filter.CAN_FilterIdHigh = (uint16_t)(CAN_PITCH_RX_ID << 5);
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

int CANPitch_Send(float pitch_deg, float yaw_deg)
{
    CanTxMsg msg = {0};
    float angles[2] = {pitch_deg, yaw_deg};
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
    msg.StdId = CAN_PITCH_TX_ID;
    msg.IDE = CAN_Id_Standard;
    msg.RTR = CAN_RTR_Data;
    msg.DLC = 4;
    return CAN_Transmit(CAN1, &msg) != CAN_TxStatus_NoMailBox;
}

void USB_LP_CAN1_RX0_IRQHandler(void)
{
    CanRxMsg msg;
    int32_t raw;
    uint8_t i;
    uint8_t count = CAN_MessagePending(CAN1, CAN_FIFO0);
    /* FIFO最多3帧：本次处理入口时已有的帧，新到的留给下一次中断。 */
    while (count--) {
        CAN_Receive(CAN1, CAN_FIFO0, &msg); /* 读取并释放FIFO项 */
        if (msg.StdId != CAN_PITCH_RX_ID || msg.IDE != CAN_Id_Standard ||
            msg.RTR != CAN_RTR_Data || msg.DLC != 4) continue;
        for (i = 0; i < 2; ++i) {
            raw = (int32_t)((uint16_t)msg.Data[2*i] |
                           ((uint16_t)msg.Data[2*i+1] << 8));
            if (raw & 0x8000)/*这里因为是把uint16_t转换为int32_t ，高位自动补的都是0*/
                 raw |= 0xFFFF0000u;  /* 只把高16位置1，保留低16位 *//* 还原16位有符号补码 */
            the_angles[i] = (float)raw / 100.0f;
        }
        /* Pitch、Yaw全部更新后，才通知主循环处理。 */
        Its_Time_to_PID = 1;
    }
}
