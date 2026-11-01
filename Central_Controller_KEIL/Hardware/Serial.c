#include "stm32f10x.h"                  // Device header
#include "Serial.h"
#include <stdarg.h>
#include <math.h>

/* 本工程的旧CMSIS未声明DWT结构体；这里直接访问周期计数寄存器。 */
#define LORA_CYCLE_NOW (*(volatile uint32_t *)0xE0001004u)/*我也不知道*/

volatile uint8_t Serial_RxData;
volatile uint8_t Serial_RxFlag;

extern volatile float target_Speed;
extern float current_target_angle_pitch_yaw[2];
extern volatile uint8_t Jump_Request;

static uint8_t control_frame[SERIAL_CONTROL_FRAME_SIZE];
static uint8_t control_count, previous_jump;
static volatile int16_t received_speed, received_yaw;

/* bit0：有新目标；bit1：有尚未交给主循环的跳跃请求。 */
volatile uint8_t Serial_ControlPending;
volatile uint8_t Lora_Timeout;
volatile uint32_t Lora_LastRxCycles;

/* 固定8字节滑动检查；半包、错包后仍能找到下一帧，不在中断里做PID。
这个函数处理数据包
*/
static void Serial_ReceiveControlByte(uint8_t byte)
{
    uint8_t i, sum = 0;
    int16_t speed, yaw;
    control_frame[control_count++] = byte;
    if (control_count < SERIAL_CONTROL_FRAME_SIZE) return;

    if (control_frame[0] == SERIAL_CONTROL_HEAD1 &&
        control_frame[1] == SERIAL_CONTROL_HEAD2)/*检查包头*/
	{
        for (i = 0; i < SERIAL_CONTROL_FRAME_SIZE - 1u; ++i)
            sum = (uint8_t)(sum + control_frame[i]);/*这是一个简单的校验方法，就是对包内数据做累加，能做到一个很弱的纠错*/
        speed = (int16_t)((uint16_t)control_frame[2] |
                         ((uint16_t)control_frame[3] << 8));
        yaw = (int16_t)((uint16_t)control_frame[4] |
                       ((uint16_t)control_frame[5] << 8));
        if (sum == control_frame[7] && control_frame[6] <= 1u &&
            yaw >= -18000 && yaw <= 18000) /*这里传这么大，其实是角度精度的保障，然后呢control_frame[6]是跳跃控制信号*/
		{
            Lora_LastRxCycles = LORA_CYCLE_NOW; /* 只有完整有效帧才续期 */
            received_speed = speed;
            received_yaw = yaw;
            Serial_ControlPending |= 0x01u;/*标记新的目标已经产生*/
            /* 0->1只请求一次；后续jump=0的帧不抹掉尚未处理的请求。 */
            if (control_frame[6] && !previous_jump) Serial_ControlPending |= 0x02u;
            previous_jump = control_frame[6];/*连续的跳跃请求是只会响应一次的，我如果想要二次请求，遥控端就要先发送无跳跃请求，再
			发送跳跃请求*/
            control_count = 0;
            return;
        }
    }
    /* 校验不通过只移走一个字节，保留可能已经收到的下一帧帧头。 */
    for (i = 1; i < SERIAL_CONTROL_FRAME_SIZE; ++i)
        control_frame[i - 1u] = control_frame[i];
    control_count = SERIAL_CONTROL_FRAME_SIZE - 1u;
	/*这么做的目的是：假设前面多收到了一个无关字节 99，那么缓冲区第一次收满 8 字节时，内容是：
	数组下标： 0   1   2   3   4   5   6   7
	收到数据：99  AA  55  E8  03  28  23  00
	此时开头是 99 AA，帧头不对。但真正的帧头 AA 55 已经在数组里面了。
	也算是在一定程度上增加纠错能力吧*/
}



int Serial_ProcessControl(void)
{
    int16_t speed, yaw;
    uint8_t pending;
    uint32_t now, mask = __get_PRIMASK();

    /* 与收包中断一起保护时间戳和整帧，避免刚收新帧却被判为超时。 */
    __disable_irq();/**/
    now = LORA_CYCLE_NOW;
    if ((uint32_t)(now - Lora_LastRxCycles) >=
        (SystemCoreClock / 1000u) * SERIAL_LORA_TIMEOUT_MS) {
        Lora_LastRxCycles = now; /* 无新包时，从这里开始下一轮10秒检查 */
        Lora_Timeout = 1u;       /* 保持置位，供其他代码读取和清零 */
        Serial_ControlPending = 0u;
        control_count = 0u;
        target_Speed = 0.0f;
        Jump_Request = 0u;
        __set_PRIMASK(mask);
        return 0;
    }
    speed = received_speed;
    yaw = received_yaw;
    pending = Serial_ControlPending;
    Serial_ControlPending = 0u;
    __set_PRIMASK(mask);
    if (!(pending & 0x01u)) return 0;

    target_Speed = speed * 0.1f; /* rpm */
    current_target_angle_pitch_yaw[1] = yaw * 0.01f; /* 度；Pitch仍由速度PI产生 */
    if (pending & 0x02u) Jump_Request = 1;
    return 1; /* 主循环用实测数据帧作响应，不另发ACK。 */
}

int Serial_SendFeedback(float speed_rpm, float yaw_deg)
{
    uint8_t frame[SERIAL_FEEDBACK_FRAME_SIZE], i, sum = 0u;
    int16_t speed, yaw;
    float scaled;
    uint32_t began;
    if (!isfinite(speed_rpm) || !isfinite(yaw_deg)) return 0;
    scaled = speed_rpm * 10.0f;
    if (scaled > 32767.0f) scaled = 32767.0f;
    if (scaled < -32768.0f) scaled = -32768.0f;
    speed = (int16_t)(scaled + (scaled >= 0.0f ? 0.5f : -0.5f));
    scaled = fmodf(yaw_deg + 180.0f, 360.0f);
    if (scaled < 0.0f) scaled += 360.0f;
    scaled = (scaled - 180.0f) * 100.0f;
    yaw = (int16_t)(scaled + (scaled >= 0.0f ? 0.5f : -0.5f));
    frame[0] = 0x55u; frame[1] = 0xAAu;
    frame[2] = (uint8_t)speed; frame[3] = (uint8_t)((uint16_t)speed >> 8);
    frame[4] = (uint8_t)yaw; frame[5] = (uint8_t)((uint16_t)yaw >> 8);
    for (i = 0u; i < SERIAL_FEEDBACK_FRAME_SIZE - 1u; ++i)
        sum = (uint8_t)(sum + frame[i]);
    frame[6] = sum;

    /* 只在主循环发送，printf仍可用；不要从中断输出调试文本。 */
    began = LORA_CYCLE_NOW;
    for (i = 0u; i < SERIAL_FEEDBACK_FRAME_SIZE; ++i) {
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
            if ((uint32_t)(LORA_CYCLE_NOW - began) >=
                (SystemCoreClock / 1000u) * SERIAL_TX_TIMEOUT_MS) return 0;
        USART_SendData(USART1, frame[i]);
    }
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
        if ((uint32_t)(LORA_CYCLE_NOW - began) >=
            (SystemCoreClock / 1000u) * SERIAL_TX_TIMEOUT_MS) return 0;
    return 1;
}

void Serial_Init(void)
{
    /* 启用DWT但不清零，避免干扰其他使用同一计数器的代码。 */
    SystemCoreClockUpdate();
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    *(volatile uint32_t *)0xE0001000u |= 1u;
    Lora_LastRxCycles = LORA_CYCLE_NOW;/*初始化的时候就会获取一次当前DWT时钟*/
    Lora_Timeout = 0u;
    control_count = previous_jump = 0;
    Serial_ControlPending = 0;
    Serial_RxData = Serial_RxFlag = 0;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = SERIAL_BAUD_RATE;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART1, &USART_InitStructure);
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_Cmd(USART1, ENABLE);
}

void Serial_SendByte(uint8_t Byte)
{
	USART_SendData(USART1, Byte);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i ++)
	{
		Serial_SendByte(Array[i]);
	}
}

void Serial_SendString(char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i ++)
	{
		Serial_SendByte(String[i]);
	}
}

uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y --)
	{
		Result *= X;
	}
	return Result;
}

void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i ++)
	{
		Serial_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');
	}
}

int fputc(int ch, FILE *f)
{
	Serial_SendByte(ch);
	return ch;
}

void Serial_Printf(char *format, ...)
{
	char String[100];
	va_list arg;
	va_start(arg, format);
	vsprintf(String, format, arg);
	va_end(arg);
	Serial_SendString(String);
}

uint8_t Serial_GetRxFlag(void)
{
	if (Serial_RxFlag == 1)
	{
		Serial_RxFlag = 0;
		return 1;
	}
	return 0;
}

uint8_t Serial_GetRxData(void)
{
	return Serial_RxData;
}

void USART1_IRQHandler(void)
{
    uint32_t status = USART1->SR;
    uint8_t byte;
    if (!(status & (USART_SR_RXNE | USART_SR_ORE | USART_SR_NE |
                    USART_SR_FE | USART_SR_PE))) return;

    /* 先读SR再读DR即可清接收/错误标志，不再手动清RXNE，避免丢新字节。 */
    byte = (uint8_t)USART1->DR;
    if (status & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) {
        control_count = 0; /* 出错的半帧丢弃，下一完整帧重新同步。 */
        return;
    }
    Serial_RxData = byte;
    Serial_RxFlag = 1;
    Serial_ReceiveControlByte(byte);
}
