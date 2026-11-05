#include "sys.h"
#include "remote_control.h"
#include "usart.h"	

#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"					// 使用FreeRTOS
#endif

#include "task.h"
 
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif
 
#if EN_USART1_RX   //如果使能了接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	

//初始化IO 串口1 
//bound:波特率
void uart_init(u32 bound){
   //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
    /* DWT供串口发送及LoRa收发超时计时，不依赖delay_init，也不清零公共计数器。 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;/*打开DWT定时器*/
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;/*设置为向上计数*/

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟
 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10复用为USART1
	
	//USART1端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9，PA10

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART1, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(USART1, ENABLE);  //使能串口1 
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);
	
#if EN_USART1_RX	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启相关中断

	//Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、

#endif
	
}


/* ISR只校验并缓存完整回传；控制目标和界面仍由GUI任务处理。 */
/*
*这里我再提一嘴，32的串口，先写SR再读DR就可以由硬件自动清除全部标志位；
*RE / NE / FE / PE：只读（r）。写 SR 对它们无效，SR &= ~USART_SR_ORE 之类的写法什么都不会发生。
RXNE、TC、LBD、CTS：rc_w0，写 0 可清。
*/
void USART1_IRQHandler(void)
{
    u8 byte;
    u32 status = USART1->SR;
    if (!(status & (USART_SR_RXNE | USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)))
        return;
    byte = (u8)USART1->DR; /* 先读SR再读DR，清接收/错误状态 */
    if (status & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) {
        RemoteControl_ResetRx();/*只要有错误帧出现，直接丢弃这一包数据，并且从头开始接收*/
        return;
    }
    RemoteControl_ReceiveByte(byte);
}

#endif	

 




/* 完整帧只在任务上下文发送。短暂暂停任务切换，防止其他任务printf插入帧中；
 * 中断仍响应。8字节/115200约0.7ms，20ms超时防止外设异常卡死界面。
 * printf不要放在ISR里；本函数成功只表示UART发完，不代表LoRa对端已收到。 */
int uart_send_bytes(const u8 *data, u16 length)
{
    u16 i;
    u32 began = DWT->CYCCNT;
    u32 timeout = (SystemCoreClock / 1000u) * UART_SEND_TIMEOUT_MS;
    int ok = 1;
    vTaskSuspendAll();
    for (i = 0; i < length; ++i) {
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {
            if ((u32)(DWT->CYCCNT - began) >= timeout) { ok = 0; goto done; }
        }
        USART_SendData(USART1, data[i]);
    }
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {
        if ((u32)(DWT->CYCCNT - began) >= timeout) { ok = 0; break; }
    }
done:
    (void)xTaskResumeAll();
    return ok;
}
