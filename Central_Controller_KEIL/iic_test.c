#include "iic_test.h"
#include "Delay.h"


void IIC_SDA(uint8_t level)
{
    if (level) GPIO_SetBits(MPU_GPIO, MPU_SDA_PIN);
    else       GPIO_ResetBits(MPU_GPIO, MPU_SDA_PIN);
}

void IIC_SCL(uint8_t level)
{
    if (level) GPIO_SetBits(MPU_GPIO, MPU_SCL_PIN);
    else       GPIO_ResetBits(MPU_GPIO, MPU_SCL_PIN);
}

uint8_t READ_SDA()
{
    return (GPIO_ReadInputDataBit(MPU_GPIO, MPU_SDA_PIN) != Bit_RESET) ? 1u : 0u;
}


/**************************************************************************
函数功能：IIC引脚初始化
入口参数：无
返回  值：无
**************************************************************************/
void IIC_Init(void)
{			
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
}
/**************************************************************************
函数功能：模拟IIC起始信号
入口参数：无
返回  值：无
**************************************************************************/
void IIC_Start(void)
{
	SDA_OUT();    
	IIC_SDA(1);	  	  
	IIC_SCL(1);
	Delay_us(4);
	//START:when CLK is high,DATA change form high to low 
 	IIC_SDA(0);
	Delay_us(4);
	IIC_SCL(0);
}	  
/**************************************************************************
函数功能：模拟IIC结束信号
入口参数：无
返回  值：无
**************************************************************************/
void IIC_Stop(void)
{
	SDA_OUT();
	IIC_SCL(0);
	IIC_SDA(0);
 	Delay_us(4);
	IIC_SCL(1); 
	IIC_SDA(1);
	Delay_us(4);							   	
}
/**************************************************************************
函数功能：IIC产生应答信号
入口参数：无
返回  值：无
**************************************************************************/
unsigned char IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      
	IIC_SDA(1);Delay_us(1);	   
	IIC_SCL(1);Delay_us(1);	 
	while(READ_SDA())
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL(0);  
	return 0;  
} 
/**************************************************************************
函数功能：IIC应答
入口参数：无
返回  值：无
**************************************************************************/
void IIC_Ack(void)
{
	IIC_SCL(0);
	SDA_OUT();
	IIC_SDA(0);
	Delay_us(2);
	IIC_SCL(1);
	Delay_us(2);
	IIC_SCL(0);
}
/**************************************************************************
函数功能：IIC不应答
入口参数：无
返回  值：无
**************************************************************************/ 
void IIC_NAck(void)
{
	IIC_SCL(0);
	SDA_OUT();
	IIC_SDA(1);
	Delay_us(2);
	IIC_SCL(1);
	Delay_us(2);
	IIC_SCL(0);
}					 				     
/**************************************************************************
函数功能：IIC发送一个位
入口参数：无
返回  值：无
**************************************************************************/
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL(0);
    for(t=0;t<8;t++)
    {              
        IIC_SDA((txd&0x80)>>7);
        txd<<=1; 	  
		Delay_us(2);   
		IIC_SCL(1);
		Delay_us(2); 
		IIC_SCL(0);	
		Delay_us(2);
    }	 
} 	    
/**************************************************************************
函数功能：II读取一个位
入口参数：无
返回  值：无
**************************************************************************/
unsigned char IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();
    for(i=0;i<8;i++ )
	{
        IIC_SCL(0); 
        Delay_us(2);
		IIC_SCL(1);
        receive<<=1;
        if(READ_SDA())receive++;   
		Delay_us(1); 
    }					 
    if (!ack)
        IIC_NAck();
    else
        IIC_Ack();
    return receive;
}

