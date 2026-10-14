#include "sys.h"
#include "delay.h"  
#include "usart.h"   
#include "lcd.h"
#include "touch.h" 
#include "lv_port_disp_template.h"
#include "lv_port_indev_template.h"

int main(void)
{ 
	NVIC_SetPriorityGrouping(NVIC_PriorityGroup_4);
	delay_init(168);  //初始化延时函数
	uart_init(115200);		//初始化串口波特率为115200
	
 	LCD_Init();					//LCD初始化 
	tp_dev.init();				//触摸屏初始化

}
