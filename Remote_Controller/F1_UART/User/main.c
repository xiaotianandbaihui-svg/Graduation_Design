#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Serial.h"

uint8_t RxData[10];
uint8_t counter=0;
int main(void)
{
	OLED_Init();
	OLED_ShowString(1, 1, "RxData:");
	
	Serial_Init();
	
	while (1)
	{
		if (Serial_GetRxFlag() == 1)
		{
			
			for(int i=0;i<10;i++)
						Serial_SendByte(RxData[i]);
			//Serial_SendByte(RxData);
			OLED_ShowHexNum(1, 8, RxData[0], 2);
			OLED_ShowHexNum(6, 8, RxData[1], 2);
		
			OLED_ShowHexNum(8,8, RxData[2],2);

		}
	}
}
