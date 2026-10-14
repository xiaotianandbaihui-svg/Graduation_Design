#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Serial.h"
#include "mpu9250_simple.h"
/*原始数据数组*/
int16_t xyz_Accel[3]={0};
int16_t xyz_Gyro[3]={0};
int16_t xyz_ReadMag[3]={0};


int main(void)
{
	
	Serial_Init();
	MPU9250_Init();//MPU9250初始化，但是这一切都要求相对应的接口已经初始化好了
	while (1)
	{
	}
}
