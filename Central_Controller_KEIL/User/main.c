#include "stm32f10x.h"
#include "Serial.h"
#include "mpu6500_spi.h"
#include <stdio.h>

#define IMU_PERIOD_MS  20u       /* 定时通知主循环；需要20ms就改为20u */
#if IMU_PERIOD_MS < 1u || IMU_PERIOD_MS > 50u
#error IMU_PERIOD_MS_must_be_1_to_50
#endif

volatile uint8_t flag_to_update;
static volatile uint32_t imu_time_us; /* 已完成的定时周期，单位us */
float angle[3];                      /* Pitch、Roll、Yaw，度；仅主循环写 */
int8_t imu_result;                   /* 1新角度、0暂无、-1错误，供调试查看 */

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
    imu_time_us = 0;
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
    while (1) {
        if (flag_to_update) {
            flag_to_update = 0; /* 先清零，保留读取/解算期间新到的请求 */
        }
    }
}
