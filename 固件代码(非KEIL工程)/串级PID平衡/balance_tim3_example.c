#include "balance_tim3_example.h"
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "misc.h"

volatile uint8_t PID_Refresh_Flag = 0u;

int Balance_TIM3_Init(void)
{
    RCC_ClocksTypeDef clocks;
    TIM_TimeBaseInitTypeDef timer;
    NVIC_InitTypeDef irq;
    uint32_t timer_hz;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    Balance_TIM3_Stop();
    RCC_GetClocksFreq(&clocks);
    timer_hz = clocks.PCLK1_Frequency;
    if (clocks.PCLK1_Frequency != clocks.HCLK_Frequency) timer_hz *= 2u;
    if (timer_hz < 1000000u || timer_hz % 1000000u != 0u ||
        timer_hz / 1000000u > 65536u) return 0;

    TIM_DeInit(TIM3);
    timer.TIM_Prescaler = (uint16_t)(timer_hz / 1000000u - 1u);
    timer.TIM_Period = 5000u - 1u; /* 每计数1us，共5ms */
    timer.TIM_CounterMode = TIM_CounterMode_Up;
    timer.TIM_ClockDivision = TIM_CKD_DIV1;
    timer.TIM_RepetitionCounter = 0u;
    TIM_TimeBaseInit(TIM3, &timer);
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update); /* 清除初始化UG产生的标志 */
    NVIC_ClearPendingIRQ(TIM3_IRQn);

    irq.NVIC_IRQChannel = TIM3_IRQn;
    irq.NVIC_IRQChannelPreemptionPriority = 1u;
    irq.NVIC_IRQChannelSubPriority = 0u;
    irq.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&irq);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
    return 1;
}

void Balance_TIM3_Stop(void)
{
    NVIC_DisableIRQ(TIM3_IRQn);
    TIM_Cmd(TIM3, DISABLE);
    TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    NVIC_ClearPendingIRQ(TIM3_IRQn);
    PID_Refresh_Flag = 0u;
}

void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        PID_Refresh_Flag = 1u;
    }
}

void Balance_CommandToDrive(const int16_t command[2],
                            int8_t dir[2], uint16_t modulation[2])
{
    int i;
    for (i = 0; i < 2; ++i) {
        dir[i] = command[i] < 0 ? -1 : 1;
        modulation[i] = (uint16_t)(command[i] < 0 ? -(int32_t)command[i] : command[i]);
    }
}