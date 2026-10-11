#include "jump_tick.h"
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"

uint8_t JumpTick_Init(void)
{
   // RCC_ClocksTypeDef clocks;
    TIM_TimeBaseInitTypeDef base;
    uint32_t timer_hz, divider;

/*   只读取SystemInit已经配置好的时钟。APB1分频时，定时器时钟翻倍。 但这里属实是多余了，硬件已经做了自动升频。
    RCC_GetClocksFreq(&clocks);
    timer_hz = clocks.PCLK1_Frequency;
    if (clocks.PCLK1_Frequency != clocks.HCLK_Frequency) timer_hz *= 2u;
    if (timer_hz < 10000u || timer_hz % 10000u != 0u) return 0;
    divider = timer_hz / 10000u;
    if (divider > 65536u) return 0;
*/


//    NVIC_DisableIRQ(TIM4_IRQn);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    TIM_DeInit(TIM4);
    TIM_TimeBaseStructInit(&base);
    base.TIM_Prescaler = (uint16_t)(divider - 1u); /* 72MHz时为7200-1 */
    base.TIM_Period = 65535u;                    /* 每次加1为0.1ms */
    base.TIM_CounterMode = TIM_CounterMode_Up;
    base.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM4, &base);

//    TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
    JumpTick_Stop(); /* 配置后保持停止，等Jump_Start成功才开始计时 */
//    NVIC_ClearPendingIRQ(TIM4_IRQn);
    return 1;
}

void JumpTick_Start(void)
{
    JumpTick_Stop();
    TIM_Cmd(TIM4, ENABLE);
}

void JumpTick_Stop(void)
{
    TIM_Cmd(TIM4, DISABLE);
    /* 向上计数时UG将CNT及预分频内部计数清零、装载PSC/ARR配置。
     * 不是把ARR=65535复制给CNT。UG也会置更新标志，因此随后清除。
     */
    TIM_GenerateEvent(TIM4, TIM_EventSource_Update);
    TIM_ClearFlag(TIM4, TIM_FLAG_Update);
}

uint32_t JumpTick_Millis(void)
{
    uint16_t count = TIM_GetCounter(TIM4);
    /* 先读CNT再检查标志，捕获恰好在读取过程中发生的溢出。
     * 未开启中断，UIF仍会在溢出后置位，并保留到主动清除。
     */
    if (TIM_GetFlagStatus(TIM4, TIM_FLAG_Update) != RESET) return JUMP_TICK_EXPIRED;
    return (uint32_t)count / 10u;
}
