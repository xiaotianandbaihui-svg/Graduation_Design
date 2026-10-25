#ifndef CYCLE_COUNTER_H
#define CYCLE_COUNTER_H

#include "stm32f10x.h"
/*这个内核私有外设我推荐去看教程： https://doc.embedfire.com/mcu/stm32/f103badao/std/zh/latest/book/DWT.html 写的很好*/
/* 工程自带的旧CMSIS没有DWT定义；Cortex-M3的CTRL/CYCCNT固定地址。
 * 只开启，不清零；用uint32_t差值计时可跨越约59.65秒的自然回绕。 */
static __inline void CycleCounter_Enable(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    *(volatile uint32_t *)0xE0001000u |= 1u;
}
static __inline uint32_t CycleCounter_Now(void)
{
    return *(volatile uint32_t *)0xE0001004u;
}

#endif
