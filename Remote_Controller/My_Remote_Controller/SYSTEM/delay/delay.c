#include "delay.h"
#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"
#include "task.h"
#endif

static u32 fac_us;

/* 只在启动调度器前初始化一次；FreeRTOS接管后仍使用HCLK/8。 */
void delay_init(u8 SYSCLK)
{
    fac_us = SYSCLK / 8u;
    SysTick->CTRL = 0u;
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
    SysTick->LOAD = 0x00FFFFFFu;
    SysTick->VAL = 0u;
    /* 启动前只开计数，不开中断，避免提前进入FreeRTOS的SysTick处理。 */
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}
/*这个初始化函数这么用其实是这样的：
vTaskStartScheduler()
    → xPortStartScheduler()
        → vPortSetupTimerInterrupt()
之后：在底层函数里
SysTick->CTRL = 0;       
SysTick->VAL  = 0;       
SysTick->LOAD = configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ - 1;

SysTick->CTRL = SysTick_CTRL_ENABLE_Msk
              | SysTick_CTRL_TICKINT_Msk;        
 */



/* 忙等阻塞；任务中暂停调度但保留中断，结束后恢复原有调度状态。 */
void delay_us(u32 nus)
{
    u32 reload, ticks, told, tnow, tcnt, part;
#if SYSTEM_SUPPORT_OS
    BaseType_t scheduler_started;
#endif
    if (nus == 0u) return;
#if SYSTEM_SUPPORT_OS
    scheduler_started = xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED;
    if (scheduler_started) vTaskSuspendAll();
#endif
    reload = SysTick->LOAD + 1u; /* 递减计数器从LOAD数到0，共LOAD+1拍。 */
    while (nus) {
        /* 分段只为防止乘法溢出；整次调用仍然阻塞，不受单次回绕限制。 */
        part = nus > 1000000u ? 1000000u : nus;
        ticks = part * fac_us;
        tcnt = 0u;
        told = SysTick->VAL;
        while (tcnt < ticks) {
            tnow = SysTick->VAL;
            tcnt += told >= tnow ? told - tnow : reload - tnow + told;
            told = tnow;
        }
        nus -= part;
    }
#if SYSTEM_SUPPORT_OS
    if (scheduler_started) (void)xTaskResumeAll();
#endif
}

void delay_ms(u16 nms)
{
    delay_us((u32)nms * 1000u);
}
