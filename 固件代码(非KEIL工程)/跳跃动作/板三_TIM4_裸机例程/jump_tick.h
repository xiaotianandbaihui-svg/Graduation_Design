#ifndef JUMP_TICK_H
#define JUMP_TICK_H

#include <stdint.h>

/* 板三专用：TIM4每0.1ms加1，ARR=65535；不用中断，不占用GPIO。
 * 仅在一次跳跃执行期间运行，不是系统时基，Delay_ms/us不能依赖它。
 * TIM3继续产生舵机PWM；不配置系统时钟或SysTick。
 */
#define JUMP_TICK_EXPIRED UINT32_MAX /* 计数超过6.5536s，不再返回回绕后的时间 */

uint8_t JumpTick_Init(void);    /* 配置后保持停止；1成功，0无法整分频 */
void JumpTick_Start(void);     /* 清零CNT及预分频计数后开始本次计时 */
void JumpTick_Stop(void);      /* 停止，通过更新事件清零并清除更新标志 */
uint32_t JumpTick_Millis(void); /* CNT/10，整数毫秒；溢出返回EXPIRED */

#endif
