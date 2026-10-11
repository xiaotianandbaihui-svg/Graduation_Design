#ifndef BOARD5_SVPWM_H
#define BOARD5_SVPWM_H

#include <stdint.h>

#define SVPWM_PWM_HZ  20000u
#define SVPWM_MAX_MOD 1000u

/* STM32F103C8T6标准库，板5，一颗MCU驱动两个DRV8323H。
 * 必须：两颗MODE均选择3PWM；推荐各自通过47kΩ接地。
 * 2026-10-06板5：PB0已连接两颗ENABLE；MODE仍共网，建议拆为MODE1/MODE2。
 *
 * id        0（电机1）               1（电机2）
 * INHA/B/C  PA1/PA2/PA3              PB7/PB8/PB9
 *           TIM2_CH2/3/4             TIM4_CH2/3/4
 * INLA/B/C  PB12/PB13/PB14           PB15/PB3/PA15
 * nFAULT    PA8                     PB5
 * ENABLE    PB0，两片共用
 *
 * 独占TIM2/TIM4，关闭JTAG、保留SWD；不配置系统时钟、SPI或中断。
 * 调用你已有的void Delay_us(uint32_t us)，延时不能占用TIM2/TIM4。
 * 不与six_step.c同时运行。所有接口由同一个执行上下文串行调用。
 */

/* 返回1成功、0失败；初始化后两路均为高阻，不会自动转动。 */
int SVPWM_Init(void);

/* voltage_deg：定子电压矢量角（度），内部归一化到[0,360)。
 * 0度沿A相正方向，正角度按A->B->C旋转；不是编码器机械角。
 * modulation：0~1000，1000对应线性区最大矢量幅值Vbus/sqrt(3)。
 * 0等同Stop；不是转速、电流或某一相的占空比命令。
 * 返回1成功、0失败。参数错误/任一nFAULT低会关闭双路，需重新Init。
 * 每次只更新一次，应用负责持续给出角度/幅值；停止调用不会自动停机。
 */
int SVPWM_Set(uint8_t id, float voltage_deg, uint16_t modulation);

void SVPWM_Stop(uint8_t id); /* 指定一路INL全拉低，高阻；另一路可继续运行。 */

#endif
