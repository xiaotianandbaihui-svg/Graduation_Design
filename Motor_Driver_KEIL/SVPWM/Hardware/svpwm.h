#ifndef BOARD5_SVPWM_H
#define BOARD5_SVPWM_H

#include <stdint.h>

#define SVPWM_PWM_HZ     20000u
#define SVPWM_MAX_MOD    1000u
#define SVPWM_POLE_PAIRS 7u

/* 静止对齐幅值，单位与modulation相同；代码接受1~100。
 * 50是调试初值，不是电流限制；太小无法转动时标定会失败。
 * 标定时两轮悬空、可自由转动；不要用手阻挡或转动轮子。 */
#define SVPWM_ALIGN_MOD 50u

/* 方向由Motor_init自动测出，不再手填：+1/-1有效，0表示未标定或失效。
 * 表示0->90度定子矢量变化时编码器机械角的增减，不代表车体前进方向。
 * 以下宏读取运行时结果，不要在工程预定义宏中再手工定义同名常量。 */
int8_t SVPWM_GetEncoderSign(uint8_t id);
#define SVPWM_ENCODER_SIGN_1 (SVPWM_GetEncoderSign(0u))
#define SVPWM_ENCODER_SIGN_2 (SVPWM_GetEncoderSign(1u))

/* STM32F103C8T6标准库，板5，一颗MCU驱动两个DRV8323H。
 * 两颗MODE各自通过47kΩ接地选择3PWM，不共用MODE网络。
 *
 * id        0（电机1）               1（电机2）
 * INHA/B/C  PA1/PA2/PA3              PB7/PB8/PB9
 *           TIM2_CH2/3/4             TIM4_CH2/3/4
 * INLA/B/C  PB12/PB13/PB14           PB15/PB3/PA15
 * nFAULT    PA8                     PB5
 * ENABLE    PB0，两片共用
 *
 * 独占TIM2/TIM4，关闭JTAG、保留SWD；不配置系统时钟或中断。
 * SPI1、CS GPIO沿用你已有的初始化，参见tle5012b.h。
 * 使用已有Delay_us/Delay_ms，延时不能占用TIM2/TIM4。
 * 不与six_step.c同时运行；所有接口由同一执行上下文串行调用。
 */

/* 唯一电机初始化入口，返回1全部成功、0失败并关闭双路。
 * 自动完成PWM/GPIO初始化、两颗编码器检查，再逐个标定电机：
 * 0度矢量对齐并记录零点 -> 保持通电直接切90度 -> 等待稳定
 * -> 计算跨零修正后的机械角增量，核对大小并取符号 -> 高阻停止。
 * 对7极对数，机械增量应约±12.86度，允许±10%作为有效位移判据。
 * 零点始终保留0度矢量下的角度，不能用90度位置覆盖。
 * 通常约2.4秒，等待延时最多约6秒，另加通信/计算时间；不写Flash。
 * 失败后先排除原因，再显式调用本函数重试，不要循环自动重试。
 * 成功后持续调用SVPWM_Update()。 */
 
int Motor_init(void);  /*直接调用这个函数即可，返回值：1  成了 0： 寄了*/

/* 底层PWM/GPIO初始化：不做标定，结束后高阻，清除旧标定。
 * Motor_init已经调用它；日常无需另外调用。 */
int SVPWM_Init(void);

/* voltage_deg：定子电压矢量角，不是编码器机械角。
 * 0度沿A相正方向，正角度按A->B->C旋转，内部归一化到[0,360)。
 * modulation：0~1000，1000对应线性区最大幅值Vbus/sqrt(3)。
 * 0等同Stop；不是转速、电流或某一相的占空比命令。
 * 返回1成功、0失败；参数错误/nFAULT低会关闭双路并清除标定。
 * 底层接口也用于开环对齐，不要求已有标定结果。 */
int SVPWM_Set(uint8_t id, float voltage_deg, uint16_t modulation);
void SVPWM_Stop(uint8_t id); /* 指定一路INL全低，高阻，保留有效标定。 */

/* dir1/dir2只能为+1/-1，分别代表ABC坐标内的正/负q轴电压。
 * 电角度=Wrap360(自动得到的encoder_sign*极对数*(机械角-零点))，
 * 电压矢量角=电角度+dir*90度。
 * modulation仍为0~1000，0关闭对应电机且不读取该路编码器。
 * 返回1成功、0失败；未标定/参数错/读取失败/驱动故障均关闭双路。
 * 出错后重新Motor_init；停止调用不会自动停机。
 * 不包含速度环、电流环或加速度限制。 */
int SVPWM_Update(int8_t dir1, int8_t dir2,
                 uint16_t modulation_1, uint16_t modulation_2);

#endif
