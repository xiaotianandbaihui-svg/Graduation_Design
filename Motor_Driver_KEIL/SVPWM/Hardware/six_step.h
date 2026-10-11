#ifndef SIX_STEP_H
#define SIX_STEP_H

#include <stdint.h>

/* BDS3620：14极=7对极。运行占空比0~1000分别表示0%~100%。
 * 标定时仍单独限制到10%，因为固定矢量对齐时没有转速反电动势。 */
#define SIX_STEP_POLE_PAIRS       7u
#define SIX_STEP_PWM_HZ          20000u
#define SIX_STEP_MAX_DUTY        1000u
#define SIX_STEP_MAX_ALIGN_DUTY  100u

/* 运行方向指令，与标定参数encoder_sign分开。 */
typedef enum {
    SIX_STEP_FORWARD = 0,
    SIX_STEP_REVERSE = 1
} SixStep_Direction;

typedef struct {
    float zero_deg;          /* A+、B-、C悬空对齐后的编码器机械角，[0,360) */
    int8_t encoder_sign;     /* 沿矢量0->1->2的方向：编码器递增=+1，递减=-1 */
} SixStep_Calibration;

typedef enum {
    SIX_STEP_OK = 0,
    SIX_STEP_ARGUMENT_ERROR = -1,
    SIX_STEP_NOT_READY = -2,
    SIX_STEP_NOT_CALIBRATED = -3,
    SIX_STEP_DRIVER_FAULT = -4,
    SIX_STEP_ENCODER_ERROR = -5,
    SIX_STEP_CALIBRATION_ERROR = -6,
    SIX_STEP_CLOCK_ERROR = -7
} SixStep_Result;

/* STM32F103C8T6标准库 + DRV8323H，MODE必须接地（6PWM）。
 * 接线沿用2026-09-29板1网络表；若实物改线，修改six_step.c的引脚表。
 * id          0 / 电机1             1 / 电机2
 * INHA/B/C    PA1/PA2/PA3           PB7/PB8/PB9
 *             TIM2_CH2/3/4         TIM4_CH2/3/4
 * INLA/B/C    PB12/PB13/PB14        PB15/PB3/PA15
 * nFAULT      PA8                  PB5（均需板上上拉）
 * ENABLE      PB0，两片共用
 *
 * 独占TIM2、TIM4；不使用PA0。关闭JTAG，保留SWD。
 * PWM时钟须为HCLK=72MHz、PCLK1=36MHz。
 * 延时直接调用你已有的void Delay_us(uint32_t us)，单位微秒；
 * 其实现须在SixStep_Init之前可用，不能占用本模块的TIM2/TIM4。
 * SPI1、编码器CS及上电等待>=10ms由你原有代码完成；本文件不初始化SPI。
 * 使用同目录精简版tle5012b.c，不要混入旧目录中的同名驱动。
 * 所有接口在同一执行上下文调用，SPI1也不得被另一任务/中断抢占。
 */

/* 配置双路GPIO/PWM并唤醒驱动器，全部输出保持关闭，不自动转动。
 * 重新调用会停止双路、清除本模块的故障锁存和RAM标定。 */
SixStep_Result SixStep_Init(void);

/* 空载、轴可自由转动时调用，两台电机都必须停止。
 * duty范围1~100，例如50=5%；约2.4秒阻塞，依次对齐0->1->2->3->4->5->0。
 * 自动识别方向，检查每步约360/(7*6)度、末段稳定性，最后关闭输出。
 * 成功后RAM标定立即生效，同时返回cal供应用保存；本文件不读写Flash。
 * 标定会转动，使用限流电源；低占空比不保证低电流。 */
SixStep_Result SixStep_Calibrate(uint8_t id, uint16_t duty,
                               SixStep_Calibration *cal);

/* 加载同一电机、接线、编码器配置下的已验证标定；双路须停止。
 * 与Calibrate二选一。零位不能随意填0，encoder_sign只能是+1或-1。 */
SixStep_Result SixStep_SetCalibration(uint8_t id,
                                    const SixStep_Calibration *cal);

/* 每次读取[0,360)机械角，按标定和7对极换算电角度，再更新六步输出。
 * 机械一圈对应7个电周期、42个换相区间；不是把机械一圈只分成6步。
 * duty为0~1000的无符号千分比：0滑行停止，500为50%，1000为100%。
 * direction单独选择正/反向转矩指令；本函数不是转速/电流闭环。
 * 建议每50~100us调用一次（两路轮流调用也须满足各自更新要求），
 * 要测量实际执行耗时；不要用Delay_ms(1)安排高速换相。
 * 本文件不注册中断、不自动定时调用，也不设置失联看门狗：
 * 停止调用不会自动停止PWM。应用必须保证调度并实现必要的超时停机。
 * 编码器错误或任一nFAULT为低：双路关闭并锁存，排查后重新Init。
 * nFAULT由本函数/标定过程轮询，没有安装故障外部中断。
 */
SixStep_Result SixStep_Update(uint8_t id, uint16_t duty, SixStep_Direction direction);

void SixStep_Stop(uint8_t id);  /* 仅停止指定电机，保留标定，可再次Update */
void SixStep_StopAll(void);     /* 双路关闭、ENABLE拉低，必须重新Init */

/* 最小接入（错误返回时不要继续执行下面的步骤）：
 *
 * SixStep_Calibration cal;
 * // 先完成你自己的SPI1、CS GPIO初始化及编码器上电等待。
 * if (SixStep_Init() != SIX_STEP_OK) { ... }
 * if (SixStep_Calibrate(0, 50, &cal) != SIX_STEP_OK) { ... }
 * // 若已有有效标定，则上句改为SixStep_SetCalibration(0, &saved_cal)。
 * // 此后由你自己的控制循环周期调用：
 * SixStep_Update(0, 50, SIX_STEP_FORWARD); // 正向5%
 * // 反向用SIX_STEP_REVERSE；duty=0或SixStep_Stop(0)停止。
 */

#endif
