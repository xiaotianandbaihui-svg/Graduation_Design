#ifndef CURRENT_SENSE_H
#define CURRENT_SENSE_H

#include <stdint.h>

/* 板5：VREF与VDDA均接3.3V；GAIN悬空=20V/V，分流电阻5mΩ。
 * VDDA可改为实测值；若更换分流电阻或GAIN配置，须同步修改下面的值。 */
#define CURRENT_SENSE_VDDA_V       3.3f
#define CURRENT_SENSE_GAIN         20.0f
#define CURRENT_SENSE_SHUNT_OHM    0.005f

typedef struct {
    uint16_t a[2], b[2];      /* id=0/1为电机1/2；12位ADC原始码，0~4095 */
} CurrentSense_Raw;

typedef struct {
    float ia_a, ib_a, ic_a;   /* 单位A；正方向为星点经绕组流向逆变器 */
} CurrentSense_Phase;

/* 由SVPWM_Init/SixStep_Init自动调用，不必在main重复初始化。
 * 前提：EN=PA0已拉高并等DRV唤醒，CAL=PB4已配置输出低，六个MOS全关闭，
 * 两轮静止且分流电阻中无电流。重新调用也必须满足这些条件。
 * 独占ADC1/ADC2，配置PA5/PA6/PA7/PB0为模拟输入；ADC时钟PCLK2/6。
 * 完成ADC自校准、DRV的CAL脉冲和四路零偏平均；1成功、0失败。
 * 初始化在主循环停机时进行；电流环运行后由TIM1中断独占采样接口。 */
int CurrentSense_Init(void);

/* 软件触发一次双ADC注入序列并等待完成，1成功、0失败（不改输出）。
 * ADC1: IN5->IN7，ADC2: IN6->IN8；同一电机A/B同步，两电机依次采样。
 * 这是调试/底层采样入口，并未与TIM2/TIM4同步！低侧电阻只有在对应
 * 低管导通且信号稳定时才代表相电流，随意调用后不能直接用作FOC反馈。
 * 正式电流环使用下面的ReadSynchronized，不能混用软件采样。
 * 超时使采样模块失效，须在停机状态重新Init，不能把上一次结果当新数据。 */
int CurrentSense_ReadRaw(CurrentSense_Raw *raw);

/* 等待下一次TIM2_TRGO硬件触发并读取新样本；两轮A/B各同步，双轮依次。
 * 前提：SVPWM_StartCurrent已建立同步及有效窗口，DWT周期计数已开启。
 * 只由电流环调用；最长等待200us，失败后停机，不复用旧样本。 */
int CurrentSense_ReadSynchronized(CurrentSense_Raw *raw);

/* 用已校准零偏换算指定电机的A/B相，并按ic=-ia-ib重构C相。
 * 1成功；0表示未初始化/参数错误/放大器输出超出保守线性区，不改输出。
 * 返回1仅表示换算成功，不代表采样窗口有效；由调用方保障低侧采样条件。
 * DRV8323H: VSO=VREF/2-GAIN*(VSP-VSN)。当前SP接分流器上端，
 * 正相电流从星点经绕组流向逆变器，在有效低侧采样窗口与分流器流向地同向，
 * 所以ia=(zero-ADC)*比例。三相均采用此方向，ic=-ia-ib。
 * 它不是车体前进方向，不随左右轮安装方向翻转。
 * 本模块不测母线电压，也不提供12A过流关断。 */
int CurrentSense_Convert(uint8_t id, const CurrentSense_Raw *raw,
                         CurrentSense_Phase *current);

#endif
