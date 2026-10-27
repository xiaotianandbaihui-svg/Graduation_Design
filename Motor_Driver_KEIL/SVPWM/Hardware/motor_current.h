#ifndef MOTOR_CURRENT_H
#define MOTOR_CURRENT_H

#include <stdint.h>

/* 板5正式电流驱动：20kHz PWM，TIM1固定周期运行双电机d/q PI。
 * Cortex-M3无FPU，先用2kHz；提频前实测MotorCurrent_LastStepUs()，
 * 一拍包含两轮SPI、等待PWM同步ADC及PI，超时会停止两轮。
 * 电流环运行时，主循环不可再直接读SPI编码器/ADC或调用电压驱动接口。 */
#define MOTOR_CURRENT_HZ                 2000u /*这里是电流环运行的频率*/
#define MOTOR_CURRENT_BUS_VOLTAGE_V      12.0f /* 估计值！当前板上无母线ADC，按实测电压填写，我这边为了有裕量，就写12V，但是满电应该是12.6V的 */
#define MOTOR_CURRENT_LIMIT_A            2.0f  /* ±1000指令对应的Iq幅值；初次调试先小电流 */
#define MOTOR_CURRENT_TRIP_A             12.0f /* 任一相达到此值即软件停机；不是硬件瞬时保护。这个电流也就是电机的堵转电流，硬件的nFault检测电流是超过12A的，那个在
更大的电流下或许有用，但是此处，软件停机是更加有用的 */
#define MOTOR_CURRENT_COMMAND_TIMEOUT_MS 100u  /* 连续无新外环指令则停机；跳跃停环后不计时 */
#define MOTOR_CURRENT_SAMPLE_GUARD_US     2u    /* 两侧稳定余量：含死区/CSA/外部RC，需实测 ，但我也懒得测了，我直接蒙：1~2us，这已经是一个很长的时间了，*/

/* voltage_ratio=(0,1]：1使用Vbus/sqrt(3)理论线性上限，较小值预留电压余量。
 * 由你自行选择。实际还受低侧采样窗口限制，取两者较小值；设1不能取消窗口。
 * 采用最小相占空比为0的公共偏移，保证采样时两轮三相低侧均导通。 */
#define MOTOR_CURRENT_VOLTAGE_RATIO      0.90f/*这个只是初始化用一下，后续会根据ADC的配置修改这个占空比*/

/*------------------------------------------------------------------------------------------------*/
/*------------------------- 这里就是电流环的PI(D)参数了，调参在此处(开始)----------------------------*/
/*------------------------------------------------------------------------------------------------*/
/* 必须按电机整定，0表示尚未填写，Start拒绝两轴中任一轴的P/I均为0。
 * Kp单位V/A，Ki单位V/(A*s)；初始化时合并Ki*Ts，运行时不再重复乘Ts。
 * 此处是两轮共用初值；两电机不同时可在MotorCurrent_Init分别传入配置。 */

#define MOTOR_CURRENT_KP_D               0.0f
#define MOTOR_CURRENT_KI_D               0.0f
#define MOTOR_CURRENT_KP_Q               0.0f
#define MOTOR_CURRENT_KI_Q               0.0f
/*------------------------------------------------------------------------------------------------*/
/*------------------------- 这里就是电流环的PI(D)参数了，调参在此处(结束)----------------------------*/
/*------------------------------------------------------------------------------------------------*/



typedef enum {
    MOTOR_CURRENT_STOPPED = 0,
    MOTOR_CURRENT_RUNNING = 1,
    MOTOR_CURRENT_CONFIG_ERROR = -1,
    MOTOR_CURRENT_ENCODER_ERROR = -2,
    MOTOR_CURRENT_SAMPLE_ERROR = -3,
    MOTOR_CURRENT_OVERCURRENT = -4,
    MOTOR_CURRENT_CONTROL_ERROR = -5,
    MOTOR_CURRENT_OVERRUN = -6,
    MOTOR_CURRENT_COMMAND_TIMEOUT = -7
} MotorCurrent_Status;

/* Motor_init成功后调用Init一次。Start清旧目标/积分并开启定时中断；
 * Stop同时停止中断与两路PWM。故障保留为负状态，不会因重复Start自动恢复。 */
int MotorCurrent_Init(void);
int MotorCurrent_Start(void);
void MotorCurrent_Stop(void);
MotorCurrent_Status MotorCurrent_GetStatus(void);
uint32_t MotorCurrent_LastStepUs(void);

/* command为[-1000,1000]，正方向沿该轮电角度增加方向，先在main乘安装极性。
 * 保留原外环接口，但幅值现在代表Iq比例而非电压调制比；外环增益需要重调。
 * 电流星点向外为正，故Iq=-command/1000*CURRENT_LIMIT，Id固定0。
 * 每收到并处理一组新CAN姿态/目标后调用；不能用重复旧命令刷新超时。 */
int MotorCurrent_SetCommand(int16_t left, int16_t right);
/* 返回电角度正方向的机械rpm缓存；不访问SPI。 */
int MotorCurrent_ReadSpeed(float *left_rpm, float *right_rpm);

#endif
