#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include <stdint.h>

/* V1配置入口；两端LoRa须使用相同信道/空中速率，并工作在透明传输模式。 */
#define REMOTE_UART_BAUD          115200u
#define REMOTE_SEND_PERIOD_MS     100u /* 遥控页由上位机时钟定期发送；回传不改变节拍 */
#define REMOTE_WHEEL_DIAMETER_M   0.064f
#define REMOTE_MAX_SPEED_MPS      1.0f /* 摇杆拉满时的目标速度，调试时可调小 */
#define REMOTE_STICK_DEAD_ZONE    0.12f /* 半径内12%为死区，防止中点附近Yaw乱跳 */
#define REMOTE_YAW_SIGN           1.0f /* 默认向右为+90度；与小车Yaw正方向不符时改-1 */
#define REMOTE_YAW_OFFSET_DEG     0.0f /* 摇杆正上方对应的绝对Yaw，须与主控姿态参考系一致 */
#define REMOTE_JUMP_FRAMES        1u   /* 发1帧jump=1，随后发0重新允许下一次跳跃 */
#define REMOTE_FRAME_SIZE         8u
#define REMOTE_FEEDBACK_SIZE      7u
#define REMOTE_LORA_TIMEOUT_MS    10000u /* 仅遥控页：10秒无有效回传则停止目标、置超时标志 */

typedef struct {
    float speed_mps;
    float yaw_deg;
    uint8_t jump;
} RemoteCommand;/*目标速度、目标yaw角、跳跃命令集中*/

typedef struct {
    float speed_mps; /* 两轮平均转速换算，轮径由REMOTE_WHEEL_DIAMETER_M确定 */
    float yaw_deg;
    uint8_t valid;     /* 当前可显示实测值；重新进入遥控页后等待新数据，简单的：记录当前是否有可用于显示的有效回传数据 */
    uint8_t timed_out; /* 遥控页当前掉线；有效回传或换页时清零，用于界面提示 */
} RemoteFeedback;
/*这个valid再详细说说：
| 情况 | `feedback.valid` |
|---|---|
| 刚进入遥控界面，尚未收到有效回传 | `0` |
| 收到完整、校验通过的回传，并由 LoRa 任务处理 | 置为 `1` |
| 保持遥控模式，期间没有超时 | 一直为 `1` |
| 接收超时 | 清为 `0` |
| 超时后重新收到并处理有效回传 | 恢复为 `1` |
| 退出遥控界面或重新初始化 | 清为 `0` |

*/


extern volatile uint8_t Lora_RxPending; /* 接收中断置1，Poll取走完整数据后清零 */
extern volatile uint8_t Lora_Timeout;   /* 超时事件锁存，收包不自动清零，由使用者清零 */
/* 最近有效回传的DWT时刻；每次超时也更新，以开始下一轮10秒检查。 */
extern volatile uint32_t Lora_LastRxCycles;
/* 仅供USART1中断调用：收字节/丢弃错误半帧，不操作LVGL或控制目标。 */
void RemoteControl_ReceiveByte(uint8_t byte);
void RemoteControl_ResetRx(void);

/* Init在调度器启动前调用一次；Set/Get由GUI使用，Poll仅由LoRa任务调用。
 * 内部暂停任务切换保护共享数据；Poll发帧时沿用UART发送保护，中断仍可响应。不能从ISR调用。 */
void RemoteControl_Init(void);
void RemoteControl_SetManualMode(uint8_t enabled); /* 进入即发首帧并启用周期发送；退出只发一帧零速后停发 */
/* dx向右、dy向下；radius为最大摇杆行程。下拉表示Yaw转向180度后前进。 */
void RemoteControl_SetStick(float dx, float dy, float radius);
void RemoteControl_Release(void); /* 速度归零，保留最后目标Yaw */
void RemoteControl_Stop(void);    /* 离开遥控页：速度归零、取消尚未发出的跳跃帧 */
void RemoteControl_RequestJump(void);
uint8_t RemoteControl_JumpBusy(void);
/* 返回完整数据副本，禁止把内部结构体指针跨任务共享。 */
RemoteCommand RemoteControl_GetCommand(void);
RemoteFeedback RemoteControl_GetFeedback(void);
float RemoteControl_SpeedToRpm(float speed_mps);
/* AA 55 | int16小端rpm*10 | int16小端Yaw*100 | jump(0/1) | 前7字节和的低8位。 */
int RemoteControl_BuildFrame(const RemoteCommand *command, uint8_t frame[REMOTE_FRAME_SIZE]);
/* LoRa任务每5ms调用：仅遥控页按SEND_PERIOD发送，不等待回传才发下一帧。
 * 松手/跳跃在下一周期帧生效；换页首帧/退出零速帧在下一次Poll发送。
 * 跳跃重复帧按UART成功发送推进，失败等下一周期；退出帧只尝试一次。
 * 1=已交串口，0=未发，-1=组帧或UART失败；UART成功不等于无线送达。
 * DWT使用无符号差值容许回绕，但Poll不能长时间停止；不支持深睡眠期间计时。 */
int RemoteControl_Poll(uint32_t now_ms);

#endif
