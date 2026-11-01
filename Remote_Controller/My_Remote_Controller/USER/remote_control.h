#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include <stdint.h>

/* V1配置入口；两端LoRa须使用相同信道/空中速率，并工作在透明传输模式。 */
#define REMOTE_UART_BAUD          115200u
#define REMOTE_SEND_PERIOD_MS     100u /* 根据LoRa空中速率/AUX实测调整，UART速率不是空中速率 */
#define REMOTE_WHEEL_DIAMETER_M   0.064f
#define REMOTE_MAX_SPEED_MPS      1.0f /* 摇杆拉满时的目标速度，调试时可调小 */
#define REMOTE_STICK_DEAD_ZONE    0.12f /* 半径内12%为死区，防止中点附近Yaw乱跳 */
#define REMOTE_YAW_SIGN           1.0f /* 默认向右为+90度；与小车Yaw正方向不符时改-1 */
#define REMOTE_YAW_OFFSET_DEG     0.0f /* 摇杆正上方对应的绝对Yaw，须与主控姿态参考系一致 */
#define REMOTE_JUMP_FRAMES        3u   /* 连续发3帧jump=1，随后发0重新允许下一次跳跃 */
#define REMOTE_FRAME_SIZE         8u
#define REMOTE_FEEDBACK_SIZE      7u
#define REMOTE_RESPONSE_TIMEOUT_MS 500u  /* 一帧发出后等响应的上限，按LoRa往返耗时调整 */
#define REMOTE_LORA_TIMEOUT_MS    10000u /* 仅遥控页：10秒无有效回传则停止目标、置超时标志 */

typedef struct {
    float speed_mps;
    float yaw_deg;
    uint8_t jump;
} RemoteCommand;

typedef struct {
    float speed_mps; /* 两轮平均转速换算，轮径由REMOTE_WHEEL_DIAMETER_M确定 */
    float yaw_deg;
    uint8_t valid;     /* 当前可显示实测值；重新进入遥控页后等待新数据 */
    uint8_t timed_out; /* 遥控页当前掉线；有效回传或换页时清零，用于界面提示 */
} RemoteFeedback;

extern volatile uint8_t Lora_RxPending; /* 接收中断置1，Poll取走完整数据后清零 */
extern volatile uint8_t Lora_Timeout;   /* 超时事件锁存，收包不自动清零，由使用者清零 */
/* 最近有效回传的DWT时刻；每次超时也更新，以开始下一轮10秒检查。 */
extern volatile uint32_t Lora_LastRxCycles;
/* 仅供USART1中断调用：收字节/丢弃错误半帧，不操作LVGL或控制目标。 */
void RemoteControl_ReceiveByte(uint8_t byte);
void RemoteControl_ResetRx(void);

/* Init在调度器启动前调用一次；Set/Get由GUI使用，Poll仅由LoRa任务调用。
 * 内部短暂暂停任务切换来保护共享数据，这些接口不能从ISR调用。 */
void RemoteControl_Init(void);
void RemoteControl_SetManualMode(uint8_t enabled); /* 换页时调用：只有遥控页启用10秒检测 */
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
/* 高频调用：发一帧后等待正常回传，超时才重试；空闲时按SEND_PERIOD发送。
 * 松手/跳跃/换页优先在本轮等待结束后发送；1=已交串口，0=未发，-1=UART失败。
 * DWT使用无符号差值容许回绕，但Poll不能长时间停止；不支持深睡眠期间计时。 */
int RemoteControl_Poll(uint32_t now_ms);

#endif
