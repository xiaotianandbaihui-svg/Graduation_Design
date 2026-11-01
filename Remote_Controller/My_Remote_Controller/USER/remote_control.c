#include "remote_control.h"
#include "usart.h"
#include <math.h>
#include "FreeRTOS.h"
#include "task.h"

#define REMOTE_PI 3.14159265358979323846f

static RemoteCommand command;
static uint32_t last_send_ms;
static uint8_t send_now;
/* 0=空闲，1=还需发送jump=0，2及以上=剩余jump=1帧数+1。 */
static uint8_t jump_frames;
static RemoteFeedback feedback;
static uint8_t feedback_frame[REMOTE_FEEDBACK_SIZE], feedback_count;
static volatile int16_t received_speed, received_yaw;
volatile uint8_t Lora_RxPending;
volatile uint8_t Lora_Timeout;
volatile uint32_t Lora_LastRxCycles;
static uint8_t waiting_response, sent_jump;
static uint32_t sent_cycles;
static uint8_t manual_mode; /* 只控制遥控页的10秒掉线检测，不影响收包/响应重试 */

void RemoteControl_ResetRx(void)
{
    feedback_count = 0u;
}

void RemoteControl_ReceiveByte(uint8_t byte)
{
    uint8_t i, sum = 0u;
    int16_t yaw;
    feedback_frame[feedback_count++] = byte;
    if (feedback_count < REMOTE_FEEDBACK_SIZE) return;
    for (i = 0u; i < REMOTE_FEEDBACK_SIZE - 1u; ++i)
        sum = (uint8_t)(sum + feedback_frame[i]);
    yaw = (int16_t)((uint16_t)feedback_frame[4] | ((uint16_t)feedback_frame[5] << 8));
    if (feedback_frame[0] == 0x55u && feedback_frame[1] == 0xAAu &&
        sum == feedback_frame[6] && yaw >= -18000 && yaw <= 18000) {
        received_speed = (int16_t)((uint16_t)feedback_frame[2] |
                                  ((uint16_t)feedback_frame[3] << 8));
        received_yaw = yaw;
        Lora_LastRxCycles = DWT->CYCCNT;
        Lora_RxPending = 1u;
        feedback_count = 0u;
        return;
    }
    /* 日志、错包、丢字节都逐字节重找帧头；无效数据不刷新超时。 */
    for (i = 1u; i < REMOTE_FEEDBACK_SIZE; ++i)
        feedback_frame[i - 1u] = feedback_frame[i];
    feedback_count = REMOTE_FEEDBACK_SIZE - 1u;
}

static float WrapYaw(float degrees)
{
    float result = fmodf(degrees + 180.0f, 360.0f);
    if (result < 0.0f) result += 360.0f;
    return result - 180.0f;
}

void RemoteControl_Init(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    Lora_RxPending = Lora_Timeout = 0u;
    Lora_LastRxCycles = DWT->CYCCNT; /* uart_init已启用DWT，不清公共计数器 */
    feedback_count = 0u;
    __set_PRIMASK(mask);
    feedback.speed_mps = feedback.yaw_deg = 0.0f;
    feedback.valid = feedback.timed_out = 0u;
    manual_mode = 0u;
    waiting_response = 0u;
    command.speed_mps = 0.0f;
    command.yaw_deg = WrapYaw(REMOTE_YAW_OFFSET_DEG);
    command.jump = 0u;
    jump_frames = 0u;
    last_send_ms = 0u;
    send_now = 1u;
}

float RemoteControl_SpeedToRpm(float speed_mps)
{
    return speed_mps * 60.0f / (REMOTE_PI * REMOTE_WHEEL_DIAMETER_M);
}

void RemoteControl_SetManualMode(uint8_t enabled)
{
    uint32_t mask;
    enabled = enabled != 0u;
    vTaskSuspendAll();
    if (manual_mode != enabled) {
        manual_mode = enabled;
        feedback.valid = feedback.timed_out = 0u;
        if (enabled) {
            /* 重新进入遥控页从现在计10秒，不消费进入前留下的旧回传。 */
            mask = __get_PRIMASK();
            __disable_irq();
            Lora_LastRxCycles = DWT->CYCCNT;
            Lora_RxPending = 0u;
            __set_PRIMASK(mask);
        }
    }
    (void)xTaskResumeAll();
}

void RemoteControl_SetStick(float dx, float dy, float radius)
{
    float amount, speed = 0.0f, yaw = 0.0f;
    if (!isfinite(dx) || !isfinite(dy) || !isfinite(radius) || radius <= 0.0f) {
        RemoteControl_Release();
        return;
    }
    amount = sqrtf(dx * dx + dy * dy) / radius;
    if (amount > 1.0f) amount = 1.0f;
    if (amount > REMOTE_STICK_DEAD_ZONE) {
        speed = REMOTE_MAX_SPEED_MPS *
                (amount - REMOTE_STICK_DEAD_ZONE) / (1.0f - REMOTE_STICK_DEAD_ZONE);
        yaw = WrapYaw(REMOTE_YAW_OFFSET_DEG + REMOTE_YAW_SIGN *
                      atan2f(dx, -dy) * (180.0f / REMOTE_PI));
    }
    /* 仅在提交完整目标时暂停任务切换；中断照常响应。 */
    vTaskSuspendAll();
    if (!manual_mode || !feedback.valid) command.speed_mps = 0.0f;
    else if (amount <= REMOTE_STICK_DEAD_ZONE) {
        if (command.speed_mps != 0.0f) {
            command.speed_mps = 0.0f;
            send_now = 1u;
        }
    } else {
        command.speed_mps = speed;
        command.yaw_deg = yaw;
    }
    (void)xTaskResumeAll();
}

void RemoteControl_Release(void)
{
    vTaskSuspendAll();
    command.speed_mps = 0.0f;
    send_now = 1u;
    (void)xTaskResumeAll();
}

void RemoteControl_Stop(void)
{
    vTaskSuspendAll();
    RemoteControl_Release(); /* FreeRTOS允许嵌套暂停，最外层恢复后才会切换任务。 */
    command.jump = 0u;
    jump_frames = jump_frames ? 1u : 0u;
    (void)xTaskResumeAll();
}

void RemoteControl_RequestJump(void)
{
    vTaskSuspendAll();
    if (manual_mode && feedback.valid && !jump_frames) {
        RemoteControl_Release();
        jump_frames = REMOTE_JUMP_FRAMES + 1u;
        command.jump = 1u;
    }
    (void)xTaskResumeAll();
}

uint8_t RemoteControl_JumpBusy(void)
{
    uint8_t busy;
    vTaskSuspendAll();
    busy = jump_frames != 0u;
    (void)xTaskResumeAll();
    return busy;
}

RemoteCommand RemoteControl_GetCommand(void)
{
    RemoteCommand copy;
    vTaskSuspendAll();
    copy = command;
    (void)xTaskResumeAll();
    return copy;
}

RemoteFeedback RemoteControl_GetFeedback(void)
{
    RemoteFeedback copy;
    vTaskSuspendAll();
    copy = feedback;
    (void)xTaskResumeAll();
    return copy;
}

int RemoteControl_BuildFrame(const RemoteCommand *c, uint8_t frame[REMOTE_FRAME_SIZE])
{
    float scaled;
    int16_t speed, yaw;
    uint8_t i, sum = 0u;
    if (!isfinite(c->speed_mps) || !isfinite(c->yaw_deg) || c->jump > 1u) return 0;
    scaled = RemoteControl_SpeedToRpm(c->speed_mps) * 10.0f;
    if (scaled > 32767.0f) scaled = 32767.0f;
    if (scaled < -32768.0f) scaled = -32768.0f;
    speed = (int16_t)(scaled + (scaled >= 0.0f ? 0.5f : -0.5f));
    scaled = WrapYaw(c->yaw_deg) * 100.0f;
    yaw = (int16_t)(scaled + (scaled >= 0.0f ? 0.5f : -0.5f));
    frame[0] = 0xAAu;
    frame[1] = 0x55u;
    frame[2] = (uint8_t)speed;
    frame[3] = (uint8_t)((uint16_t)speed >> 8);
    frame[4] = (uint8_t)yaw;
    frame[5] = (uint8_t)((uint16_t)yaw >> 8);
    frame[6] = c->jump;
    for (i = 0u; i < REMOTE_FRAME_SIZE - 1u; ++i) sum = (uint8_t)(sum + frame[i]);
    frame[7] = sum;
    return 1;
}

/* 由Poll短暂暂停任务切换后调用，只处理状态并生成本帧，不等待UART。 */
static int PrepareFrame(uint32_t now_ms, uint8_t frame[REMOTE_FRAME_SIZE])
{
    uint8_t received, timed_out;
    int16_t speed, yaw;
    uint32_t now, mask = __get_PRIMASK();

    __disable_irq();
    now = DWT->CYCCNT;
    timed_out = manual_mode && (uint32_t)(now - Lora_LastRxCycles) >=
                (SystemCoreClock / 1000u) * REMOTE_LORA_TIMEOUT_MS;
    received = Lora_RxPending;
    speed = received_speed; yaw = received_yaw;
    Lora_RxPending = 0u;
    if (timed_out) {
        Lora_LastRxCycles = now; /* 每10秒重新检查一次，标志由使用者清零 */
        Lora_Timeout = 1u;
        feedback_count = 0u;
        received = 0u;
    }
    __set_PRIMASK(mask);

    if (timed_out) {
        feedback.valid = 0u;
        feedback.timed_out = 1u; /* 当前链路状态，与可由外部清零的超时事件标志分开 */
        RemoteControl_Stop(); /* 取消尚未完成的跳跃，下一帧发送零速度及jump=0 */
        waiting_response = 0u;
    }
    if (received) {
        feedback.speed_mps = speed * 0.1f * (REMOTE_PI * REMOTE_WHEEL_DIAMETER_M) / 60.0f;
        feedback.yaw_deg = yaw * 0.01f;
        feedback.valid = 1u;
        feedback.timed_out = 0u;
        /* 普通实测帧就是响应；只有收到响应才推进跳跃脉冲，不以UART发完代替。 */
        if (waiting_response) {
            if ((sent_jump && jump_frames > 1u) || (!sent_jump && jump_frames == 1u))
                --jump_frames;
            waiting_response = 0u;
        }
    }
    if (waiting_response) {
        if ((uint32_t)(now - sent_cycles) <
            (SystemCoreClock / 1000u) * REMOTE_RESPONSE_TIMEOUT_MS) return 0;
        waiting_response = 0u;
        send_now = 1u; /* 响应丢失后发送最新目标，不能一直卡在等待状态 */
    }
    if (!send_now && (uint32_t)(now_ms - last_send_ms) < REMOTE_SEND_PERIOD_MS) return 0;
    last_send_ms = now_ms;
    send_now = 0u;
    command.jump = jump_frames > 1u ? 1u : 0u;
    if (!RemoteControl_BuildFrame(&command, frame)) return -1;
    sent_jump = command.jump;
    waiting_response = 1u;
    return 1;
}

int RemoteControl_Poll(uint32_t now_ms)
{
    uint8_t frame[REMOTE_FRAME_SIZE];
    int result;
    vTaskSuspendAll();
    result = PrepareFrame(now_ms, frame);
    (void)xTaskResumeAll();
    if (result != 1) return result;
    sent_cycles = DWT->CYCCNT;
    /* 即使UART只发出半帧，也先留出一轮响应时间，避免连续塞数据。
     * UART内部仍保留完整帧的发送保护；不持有上述共享数据保护等待收包。 */
    return uart_send_bytes(frame, sizeof(frame)) ? 1 : -1;
}
