/* 板三STM32F103C8T6标准库裸机例程。与工程已有main二选一或合并内容。 */
#include "stm32f10x.h"
#include "jump.h"
#include "jump_tick.h"
#include "Serial.h"

#define JUMP_CMD_NONE  0u
#define JUMP_CMD_START 1u
#define JUMP_CMD_STOP  2u

/* 按键/CAN解析到一次新命令时赋值，或调试器手动修改。
 * 只保存最近的一条命令，不是队列；不要每轮循环重复写START。
 * STOP取消后续目标，保持已发出的舵机目标，不是断电急停。
 */
volatile uint8_t g_jump_command = JUMP_CMD_NONE;
volatile uint8_t g_jump_ready;
volatile int8_t g_jump_state = JUMP_IDLE;

static uint8_t TakeCommand(void)
{
    uint8_t command;
    /* Cortex-M3独占访问：原子取出并清零，不关闭中断。
     * 中断若在中途到来，独占写入失败后重读最新命令。 */
    do {
        command = __LDREXB(&g_jump_command);
    } while (__STREXB(JUMP_CMD_NONE, &g_jump_command) != 0u);
    return command;
}

int main(void)
{
    uint32_t now_ms, updated_ms;
    uint8_t command;

    /* 启动文件已经调用SystemInit；本例不重复调用。 */
    Serial_Init(); /* 逐飞驱动初始化失败时可输出诊断 */
    if (!JumpTick_Init()) {
        g_jump_state = JUMP_ERROR;
        for (;;) { }
    }
    if (Servo_Init(SERVO_PERIOD_US) != SERVO_OK ||
        imu963ra_init() != 0u || imu963ra_spi_error) {
        g_jump_state = JUMP_ERROR;
        for (;;) { } /* 初始化失败，不执行跳跃 */
    }
    (void)TakeCommand(); /* 丢弃初始化期间收到的命令；上电不自动跳跃 */
    updated_ms = 0;
    g_jump_ready = 1;

    for (;;) {
        command = TakeCommand();
        if (command == JUMP_CMD_STOP) {
            Jump_Stop();
            JumpTick_Stop();
            g_jump_state = JUMP_IDLE;
        } else if (command == JUMP_CMD_START && g_jump_state != JUMP_RUNNING) {
            /* 每次动作的起始时刻都是0；Start成功后才开启TIM4。 */
            g_jump_state = Jump_Start(0u);
            updated_ms = 0;
            if (g_jump_state == JUMP_RUNNING) JumpTick_Start();
            else JumpTick_Stop();
        }

        if (g_jump_state == JUMP_RUNNING) {
            now_ms = JumpTick_Millis(); /* CNT/10：从本次起跳计时起经过的毫秒 */
            if (now_ms == JUMP_TICK_EXPIRED) {
                Jump_Stop(); /* 超过6.5536s，不能把回绕后的CNT当作新时间 */
                JumpTick_Stop();
                g_jump_state = JUMP_ERROR;
            } else if (now_ms != updated_ms) {
                updated_ms = now_ms;
                g_jump_state = Jump_Update(now_ms);
                /* 正常结束或报错都停表清零，保留跳跃结果供查看。 */
                if (g_jump_state != JUMP_RUNNING) JumpTick_Stop();
            }
        }

        /* 在这里处理按键、CAN等短任务，不能长时间Delay或忙等。
         * SPI2与Jump接口都在主循环调用；其他调腿算法不能同时覆盖舵机目标。
         */
    }
}
