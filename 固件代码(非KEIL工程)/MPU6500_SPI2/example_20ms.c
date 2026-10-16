/* 独立测试main：替换工程原main使用，不要让两个main同时参与构建。
 * 独占TIM2作为0.1ms时钟，不使用中断；SysTick仍可供Delay使用。
 * 启动保持静止约1.4秒。循环中不放阻塞printf，先在调试器看变量。
 */
#include "stm32f10x.h"
#include "mpu6500_spi.h"

MPU6500_Raw imu_raw;
float angle[3];                  /* Pitch、Roll、Yaw，度 */
volatile float imu_dt_ms;        /* 正常约20，可用于核实实际周期 */
volatile uint8_t imu_angle_valid;
volatile uint8_t imu_fault;

static void TimeInit(void)
{
    RCC_ClocksTypeDef clocks;
    TIM_TimeBaseInitTypeDef timer;
    uint32_t tim_hz;
    RCC_GetClocksFreq(&clocks);
    tim_hz = clocks.PCLK1_Frequency;
    if (clocks.PCLK1_Frequency != clocks.HCLK_Frequency) tim_hz *= 2u;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    TIM_DeInit(TIM2);
    TIM_TimeBaseStructInit(&timer);
    timer.TIM_Prescaler = (uint16_t)(tim_hz/10000u-1u);
    timer.TIM_Period = 65535;
    TIM_TimeBaseInit(TIM2, &timer);
    TIM_Cmd(TIM2, ENABLE);
}

int main(void)
{
    uint16_t now, previous = 0, elapsed;
    uint8_t first = 1;
    int8_t result;
    /* 系统时钟由启动代码配置。初始化失败后，先检查接线/静止条件。 */
    if (!MPU6500_Init()) { imu_fault = 1; while (1) {} }
    TimeInit();
    while (1) {
        now = TIM_GetCounter(TIM2);
        elapsed = (uint16_t)(now-previous); /* 可跨一次16位计数回绕 */
        if (elapsed < 200u) continue;       /* 200*0.1ms=20ms */
        result = MPU6500_ReadRaw(&imu_raw);
        if (result < 0) { imu_fault = 1; imu_angle_valid = 0; while (1) {} }
        if (result == 0) continue;
        imu_dt_ms = (float)elapsed*0.1f;
        previous = now;
        imu_angle_valid = MPU6500_CalcAngles(&imu_raw, first ? 0.0f : imu_dt_ms, angle);
        first = 0;
        if (imu_angle_valid) {
            /* 可在这里提交CAN发送；不要加入阻塞式串口打印/Delay_ms。 */
        }
    }
}
