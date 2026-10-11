#include "motor_port.h"
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_exti.h"
#include "misc.h"

/* 依据板1旧版网络表。修改接线时，应同时修改本文件和README的引脚表。
 * 不使用PA0；SPI1、TIM2、TIM3、TIM4由本例独占。 */
#define FAULT_LINES  (EXTI_Line5 | EXTI_Line8)
#define PWM_PERIOD  (72000000u / MOTOR_PWM_HZ)

static volatile uint32_t port_fault;
static volatile uint32_t last_feed;
static volatile uint8_t output_mask;
static volatile uint8_t killed = 1u;
static uint8_t last_vector[2] = {255u, 255u};
static uint8_t ready;

/* 正向相矢量次序：A+B-、A+C-、B+C-、B+A-、C+A-、C+B-。 */
static const uint8_t high_phase[6] = {0, 0, 1, 1, 2, 2};
static const uint8_t low_phase[6]  = {1, 2, 2, 0, 0, 1};

static uint32_t enter_critical(void)
{
    uint32_t state = __get_PRIMASK();
    __disable_irq();
    return state;
}

static void leave_critical(uint32_t state)
{
    __set_PRIMASK(state);
}

static void gpio_mode(GPIO_TypeDef *port, uint16_t pins, GPIOMode_TypeDef mode)
{
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = pins;
    gpio.GPIO_Mode = mode;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(port, &gpio);
}

uint32_t MotorPort_Ticks(void)
{
    return DWT->CYCCNT;
}

void MotorPort_DelayUs(uint32_t us)
{
    uint32_t start = MotorPort_Ticks();
    uint32_t ticks = us * MOTOR_CPU_TICKS_PER_US;
    while ((uint32_t)(MotorPort_Ticks() - start) < ticks) { }
}

static void high_pins_mode(uint8_t id, GPIOMode_TypeDef mode)
{
    if (id == 0u) gpio_mode(GPIOA, GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3, mode);
    else gpio_mode(GPIOB, GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9, mode);
}

static void low_pins_off(uint8_t id)
{
    if (id == 0u) GPIO_ResetBits(GPIOB, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14);
    else {
        GPIO_ResetBits(GPIOB, GPIO_Pin_15 | GPIO_Pin_3);
        GPIO_ResetBits(GPIOA, GPIO_Pin_15);
    }
}

static void low_phase_on(uint8_t id, uint8_t phase)
{
    if (id == 0u) GPIO_SetBits(GPIOB, (uint16_t)(GPIO_Pin_12 << phase));
    else if (phase == 0u) GPIO_SetBits(GPIOB, GPIO_Pin_15);
    else if (phase == 1u) GPIO_SetBits(GPIOB, GPIO_Pin_3);
    else GPIO_SetBits(GPIOA, GPIO_Pin_15);
}

/* 调用者持有短临界区。把PWM引脚切回输出低，明确消除旧PWM电平，
 * 而不是仅仅往CCR的预装载寄存器写0、等待未来的更新事件。 */
static void force_off(uint8_t id)
{
    TIM_TypeDef *tim = (id == 0u) ? TIM2 : TIM4;
    if (id == 0u) GPIO_ResetBits(GPIOA, GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3);
    else GPIO_ResetBits(GPIOB, GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9);
    high_pins_mode(id, GPIO_Mode_Out_PP);
    low_pins_off(id);
    TIM_Cmd(tim, DISABLE);
    TIM_SetCompare2(tim, 0u);
    TIM_SetCompare3(tim, 0u);
    TIM_SetCompare4(tim, 0u);
    TIM_GenerateEvent(tim, TIM_EventSource_Update);
    last_vector[id] = 255u;
    output_mask &= (uint8_t)~(1u << id);
}

void MotorPort_Stop(uint8_t id)
{
    uint32_t state;
    if (!ready || id > 1u) return;
    state = enter_critical();
    force_off(id);
    leave_critical(state);
}

void MotorPort_Kill(uint32_t fault)
{
    uint32_t state = enter_critical();
    port_fault |= fault;
    killed = 1u;
    if (ready) {
        GPIO_ResetBits(GPIOB, GPIO_Pin_0); /* 共享ENABLE：先关两个驱动器 */
        EXTI->IMR &= ~FAULT_LINES;
        force_off(0u);
        force_off(1u);
    }
    leave_critical(state);
}

uint32_t MotorPort_Fault(void)
{
    return port_fault;
}

void MotorPort_Feed(void)
{
    last_feed = MotorPort_Ticks();
}

static void pwm_init(TIM_TypeDef *tim)
{
    TIM_TimeBaseInitTypeDef base;
    TIM_OCInitTypeDef oc;
    TIM_TimeBaseStructInit(&base);
    base.TIM_Prescaler = 0u;
    base.TIM_Period = PWM_PERIOD - 1u;
    base.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(tim, &base);
    TIM_OCStructInit(&oc);
    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    oc.TIM_Pulse = 0u;
    TIM_OC2Init(tim, &oc);
    TIM_OC3Init(tim, &oc);
    TIM_OC4Init(tim, &oc);
    TIM_OC2PreloadConfig(tim, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(tim, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(tim, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(tim, ENABLE);
    TIM_GenerateEvent(tim, TIM_EventSource_Update);
}

int MotorPort_Init(void)
{
    RCC_ClocksTypeDef clocks;
    SPI_InitTypeDef spi;
    TIM_TimeBaseInitTypeDef base;
    EXTI_InitTypeDef exti;
    NVIC_InitTypeDef irq;
    uint32_t irq_state;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                          RCC_APB2Periph_AFIO | RCC_APB2Periph_SPI1, ENABLE);
    GPIO_ResetBits(GPIOB, GPIO_Pin_0);
    gpio_mode(GPIOB, GPIO_Pin_0, GPIO_Mode_Out_PP);
    /* 即使只控制电机1，也处理电机2的所有门极输入，防止共享EN误开启。 */
    GPIO_ResetBits(GPIOA, GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_15);
    GPIO_ResetBits(GPIOB, GPIO_Pin_3 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 |
                         GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
    GPIO_PinRemapConfig(GPIO_Remap_SPI1, DISABLE);
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, DISABLE);
    GPIO_PinRemapConfig(GPIO_Remap_TIM4, DISABLE);
    /* SWJ_CFG读值未定义，部分SPL重映射函数会改写它；最后明确设置一次。 */
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); /* 保留PA13/PA14 SWD */
    high_pins_mode(0u, GPIO_Mode_Out_PP);
    high_pins_mode(1u, GPIO_Mode_Out_PP);
    gpio_mode(GPIOA, GPIO_Pin_15, GPIO_Mode_Out_PP);
    gpio_mode(GPIOB, GPIO_Pin_3 | GPIO_Pin_12 | GPIO_Pin_13 |
                     GPIO_Pin_14 | GPIO_Pin_15, GPIO_Mode_Out_PP);
    killed = 1u;
    ready = 0u;
    port_fault = 0u;
    output_mask = 0u;
    EXTI->IMR &= ~FAULT_LINES;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 |
                          RCC_APB1Periph_TIM4, ENABLE);
    TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
    TIM_Cmd(TIM3, DISABLE);
    TIM_DeInit(TIM2);
    TIM_DeInit(TIM3);
    TIM_DeInit(TIM4);
    SystemCoreClockUpdate();
    RCC_GetClocksFreq(&clocks);
    if (clocks.HCLK_Frequency != 72000000u ||
        clocks.PCLK1_Frequency != 36000000u ||
        clocks.PCLK2_Frequency != 72000000u) return 0;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    pwm_init(TIM2);
    pwm_init(TIM4);
    ready = 1u;
    MotorPort_Stop(0u);
    MotorPort_Stop(1u);

    GPIO_SetBits(GPIOA, GPIO_Pin_4);
    GPIO_SetBits(GPIOB, GPIO_Pin_1);
    gpio_mode(GPIOA, GPIO_Pin_4, GPIO_Mode_Out_PP);
    gpio_mode(GPIOB, GPIO_Pin_1, GPIO_Mode_Out_PP);
    gpio_mode(GPIOA, GPIO_Pin_5, GPIO_Mode_AF_PP);
    gpio_mode(GPIOA, GPIO_Pin_6 | GPIO_Pin_7, GPIO_Mode_IN_FLOATING);
    SPI_I2S_DeInit(SPI1);
    SPI_StructInit(&spi);
    spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    spi.SPI_Mode = SPI_Mode_Master;
    spi.SPI_DataSize = SPI_DataSize_16b;
    spi.SPI_CPOL = SPI_CPOL_Low;
    spi.SPI_CPHA = SPI_CPHA_2Edge;      /* Mode 1：上升沿更新，下降沿采样 */
    spi.SPI_NSS = SPI_NSS_Soft;
    spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16; /* 72M/16=4.5MHz */
    spi.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_Init(SPI1, &spi);
    SPI_Cmd(SPI1, ENABLE);

    gpio_mode(GPIOA, GPIO_Pin_8, GPIO_Mode_IPU);
    gpio_mode(GPIOB, GPIO_Pin_5, GPIO_Mode_IPU);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource8);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource5);
    exti.EXTI_Line = FAULT_LINES;
    exti.EXTI_Mode = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = EXTI_Trigger_Falling;
    exti.EXTI_LineCmd = ENABLE;
    irq_state = enter_critical();
    EXTI_Init(&exti);
    EXTI->IMR &= ~FAULT_LINES; /* 驱动器唤醒完成后才启用 */
    EXTI_ClearITPendingBit(FAULT_LINES);
    leave_critical(irq_state);
    irq.NVIC_IRQChannel = EXTI9_5_IRQn;
    irq.NVIC_IRQChannelPreemptionPriority = 0u;
    irq.NVIC_IRQChannelSubPriority = 0u;
    irq.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&irq);

    TIM_TimeBaseStructInit(&base);
    base.TIM_Prescaler = 71u;
    base.TIM_Period = 99u;             /* 每100us检查一次软件更新是否失联 */
    TIM_TimeBaseInit(TIM3, &base);
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    irq.NVIC_IRQChannel = TIM3_IRQn;
    irq.NVIC_IRQChannelPreemptionPriority = 1u;
    irq.NVIC_IRQChannelSubPriority = 0u;
    NVIC_Init(&irq);
    MotorPort_Feed();
    TIM_Cmd(TIM3, ENABLE);
    return 1;
}

int MotorPort_Enable(void)
{
    uint32_t state;
    uint32_t faults = 0u;
    if (!ready || port_fault != 0u) return 0;
    GPIO_SetBits(GPIOB, GPIO_Pin_0);
    MotorPort_DelayUs(2000u);          /* DRV8323H唤醒最大1ms，留余量 */
    state = enter_critical();
    EXTI_ClearITPendingBit(FAULT_LINES);
    if (!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8)) faults |= PORT_FAULT_DRV1;
    if (!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5)) faults |= PORT_FAULT_DRV2;
    if (faults != 0u) {
        leave_critical(state);
        MotorPort_Kill(faults);
        return 0;
    }
    killed = 0u;
    MotorPort_Feed();
    EXTI->IMR |= FAULT_LINES;
    leave_critical(state);
    return 1;
}

static int spi_word(uint16_t tx, uint16_t *rx)
{
    uint32_t start = MotorPort_Ticks();
    const uint32_t timeout = 100u * MOTOR_CPU_TICKS_PER_US;
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) {
        if ((uint32_t)(MotorPort_Ticks() - start) > timeout) return 0;
    }
    SPI_I2S_SendData(SPI1, tx);
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET) {
        if ((uint32_t)(MotorPort_Ticks() - start) > timeout) return 0;
    }
    *rx = SPI_I2S_ReceiveData(SPI1);
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET) {
        if ((uint32_t)(MotorPort_Ticks() - start) > timeout) return 0;
    }
    return 1;
}

int MotorPort_SSCRead(uint8_t id, uint16_t command, uint16_t *data, uint16_t *safety)
{
    GPIO_TypeDef *cs_port = (id == 0u) ? GPIOA : GPIOB;
    uint16_t cs_pin = (id == 0u) ? GPIO_Pin_4 : GPIO_Pin_1;
    uint16_t discard;
    int ok = 0;
    if (!ready || id > 1u || data == 0 || safety == 0) return 0;
    MotorPort_DelayUs(1u);             /* CS高电平间隔 >=600ns */
    gpio_mode(GPIOA, GPIO_Pin_7, GPIO_Mode_AF_PP);
    GPIO_ResetBits(cs_port, cs_pin);
    MotorPort_DelayUs(1u);
    if (!spi_word(command, &discard)) goto finish;
    /* 器件DATA是双向线。命令结束后释放MOSI，避免与传感器输出争用。
     * SPI仍为全双工主机，发送dummy只用于产生SCK，数据从PA6读回。 */
    gpio_mode(GPIOA, GPIO_Pin_7, GPIO_Mode_IN_FLOATING);
    MotorPort_DelayUs(1u);             /* 换向间隔 >=130ns */
    if (!spi_word(0xffffu, data)) goto finish;
    if (!spi_word(0xffffu, safety)) goto finish;
    ok = 1;
finish:
    if (!ok) SPI_Cmd(SPI1, DISABLE);   /* 故障后由Motor_Init显式重新初始化 */
    MotorPort_DelayUs(1u);
    GPIO_SetBits(cs_port, cs_pin);
    gpio_mode(GPIOA, GPIO_Pin_7, GPIO_Mode_IN_FLOATING);
    MotorPort_DelayUs(1u);
    return ok;
}

void MotorPort_Apply(uint8_t id, uint8_t vector, uint16_t duty)
{
    uint32_t state;
    uint16_t compare;
    TIM_TypeDef *tim;
    if (!ready || id > 1u || vector > 5u) return;
    if (duty == 0u) { MotorPort_Stop(id); return; }
    if (duty > (uint16_t)MOTOR_MAX_DUTY) duty = (uint16_t)MOTOR_MAX_DUTY;
    tim = (id == 0u) ? TIM2 : TIM4;
    compare = (uint16_t)((PWM_PERIOD * duty) / 1000u);
    state = enter_critical();
    if (killed || port_fault != 0u) { leave_critical(state); return; }
    if (last_vector[id] != vector) {
        force_off(id);
        MotorPort_DelayUs(2u);         /* 扇区切换先全关；不是靠延迟产生20kHz PWM */
        if (high_phase[vector] == 0u) TIM_SetCompare2(tim, compare);
        if (high_phase[vector] == 1u) TIM_SetCompare3(tim, compare);
        if (high_phase[vector] == 2u) TIM_SetCompare4(tim, compare);
        TIM_GenerateEvent(tim, TIM_EventSource_Update);
        TIM_SetCounter(tim, 0u);
        low_phase_on(id, low_phase[vector]);
        high_pins_mode(id, GPIO_Mode_AF_PP);
        TIM_Cmd(tim, ENABLE);
        last_vector[id] = vector;
        output_mask |= (uint8_t)(1u << id);
    } else {
        /* 同扇区仅改变CCR预装载，下个PWM周期生效，不重启计数器。 */
        if (high_phase[vector] == 0u) TIM_SetCompare2(tim, compare);
        if (high_phase[vector] == 1u) TIM_SetCompare3(tim, compare);
        if (high_phase[vector] == 2u) TIM_SetCompare4(tim, compare);
    }
    leave_critical(state);
}

void EXTI9_5_IRQHandler(void)
{
    uint32_t pending = EXTI->PR & EXTI->IMR & FAULT_LINES;
    uint32_t faults = 0u;
    if (pending & EXTI_Line8) faults |= PORT_FAULT_DRV1;
    if (pending & EXTI_Line5) faults |= PORT_FAULT_DRV2;
    if (pending != 0u) {
        EXTI_ClearITPendingBit(pending);
        MotorPort_Kill(faults);       /* 硬件自动重试也不让软件自动重新转动 */
    }
    /* 若工程还使用EXTI6/7/9，在此合并对应分支，不能另定义同名函数。 */
}

void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        if (output_mask != 0u &&
            (uint32_t)(MotorPort_Ticks() - last_feed) >
            MOTOR_TIMEOUT_US * MOTOR_CPU_TICKS_PER_US) {
            MotorPort_Kill(PORT_FAULT_TIMEOUT);
        }
    }
}
