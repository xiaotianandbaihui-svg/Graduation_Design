#include "tle5012b.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_spi.h"
#include "Delay.h"
/* 仅限制等待次数，防止SPI异常时永久卡住，不是精确的微秒超时。 */
#define SPI_WAIT_LIMIT 10000u

/* 使用已有微秒延时，满足SSC片选及DATA换向间隔。 */
static void SSC_Gap(void)
{
    Delay_us(2);
}

static void CS(uint8_t id, int high)
{
    GPIO_TypeDef *port = id == 0u ? GPIOA : GPIOB;
    uint16_t pin = id == 0u ? GPIO_Pin_4 : GPIO_Pin_1;
    if (high) GPIO_SetBits(port, pin);
    else GPIO_ResetBits(port, pin);
}

/* 双向DATA模式：发命令时驱动PA7，接收时释放PA7，让编码器驱动DATA。 */
static void DATA_Output(int enable)
{
#if TLE5012B_RELEASE_MOSI
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = GPIO_Pin_7;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = enable ? GPIO_Mode_AF_PP : GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);
#else
    (void)enable;
#endif
}

/* 一个16位字的收发：直接使用SPI1标准库函数。 */
static TLE_Result SPI1_Word(uint16_t tx, uint16_t *rx)
{
    uint32_t wait = SPI_WAIT_LIMIT;
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
        if (--wait == 0u) return TLE_SPI_TIMEOUT;

    SPI_I2S_SendData(SPI1, tx);

    wait = SPI_WAIT_LIMIT;
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)
        if (--wait == 0u) return TLE_SPI_TIMEOUT;
    *rx = SPI_I2S_ReceiveData(SPI1);

    wait = SPI_WAIT_LIMIT;
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET)
        if (--wait == 0u) return TLE_SPI_TIMEOUT;
    return TLE_OK;
}

/* 16位命令+16位数据的CRC；SPI传输仍然是16位，CRC按协议逐字节算。 */
static uint8_t CRC8(uint16_t command, uint16_t data)
{
    uint32_t message = ((uint32_t)command << 16) | data;
    uint8_t crc = 0xFFu;
    uint8_t byte, bit;
    for (byte = 0; byte < 4u; ++byte) {
        crc ^= (uint8_t)(message >> 24);
        message <<= 8;
        for (bit = 0; bit < 8u; ++bit) {
            if (crc & 0x80u) crc = (uint8_t)((crc << 1) ^ 0x1Du);
            else crc = (uint8_t)(crc << 1);
        }
    }
    return (uint8_t)~crc;
}

/* 完成一个寄存器的SSC读/写，收到安全字后结束本次片选。 */
static TLE_Result Transfer(uint8_t id, uint16_t command, uint16_t *value)
{
    uint16_t data = *value, safety, discard;
    uint8_t response;
    TLE_Result result;
    int reading = (command & 0x8000u) != 0u;

    CS(id, 1);
    SSC_Gap();
    DATA_Output(1);
    CS(id, 0);
    SSC_Gap();

    result = SPI1_Word(command, &discard);
    if (result != TLE_OK) goto finish;
    if (!reading) {
        result = SPI1_Word(data, &discard);   /* 写：先把数据发给编码器 */
        if (result != TLE_OK) goto finish;
    }

    DATA_Output(0);
    SSC_Gap();
    if (reading) {
        result = SPI1_Word(0xFFFFu, &data);  /* 读：发送占位字以产生16个时钟 */
        if (result != TLE_OK) goto finish;
    }
    result = SPI1_Word(0xFFFFu, &safety);     /* 读、写都要接收安全字 */
    if (result != TLE_OK) goto finish;

    if (CRC8(command, data) != (uint8_t)safety) {
        result = TLE_CRC_ERROR;
        goto finish;
    }
    response = (uint8_t)((safety >> 8) & 0x0Fu);
    if (response != 0xEu && response != 0xDu && response != 0xBu && response != 0x7u) {
        result = TLE_RESPONSE_ERROR;
        goto finish;
    }
    /* 读STAT用于取得并清除历史状态，此次允许历史状态非正常。
     * 正常读角度和写寄存器，要求安全字的4个状态位全部为1。 */
    if (command != 0x8001u && (safety & 0xF000u) != 0xF000u) {
        result = TLE_STATUS_ERROR;
        goto finish;
    }
    if (reading) *value = data;

finish:
    SSC_Gap();
    CS(id, 1);
    DATA_Output(0);
    SSC_Gap();
    return result;
}

TLE_Result TLE5012B_ReadReg(uint8_t id, uint8_t reg, uint16_t *value)
{
    uint16_t command, data = 0u;
    TLE_Result result;
    if (id > 1u || reg > 0x11u || value == 0) return TLE_ARGUMENT_ERROR;
    /* RW=1，ND=1；05h..11h使用配置访问LOCK=1010b。 */
    command = (uint16_t)(0x8001u | ((uint16_t)reg << 4));
    if (reg >= 0x05u) command |= 0x5000u;
    result = Transfer(id, command, &data);
    if (result == TLE_OK) *value = data;
    return result;
}

TLE_Result TLE5012B_WriteReg(uint8_t id, uint8_t reg, uint16_t value)
{
    uint16_t command;
    /* 原始寄存器写：只接受ACSTAT和05h..0Fh，拒绝已知只读寄存器。
     * 各寄存器保留位/可写位及配置区CRC仍须按手册处理，见README。 */
    if (id > 1u || (reg != TLE_REG_ACSTAT && (reg < 0x05u || reg > 0x0Fu)))
        return TLE_ARGUMENT_ERROR;
    command = (uint16_t)(0x5001u | ((uint16_t)reg << 4));
    return Transfer(id, command, &value);
}

TLE_Result TLE5012B_ReadAngle(uint8_t id, float *degrees)
{
    uint16_t raw;
    TLE_Result result;
    if (degrees == 0) return TLE_ARGUMENT_ERROR;
    result = TLE5012B_ReadReg(id, TLE_REG_AVAL, &raw);
    if (result == TLE_OK)
        *degrees = (float)(raw & 0x7FFFu) * (360.0f / 32768.0f);/* bit15为读取状态，不属于15位角度。 */
    return result;
}

TLE_Result TLE5012B_ReadAngularSpeed(uint8_t id, float *deg_per_s)
{
    /* 与Infineon官方calculateAngleSpeed()使用的标称更新周期一致。 */
    static const float update_us[4] = {21.3f, 42.7f, 85.3f, 170.6f};
    uint16_t mod1, mod2, raw, angle_range;
    int32_t speed;
    float period_us, window;
    TLE_Result result;
    if (id > 1u || deg_per_s == 0) return TLE_ARGUMENT_ERROR;

    result = TLE5012B_ReadReg(id, TLE_REG_MOD1, &mod1);
    if (result != TLE_OK) return result;
    result = TLE5012B_ReadReg(id, TLE_REG_MOD2, &mod2);
    if (result != TLE_OK) return result;

    angle_range = (uint16_t)((mod2 & 0x7FF0u) >> 4); /* ANGRANGE，通常为128 */
    /* 不在未知外部时钟下套用内部时钟周期；DSPU暂停时也不返回旧速度。 */
    if ((mod1 & 0x0014u) != 0u || angle_range == 0u) return TLE_CONFIG_ERROR;
    period_us = update_us[(mod1 >> 14) & 3u];       /* FIR_MD，bit15..14 */
    window = (mod2 & 0x0004u) ? 3.0f : 2.0f;      /* PREDICT：开3，关2 */

    result = TLE5012B_ReadReg(id, TLE_REG_ASPD, &raw); /* 命令0x8031 */
    if (result != TLE_OK) return result;
    speed = (int32_t)(raw & 0x7FFFu);              /* bit15是读取状态 */
    if (speed & 0x4000) speed |= 0xFFFFC000;       /* bit14为1时，将bit31..15补1 */

    *deg_per_s = (float)speed * (360.0f / 32768.0f) *
                (128.0f / (float)angle_range) * 1000000.0f / (window * period_us);
    return TLE_OK;
}
float angle;
TLE_Result TLE5012B_Init(uint8_t id)
{
    uint16_t status, mod1, mod2;
    TLE_Result result = TLE5012B_ReadReg(id, TLE_REG_STAT, &status);
    if (result != TLE_OK) return result;
    result = TLE5012B_ReadReg(id, TLE_REG_MOD1, &mod1);
    if (result != TLE_OK) return result;
    result = TLE5012B_ReadReg(id, TLE_REG_MOD2, &mod2);
    if (result != TLE_OK) return result;
    /* 当前机械角换算要求完整360度量程（ANGRANGE=128），且DSPU未暂停。
     * 只检查，不擅自改写模块的配置及配置区CRC。 */
    if ((mod1 & 0x0010u) != 0u || ((mod2 & 0x7FF0u) >> 4) != 128u)
        return TLE_CONFIG_ERROR;
    return TLE5012B_ReadAngle(id, &angle);
}
