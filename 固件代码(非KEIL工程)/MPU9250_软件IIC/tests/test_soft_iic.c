/* 主机脚本从机：检查线上地址/位序、重复START、ACK/NACK、错误退出。
 * 这是协议逻辑检查，不模拟PCB上升时间或真实从机的所有电气行为。
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "soft_iic.h"
#include "iic_port.h"

static uint8_t sda, scl, answers[512], rising[512];
static size_t answer_count, answer_index, rising_count, frame[8];
static unsigned int starts, stops, stretch, stuck_scl, delay_us;

static void reset(void)
{
    sda = scl = 1;
    answer_count = answer_index = rising_count = 0;
    starts = stops = stretch = stuck_scl = delay_us = 0;
    memset(frame, 0, sizeof(frame));
}

static void answer(uint8_t value)
{
    assert(answer_count < sizeof(answers));
    answers[answer_count++] = value;
}

static void answer_byte(uint8_t value)
{
    unsigned int i;
    for (i = 0; i < 8u; ++i) {
        answer((uint8_t)((value >> (7u - i)) & 1u));
    }
}

void SDA(uint8_t level)
{
    if (scl && sda != level) {
        if (!level) {
            assert(starts < 8u);
            frame[starts++] = rising_count;
        } else {
            ++stops;
        }
    }
    sda = level;
}

void SCK(uint8_t level)
{
    if (level && !scl) {
        assert(rising_count < sizeof(rising));
        rising[rising_count++] = sda;
    }
    scl = level;
}

uint8_t SDA_Read(void)
{
    assert(answer_index < answer_count);
    return answers[answer_index++];
}

uint8_t SCK_Read(void)
{
    if (stuck_scl) return 0;
    if (stretch) {
        --stretch;
        return 0;
    }
    return scl;
}

void IIC_DelayUs(uint32_t us)
{
    delay_us += us;
}

static uint8_t sent(size_t offset)
{
    unsigned int i;
    uint8_t value = 0;
    for (i = 0; i < 8u; ++i) {
        assert(offset + i < rising_count);
        value = (uint8_t)((value << 1) | rising[offset + i]);
    }
    return value;
}

static void finished(void)
{
    assert(answer_index == answer_count);
    assert(sda == 1 && scl == 1);
}

int main(void)
{
    uint8_t data[2] = {0xA5, 0x5A};
    unsigned int nack_at, i;

    reset(); answer(1);
    assert(IIC_Init() == IIC_OK);
    finished();

    reset(); answer(0);
    assert(IIC_Init() == IIC_ERR_BUS_BUSY);
    finished();

    reset();
    answer(1); answer(0); answer(0); answer(0); answer(0); answer(1);
    assert(IIC_WriteRegs(0x68, 0x20, data, 2) == IIC_OK);
    assert(starts == 1 && stops == 1);
    assert(sent(0) == 0xD0 && sent(9) == 0x20);
    assert(sent(18) == 0xA5 && sent(27) == 0x5A);
    assert(rising[8] == 1 && rising[17] == 1); /* 释放SDA等从机ACK。 */
    finished();

    reset();
    answer(1); answer(0); answer(0); answer(1); answer(0);
    answer_byte(0x80); answer_byte(0x01); answer(1);
    assert(IIC_ReadRegs(0x68, 0x3B, data, 2) == IIC_OK);
    assert(starts == 2 && stops == 1);
    assert(sent(frame[0]) == 0xD0 && sent(frame[0] + 9) == 0x3B);
    assert(sent(frame[1]) == 0xD1);
    assert(rising[frame[1] + 17] == 0); /* 第一字节后主机ACK。 */
    assert(rising[frame[1] + 26] == 1); /* 最后一字节主机NACK。 */
    assert(data[0] == 0x80 && data[1] == 0x01);
    finished();

    reset();
    answer(1); answer(0); answer(0); answer(1); answer(0);
    answer_byte(0x00); answer(1);
    assert(IIC_ReadReg(0x69, 0x75, data) == IIC_OK);
    assert(data[0] == 0); /* 有效的0不能被当作通信失败。 */
    assert(sent(frame[0]) == 0xD2 && sent(frame[1]) == 0xD3);
    assert(rising[frame[1] + 17] == 1);
    finished();

    /* 地址、寄存器、数据的任何一个NACK都应终止，产生STOP。 */
    for (nack_at = 0; nack_at < 3u; ++nack_at) {
        reset(); answer(1);
        for (i = 0; i < nack_at; ++i) answer(0);
        answer(1); answer(1);
        assert(IIC_WriteReg(0x68, 0x6B, 1) == IIC_ERR_NACK);
        assert(starts == 1 && stops == 1);
        assert(rising_count == (nack_at + 1u) * 9u + 1u);
        finished();
    }

    reset(); answer(1); answer(0); answer(0); answer(1); answer(1); answer(1);
    assert(IIC_ReadReg(0x68, 0x75, data) == IIC_ERR_NACK);
    assert(starts == 2 && stops == 1);
    finished();

    reset(); answer(0);
    assert(IIC_ReadReg(0x68, 0x75, data) == IIC_ERR_BUS_BUSY);
    assert(starts == 0 && stops == 0);
    finished();

    reset(); stuck_scl = 1;
    assert(IIC_ReadReg(0x68, 0x75, data) == IIC_ERR_SCL_TIMEOUT);
    assert(delay_us >= 1000u && delay_us < 1100u);
    finished();

    reset(); stretch = 3; answer(1);
    assert(IIC_Init() == IIC_OK && stretch == 0);
    finished();

    reset(); answer(1); answer(0); answer(0); answer(0); answer(0);
    assert(IIC_WriteReg(0x68, 0x6B, 1) == IIC_ERR_STOP);
    finished();

    reset();
    assert(IIC_ReadReg(0xD0, 0x75, data) == IIC_ERR_ARGUMENT);
    assert(IIC_ReadRegs(0x68, 0x75, data, 0) == IIC_ERR_ARGUMENT);
    assert(IIC_ReadReg(0x68, 0x75, NULL) == IIC_ERR_ARGUMENT);
    assert(IIC_WriteRegs(0x68, 0x75, NULL, 1) == IIC_ERR_ARGUMENT);
    assert(starts == 0 && stops == 0);
    puts("soft_iic: protocol, ACK/NACK, zero data, timeout and argument checks PASS");
    return 0;
}
