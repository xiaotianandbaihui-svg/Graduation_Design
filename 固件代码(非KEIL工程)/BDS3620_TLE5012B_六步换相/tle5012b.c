#include "tle5012b.h"
#include "motor_port.h"

#define TLE_CMD_READ_STAT  0x8001u  /* RW=1，ADDR=0，ND=1，附安全字 */
#define TLE_CMD_READ_AVAL  0x8021u  /* RW=1，ADDR=2，ND=1 */

uint8_t TLE5012B_CRC8(const uint8_t *bytes, uint32_t length)
{
    uint8_t crc = 0xffu;
    uint32_t i;
    uint8_t bit;
    for (i = 0; i < length; ++i) {
        crc ^= bytes[i];
        for (bit = 0; bit < 8u; ++bit) {
            crc = (uint8_t)((crc & 0x80u) ? (((uint32_t)crc << 1) ^ 0x1du) : ((uint32_t)crc << 1));
        }
    }
    return (uint8_t)~crc;
}
/*CRC校验的值是command和data*/
TLE_Result TLE5012B_CheckFrame(uint16_t command, uint16_t data,
                              uint16_t safety, int check_status)
{
    uint8_t bytes[4];
    uint8_t response;
    bytes[0] = (uint8_t)(command >> 8);
    bytes[1] = (uint8_t)command;
    bytes[2] = (uint8_t)(data >> 8);
    bytes[3] = (uint8_t)data;
    /* CRC 覆盖命令和数据，高字节在前；不把安全字高字节加入 CRC。
     * 与 Infineon 官方 TLE5012b::checkSafety() 一致。 */
    if (TLE5012B_CRC8(bytes, 4u) != (uint8_t)safety) return TLE_CRC_ERROR;
    response = (uint8_t)((safety >> 8) & 0x0fu);
    if (response != 0x0eu && response != 0x0du &&
        response != 0x0bu && response != 0x07u) return TLE_RESPONSE_ERROR;
    if (check_status) {
        if (!(safety & 0x8000u)) return TLE_RESET_ERROR;
        if (!(safety & 0x4000u)) return TLE_SYSTEM_ERROR;
        if (!(safety & 0x2000u)) return TLE_INTERFACE_ERROR;
        if (!(safety & 0x1000u)) return TLE_ANGLE_ERROR;
    }
    return TLE_OK;
}

TLE_Result TLE5012B_Init(uint8_t id)
{
    uint16_t data, safety, raw;
    TLE_Result result;
    if (id > 1u) return TLE_BAD_ARGUMENT;
    MotorPort_DelayUs(10000u);
    if (!MotorPort_SSCRead(id, TLE_CMD_READ_STAT, &data, &safety)) return TLE_BUS_ERROR;
    /* 上电时允许已有的复位/历史状态，读 STAT 清除锁存；通信仍须通过 CRC。
     * 随后的 AVAL 必须所有状态正常。正常运行时绝不自动忽略复位。 */
    result = TLE5012B_CheckFrame(TLE_CMD_READ_STAT, data, safety, 0);
    if (result != TLE_OK) return result;
    return TLE5012B_ReadRaw(id, &raw);
}

TLE_Result TLE5012B_ReadRaw(uint8_t id, uint16_t *raw)
{
    uint16_t data, safety;
    TLE_Result result;
    if (id > 1u || raw == 0) return TLE_BAD_ARGUMENT;
    if (!MotorPort_SSCRead(id, TLE_CMD_READ_AVAL, &data, &safety)) return TLE_BUS_ERROR;
    result = TLE5012B_CheckFrame(TLE_CMD_READ_AVAL, data, safety, 1);
    if (result != TLE_OK) return result;
    *raw = data & 0x7fffu;  /* 15 位模一圈数值，0..32767；去掉 NEW 位 */
    return TLE_OK;
}

TLE_Result TLE5012B_ReadDegrees(uint8_t id, float *degrees)
{
    uint16_t raw;
    TLE_Result result;
    if (degrees == 0) return TLE_BAD_ARGUMENT;
    result = TLE5012B_ReadRaw(id, &raw);
    if (result == TLE_OK) *degrees = (float)raw * (360.0f / 32768.0f);
    return result;
}
