#ifndef TLE5012B_H
#define TLE5012B_H

#include <stdint.h>

typedef enum {
    TLE_OK = 0,
    TLE_BAD_ARGUMENT = -1,
    TLE_BUS_ERROR = -2,
    TLE_CRC_ERROR = -3,
    TLE_RESET_ERROR = -4,
    TLE_SYSTEM_ERROR = -5,
    TLE_INTERFACE_ERROR = -6,
    TLE_ANGLE_ERROR = -7,
    TLE_RESPONSE_ERROR = -8
} TLE_Result;

/* id=0：编码器1/PA4，id=1：编码器2/PB1。
 * MotorPort_Init() 之后使用；Motor_Init() 会自动完成这两层初始化。
 * 同一条 SPI 总线不可被主循环和其他中断同时调用。 */
TLE_Result TLE5012B_Init(uint8_t id);
TLE_Result TLE5012B_ReadRaw(uint8_t id, uint16_t *raw);
TLE_Result TLE5012B_ReadDegrees(uint8_t id, float *degrees);

/* 协议纯函数，供校验/测试使用。应用无需直接调用。 */
uint8_t TLE5012B_CRC8(const uint8_t *bytes, uint32_t length);
TLE_Result TLE5012B_CheckFrame(uint16_t command, uint16_t data,
                              uint16_t safety, int check_status);

#endif
