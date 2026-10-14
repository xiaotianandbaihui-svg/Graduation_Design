#ifndef TLE5012B_SIMPLE_H
#define TLE5012B_SIMPLE_H

#include <stdint.h>

/* TLE5012B芯片本身使用双向DATA。
 * 1：接收时把PA7改为浮空输入，MISO/PA6仍负责读取（默认）。
 * 0：全程保持PA7复用输出；只适用于已确认支持此方式的模块适配电路。
 * 成品板有MOSI/MISO接口不代表可以省略释放；应以模块原理图为准。
 * 使用1时要求模块在PA7释放后仍能把DATA送到PA6。
 * 这不重配SPI1，只在传输中切换PA7的GPIO模式。 */
#ifndef TLE5012B_RELEASE_MOSI
#define TLE5012B_RELEASE_MOSI 1
#endif
#if TLE5012B_RELEASE_MOSI != 0 && TLE5012B_RELEASE_MOSI != 1
#error TLE5012B_RELEASE_MOSI_must_be_0_or_1
#endif

/* 以下是寄存器编号，不是已经左移4位的命令字段。 */
#define TLE_REG_STAT       0x00u
#define TLE_REG_ACSTAT     0x01u
#define TLE_REG_AVAL       0x02u
#define TLE_REG_ASPD       0x03u
#define TLE_REG_MOD1       0x06u
#define TLE_REG_MOD2       0x08u
#define TLE_REG_MOD3       0x09u
#define TLE_REG_MOD4       0x0Eu

typedef enum {
    TLE_OK = 0,
    TLE_ARGUMENT_ERROR = -1,
    TLE_SPI_TIMEOUT = -2,
    TLE_CRC_ERROR = -3,
    TLE_STATUS_ERROR = -4,
    TLE_RESPONSE_ERROR = -5,
    TLE_CONFIG_ERROR = -6
} TLE_Result;

/* 前提：你已经配置好SPI1和CS GPIO，且编码器上电稳定至少10ms。
 * SPI1：主机、16位、MSB先、CPOL=Low、CPHA=2Edge、双线全双工、软件NSS。
 * PA5=SCK，PA6=MISO，PA7=MOSI；默认推挽SSC时SCK不超过8MHz。
 * id=0：CS为PA4；id=1：CS为PB1。未选中的CS保持高。
 * 这些函数不配置SPI1、不占用定时器，也不执行任何电机控制。
 * 同一SPI总线不可被其他中断/任务同时使用。 */

TLE_Result TLE5012B_Init(uint8_t id); /* 清上电状态，检查360度量程/DSPU并读角度 */
TLE_Result TLE5012B_ReadReg(uint8_t id, uint8_t reg, uint16_t *value);
TLE_Result TLE5012B_WriteReg(uint8_t id, uint8_t reg, uint16_t value);
TLE_Result TLE5012B_ReadAngle(uint8_t id, float *degrees); /* [0, 360)，机械角 */
/* 直接读取芯片角速度，单位度/秒，有正负；使用芯片内部时钟的标称周期。
 * 不需要传dt，也不改写芯片配置。失败时不修改输出变量。 */
TLE_Result TLE5012B_ReadAngularSpeed(uint8_t id, float *deg_per_s);

#endif
