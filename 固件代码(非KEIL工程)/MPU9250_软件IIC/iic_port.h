#ifndef IIC_PORT_H
#define IIC_PORT_H

#include <stdint.h>

/* 这5个函数由你的单片机工程提供实现。
 * SDA/SCK必须使用开漏输出：0=拉低，1=释放，由外部上拉电阻拉高。
 * Read函数读取引脚实际电平（IDR），不要读取输出锁存器（ODR）。
 */
void SDA(uint8_t level);
void SCK(uint8_t level);       /* I2C中通常叫SCL，这里沿用你给出的SCK名称。 */
uint8_t SDA_Read(void);        /* 返回0或1。 */
uint8_t SCK_Read(void);        /* 返回0或1，用于检测时钟拉伸/线路被拉低。 */
void IIC_DelayUs(uint32_t us); /* 用硬件定时器等实现，不能使用可被优化掉的空循环。 */

#endif
