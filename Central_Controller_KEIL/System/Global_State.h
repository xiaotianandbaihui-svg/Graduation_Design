/*小车全局状态*/
#ifndef Gloabal_State_H
#define Gloabal_State_H

#include <stm32f10x.h>

#define	Stand_Stop	0x0001	/*静止直立模式，就是双轮速度都降到0的某个范围内*/
#define	Jumping		0x0002	/*跳跃模式，进入跳跃模式从调用跳跃函数开始，一旦进入跳跃模式，自动退出其他全部模式*/
#define	Running		0x0004	/*普通模式，判定条件是两腿的y坐标的绝对值相差在某个阈值内，机身的Roll角绝对值在某个0的某个确定的范围内，则进入普通奔跑模式*/
uint8_t State_Lock=0;/*状态锁，防止在跳跃模式下，其他模式的代码对舵机进行控制*/

#endif