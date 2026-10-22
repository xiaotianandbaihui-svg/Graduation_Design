/* 状态仅由主循环修改；跳跃或故障锁定时，普通PID不能控制足端。 */
#ifndef GLOBAL_STATE_H
#define GLOBAL_STATE_H

#include <stdint.h>

#define Stand_Stop  0x01u
#define Jumping     0x02u
#define Running     0x04u
#define Robot_Fault 0x08u

extern uint8_t Robot_State;
extern uint8_t State_Lock;

#endif
