#ifndef GLOBAL_STATE_H
#define GLOBAL_STATE_H

/* 唯一的控制状态；仅主循环修改。跳跃各阶段也用它，不再另设锁或phase。 */
typedef enum {
    ROBOT_NORMAL,
    ROBOT_JUMP_SQUAT,       /* 下蹲等待 */
    ROBOT_JUMP_PUSH,        /* 蹬伸，检测离地 */
    ROBOT_JUMP_RETRACT,     /* 空中收腿 */
    ROBOT_JUMP_LAND,        /* 中间高度，等待触地 */
    ROBOT_JUMP_BUFFER,      /* 触地后逐步收腿 */
    ROBOT_JUMP_LOW_HOLD,    /* 最短位置等待 */
    ROBOT_JUMP_RECOVER,     /* 恢复中间高度并等待舵机 */
    ROBOT_FAULT
} RobotState;

extern RobotState Robot_State;
#endif
