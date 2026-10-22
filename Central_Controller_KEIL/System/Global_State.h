/* 状态仅由主循环修改；中断只提交请求，不直接切换控制模式。 */
#ifndef GLOBAL_STATE_H
#define GLOBAL_STATE_H

typedef enum {
    ROBOT_NORMAL,   /* 普通PID控制，包括静止和运动；允许开始跳跃 */
    ROBOT_JUMPING,  /* 跳跃独占足端输出，完成恢复后才回到普通状态 */
    ROBOT_FAULT     /* 故障停机，须检查后重启；跳跃接口不能清除故障 */
} RobotState;

extern RobotState Robot_State;

#endif
