#ifndef JUMP_H
#define JUMP_H

#include <stdint.h>
#include <math.h>
#include "balance_pid.h"
#include "Global_State.h"

/* 共用leg_ik中的杆长、舵机安装偏置和统一足端接口，不另设一套舵机映射。
 * +y向下：LOW为最短腿，HIGH为最长腿。NAN项必须实测填写后才允许启动。 */
#ifndef JUMP_EXTERNAL_CONFIG
#define JUMP_MID_MM            FOOT_INITIAL_Y_MM /* 跳跃中间/恢复位置的两腿平均高度，mm */
#define JUMP_LOW_MM            NAN               /* 最短单腿的下限，mm；两腿等量收缩至短腿到此值 */
#define JUMP_HIGH_MM           NAN               /* 最长单腿的上限，mm；两腿等量伸长至长腿到此值 */
#define JUMP_SQUAT_MS          300u              /* 下蹲后的舵机等待时间，这个是用来等待的 */
#define JUMP_PUSH_MIN_MS        40u              /* 蹬伸后最短等待，之后才判断离地 */
#define JUMP_PREP_DELAY_MS      80u              /* 收腿后等多久放回中间位置，待实测 */
#define JUMP_LOW_HOLD_MS       100u              /* 缓冲到最短指令后的等待，待实测 */
#define JUMP_RECOVER_MS        FOOT_INITIAL_SETTLE_MS /* 回到中间位置后等多久恢复普通控制，待实测 */
#define JUMP_BUFFER_STEP_MM     2.0f             /* 触地后每个采样周期缩腿量，待实测 */
#define JUMP_TIMEOUT_MS       1500u              /* 起跳到触地的最长等待，不含缓冲/恢复 */
#define JUMP_SAMPLE_MS        FOOT_PID_PERIOD_MS
#define JUMP_AIR_G              0.35f
#define JUMP_AIR_SAMPLES        3u
#define JUMP_LAND_G             0.80f
#define JUMP_LAND_SAMPLES       4u
#define JUMP_LAND_BLANK_MS     60u               /* 收放腿后忽略短暂冲击 */
#endif

#define JUMP_ERROR    (-1)
#define JUMP_IDLE       0
#define JUMP_RUNNING    1
#define JUMP_LANDED     2 /* 已下发恢复指令并等待指定时间，不是机械到位反馈 */

/* 遥控/调试入口：置1请求跳跃，由主循环消费；只在普通模式受理。 */
extern volatile uint8_t Jump_Request;

/* Start前须确认驱动板已暂停。冻结当前x和左右高度差；三组目标都合法才下发。
 * 重复Start不重启动作；Update每周期接收一份新的加速度模长(g)。
 * 动作失败进入ROBOT_FAULT，不能自动恢复；无单独的锁、phase或运行标志。
 * LANDED只在恢复完成的那次调用返回，表示指令等待结束，不是机械位置反馈。 */
int8_t Jump_Start(uint32_t now_ms);
int8_t Jump_Update(uint32_t now_ms, float accel_g);

#endif
