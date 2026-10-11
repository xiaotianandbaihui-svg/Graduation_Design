#ifndef JUMP_H
#define JUMP_H

#include <stdint.h>
#include <math.h>
#include "servo.h"
#include "mpu9250_simple.h"

/* 参数都在这里填写。两腿使用相同杆长；四项顺序为左AB、左ED、右AB、右ED。
 * JUMP_EXTERNAL_CONFIG供工程预先定义同名宏时使用，正常直接改下面即可。 */
#ifndef JUMP_EXTERNAL_CONFIG
#define JUMP_GEOMETRY          {NAN, NAN, NAN, NAN, NAN} /* L1~L5，mm */
#define JUMP_SERVO_IDS         {0, 1, 2, 3}
#define JUMP_OFFSET_DEG        {NAN, NAN, NAN, NAN}
#define JUMP_DIRECTION         {0, 0, 0, 0} /* 实际各填+1或-1 */
#define JUMP_MIN_DEG           {NAN, NAN, NAN, NAN}
#define JUMP_MAX_DEG           {NAN, NAN, NAN, NAN}
#define JUMP_X_MM              0.0f
#define JUMP_LOW_MM            NAN      /* 最低可用腿高，不是机构死点 */
#define JUMP_HIGH_MM           NAN      /* 最高可用腿高 */
#define JUMP_MID_MM            ((JUMP_LOW_MM + JUMP_HIGH_MM) * 0.5f) /* 落地准备高度 */
#define JUMP_BUFFER_MM         NAN      /* 屈腿终点：必须大于LOW、小于MID，留出行程余量 */
#define JUMP_BUFFER_MS         160u     /* 触地后屈腿的计划用时，调试初值 */
#define JUMP_BUFFER_MAX_MM_S   150.0f   /* 腿高指令最大变化速度；必要时自动延长用时 */
#define JUMP_SQUAT_MS          300u
#define JUMP_PUSH_MIN_MS       40u      /* 蹬伸后至少等待这些时间再判断离地 */
#define JUMP_PREP_DELAY_MS     80u      /* 腾空收腿后等待多久放到中间高度；不代表检测最高点 */
#define JUMP_TIMEOUT_MS        1500u    /* 从启动到触地估计的超时，不含后续屈腿 */
#define JUMP_SAMPLE_MS         5u       /* 原MPU初始化200Hz，每5ms至多读一次 */
#define JUMP_ACCEL_LSB_PER_G   16384.0f /* 原MPU初始化为±2g */
#define JUMP_AIR_G             0.35f    /* 以下均为待实测调整的判据初值 */
#define JUMP_AIR_SAMPLES       3u       /* 连续低于AIR_G才确认腾空 */
#define JUMP_LAND_G            0.80f
#define JUMP_LAND_SAMPLES      4u       /* 腾空后连续高于LAND_G，估计已触地 */
#define JUMP_LAND_BLANK_MS     60u      /* 空中收腿/放腿后短暂忽略落地判据 */
#endif

/* 底层接口也用宏留出：均要求成功返回非0。可换成你自己的驱动。 */
#ifndef JUMP_READ_ACCEL
#define JUMP_READ_ACCEL(raw)   MPU9250_ReadAccel(raw)
#endif
#ifndef JUMP_WRITE_SERVO
#define JUMP_WRITE_SERVO(id, deg) (Servo_SetAngle((id), (deg)) == SERVO_OK)
#endif

#define JUMP_ERROR    (-1)
#define JUMP_IDLE       0
#define JUMP_RUNNING    1
#define JUMP_LANDED     2       /* 已估计触地且屈腿指令轨迹完成；不保证实际站稳 */

/* 输入两腿共用的局部足端x/y(mm)，输出四个舵机指令角(度)。
 * angle顺序为左AB、左ED、右AB、右ED；1成功、0失败，失败不修改angle。 */
uint8_t Get_Angles(float x_mm, float y_mm, float angle[4]);

/* 舵机、MPU及毫秒时基先由原程序初始化。Start立即下发最低目标。
 * Start/Update返回上面的四种状态；Update在主循环每1~5ms调用即可。
 * 动作：最低->最高->腾空收短->中间高度->触地后平滑屈腿到BUFFER高度。
 * 屈腿期间仍返回RUNNING；指令轨迹完成才返回LANDED，不自动恢复站姿。
 * 若在空中收短阶段提前触地，则保持短腿并返回LANDED，不再放长或继续收短。
 * Update内读取MPU，必须使用同一执行上下文，不能与其他IIC操作并发。 */
int8_t Jump_Start(uint32_t now_ms);
int8_t Jump_Update(uint32_t now_ms);
void Jump_Stop(void); /* 取消后续动作，保留已发出的舵机目标 */

#endif
