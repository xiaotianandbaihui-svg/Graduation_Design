#ifndef REMOTE_DASHBOARD_H
#define REMOTE_DASHBOARD_H

#include "lvgl.h"
#include "remote_control.h"

/* 只影响表盘量程，不限制实际速度或控制目标；超量程时数字仍显示真实值。 */
#define REMOTE_DASHBOARD_SPEED_MAX_MPS 1.0f

/* 仅GUI任务调用。创建一次，随后由界面50ms刷新同步数据和动画。 */
void RemoteDashboard_Create(lv_obj_t *parent);
void RemoteDashboard_Update(const RemoteCommand *target, const RemoteFeedback *actual);

#endif
