#ifndef APP_TASKS_H
#define APP_TASKS_H

#include "FreeRTOS.h"

/* 调度器启动前调用一次：按task_config.c配置表创建全部应用任务。 */
BaseType_t AppTasks_Create(void);
void App_GuiTask(void *argument);
void App_LoraTask(void *argument);

#endif
