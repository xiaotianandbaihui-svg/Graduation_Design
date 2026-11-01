#ifndef TASK_CONFIG_H
#define TASK_CONFIG_H

#include "FreeRTOS.h"
#include "task.h"

/* 只在启动前修改配置；任务参数统一传本表项的地址，handle由创建函数填写。 */
typedef struct {
    const char *name;                   /*任务名称*/
    TaskFunction_t entry;               /*任务函数函数指针*/
    configSTACK_DEPTH_TYPE stack_words; /* 单位为StackType_t；本机1字=4字节，这个要去看底层函数pvMalloc函数 */
    UBaseType_t priority;               /* 数字越大优先级越高，须小于configMAX_PRIORITIES，这里默认是1到32 */
    uint32_t period_ms;                 /* GUI每轮结束等待/LoRa定期轮询的间隔，至少1 tick */
    TaskHandle_t* handle;                /*任务句柄，是任务控制块指针*/
} AppTaskConfig;

extern AppTaskConfig app_task_config[];
extern const UBaseType_t app_task_count;

#endif
