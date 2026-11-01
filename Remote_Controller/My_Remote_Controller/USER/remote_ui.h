#ifndef REMOTE_UI_H
#define REMOTE_UI_H

typedef enum {
    REMOTE_PAGE_HOME = 0,
    REMOTE_PAGE_MANUAL,
    REMOTE_PAGE_AUTO,
    REMOTE_PAGE_COUNT
} RemotePage;

/* 先lv_init、显示/触摸注册、RemoteControl_Init，再调用本函数。 */
void RemoteUi_Init(void);
/* 增加页面：扩展RemotePage，并在remote_ui.c的page_create表添加创建函数。 */
void RemoteUi_ShowPage(RemotePage page);
RemotePage RemoteUi_GetPage(void);
void RemoteUi_SetTxResult(int result); /* 仅反映本机串口交付，不能证明小车收到 */

#endif
