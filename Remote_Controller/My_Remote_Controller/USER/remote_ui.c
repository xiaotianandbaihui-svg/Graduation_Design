#include "remote_ui.h"
#include "remote_control.h"
#include "remote_font.h"
#include "remote_dashboard.h"
#include <math.h>

#define COLOR_BG     0x07111F
#define COLOR_PANEL  0x10283F
#define COLOR_MUTED  0x91A3BC
#define COLOR_TEXT   0xF0F5FC
#define COLOR_ACCENT 0x2CD5FF
#define STICK_SIZE   160
#define STICK_TRAVEL 50

static lv_obj_t *pages[REMOTE_PAGE_COUNT];
static RemotePage current_page;
static lv_obj_t *stick, *knob, *jump_button, *tx_error_label;
static uint8_t tx_failed;

static lv_obj_t *CreateHomePage(void);
static lv_obj_t *CreateManualPage(void);
static lv_obj_t *CreateAutoPage(void);
/* 新页面只需增加一个创建函数和此表项，不必改写主循环。 */
static lv_obj_t *(*const page_create[REMOTE_PAGE_COUNT])(void) = {
    CreateHomePage, CreateManualPage, CreateAutoPage
};

static lv_obj_t *Box(lv_obj_t *parent, int x, int y, int w, int h, uint32_t color, int radius)
{
    lv_obj_t *obj = lv_obj_create(parent);
    lv_obj_remove_style_all(obj);
    lv_obj_set_pos(obj, x, y);
    lv_obj_set_size(obj, w, h);
    lv_obj_set_style_bg_color(obj, lv_color_hex(color), 0);
    lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(obj, radius, 0);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
    return obj;
}

static lv_obj_t *Text(lv_obj_t *parent, const char *text, int x, int y,
                      const lv_font_t *font, uint32_t color)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_style_text_font(label, font, 0);
    lv_obj_set_style_text_color(label, lv_color_hex(color), 0);
    lv_label_set_text(label, text);
    lv_obj_set_pos(label, x, y);
    lv_obj_clear_flag(label, LV_OBJ_FLAG_CLICKABLE);
    return label;
}

static lv_obj_t *Button(lv_obj_t *parent, const char *text, int x, int y,
                        int w, int h, uint32_t color, lv_event_cb_t callback)
{
    lv_obj_t *button = lv_btn_create(parent);
    lv_obj_t *label;
    lv_obj_remove_style_all(button);
    lv_obj_set_pos(button, x, y);
    lv_obj_set_size(button, w, h);
    lv_obj_clear_flag(button, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_radius(button, 14, 0);
    lv_obj_set_style_bg_opa(button, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(button, lv_color_hex(color), 0);
    lv_obj_set_style_bg_color(button, lv_color_darken(lv_color_hex(color), 45), LV_STATE_PRESSED);
    lv_obj_set_style_bg_color(button, lv_color_hex(0x30445B), LV_STATE_DISABLED);
    label = Text(button, text, 0, 0, &remote_font_24,
                 color == COLOR_ACCENT ? COLOR_BG : COLOR_TEXT);
    lv_obj_center(label);
    lv_obj_add_event_cb(button, callback, LV_EVENT_CLICKED, NULL);
    return button;
}

static void Refresh(lv_timer_t *timer)
{
    RemoteCommand target;
    RemoteFeedback actual;
    (void)timer;
    if (current_page != REMOTE_PAGE_MANUAL) return;
    target = RemoteControl_GetCommand();
    actual = RemoteControl_GetFeedback();
    RemoteDashboard_Update(&target, &actual);
    if (RemoteControl_JumpBusy()) lv_obj_add_state(jump_button, LV_STATE_DISABLED);
    else lv_obj_clear_state(jump_button, LV_STATE_DISABLED);
}

static void ResetStick(void)
{
    if (knob) {
        lv_obj_align(knob, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_style_bg_color(knob, lv_color_hex(0x29465F), 0);
    }
}

static void StickEvent(lv_event_t *event)
{
    lv_event_code_t code = lv_event_get_code(event);
    if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
        RemoteControl_Release();
        ResetStick();
        Refresh(NULL);
    } else if (code == LV_EVENT_PRESSED || code == LV_EVENT_PRESSING) {
        lv_indev_t *indev = lv_indev_get_act();
        lv_point_t point;
        lv_area_t area;
        float dx, dy, distance, scale;
        if (!indev) return;
        lv_indev_get_point(indev, &point);
        lv_obj_get_coords(stick, &area);
        dx = point.x - (area.x1 + area.x2) * 0.5f;
        dy = point.y - (area.y1 + area.y2) * 0.5f;
        RemoteControl_SetStick(dx, dy, STICK_TRAVEL);
        distance = sqrtf(dx * dx + dy * dy);
        scale = distance > STICK_TRAVEL ? STICK_TRAVEL / distance : 1.0f;
        lv_obj_align(knob, LV_ALIGN_CENTER, (lv_coord_t)(dx * scale), (lv_coord_t)(dy * scale));
        lv_obj_set_style_bg_color(knob, lv_color_hex(COLOR_ACCENT), 0);
        Refresh(NULL);
    }
}

static void AutoEvent(lv_event_t *event)
{
    (void)event;
    RemoteUi_ShowPage(REMOTE_PAGE_AUTO);
}

static void BackEvent(lv_event_t *event)
{
    (void)event;
    RemoteUi_ShowPage(REMOTE_PAGE_HOME);
}

static void ManualEvent(lv_event_t *event)
{
    (void)event;
    RemoteUi_ShowPage(REMOTE_PAGE_MANUAL);
}

static void JumpEvent(lv_event_t *event)
{
    (void)event;
    RemoteControl_RequestJump();
    ResetStick();
    Refresh(NULL);
}

static lv_obj_t *Screen(void)
{
    lv_obj_t *screen = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(screen, lv_color_hex(COLOR_BG), 0);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);
    lv_obj_clear_flag(screen, LV_OBJ_FLAG_SCROLLABLE);
    return screen;
}

static lv_obj_t *CreateManualPage(void)
{
    lv_obj_t *screen = Screen();
    lv_obj_t *panel;
    Box(screen, 24, 24, 4, 30, COLOR_ACCENT, 2);
    Text(screen, "遥控模式", 42, 24, &remote_font_24, COLOR_TEXT);
    Button(screen, "返回主界面", 602, 18, 174, 52, COLOR_PANEL, BackEvent);
    Box(screen, 24, 86, 752, 1, 0x24334A, 0);

    stick = Box(screen, 28, 292, STICK_SIZE, STICK_SIZE, COLOR_PANEL, LV_RADIUS_CIRCLE);
    lv_obj_set_style_border_width(stick, 2, 0);
    lv_obj_set_style_border_color(stick, lv_color_hex(0x34506A), 0);
    lv_obj_add_flag(stick, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_ADV_HITTEST);
    lv_obj_add_event_cb(stick, StickEvent, LV_EVENT_ALL, NULL);
    panel = Box(stick, 0, 0, 112, 112, COLOR_PANEL, LV_RADIUS_CIRCLE);
    lv_obj_center(panel);
    lv_obj_set_style_border_width(panel, 1, 0);
    lv_obj_set_style_border_color(panel, lv_color_hex(0x263B53), 0);
    panel = Box(stick, 0, 0, 120, 1, 0x263B53, 0);
    lv_obj_center(panel);
    panel = Box(stick, 0, 0, 1, 120, 0x263B53, 0);
    lv_obj_center(panel);
    knob = Box(stick, 0, 0, 48, 48, 0x29465F, LV_RADIUS_CIRCLE);
    lv_obj_set_style_border_width(knob, 2, 0);
    lv_obj_set_style_border_color(knob, lv_color_hex(COLOR_ACCENT), 0);
    lv_obj_center(knob);

    RemoteDashboard_Create(screen);
    jump_button = Button(screen, "跳跃", 656, 392, 120, 60, COLOR_ACCENT, JumpEvent);
    tx_error_label = Text(screen, "串口发送失败", 292, 450, &remote_font_20, 0xFF9D7C);
    if (!tx_failed) lv_obj_add_flag(tx_error_label, LV_OBJ_FLAG_HIDDEN);
    return screen;
}

static lv_obj_t *CreateHomePage(void)
{
    lv_obj_t *screen = Screen();
    Box(screen, 24, 24, 4, 30, COLOR_ACCENT, 2);
    Text(screen, "主界面", 42, 24, &remote_font_24, COLOR_TEXT);
    Box(screen, 24, 86, 752, 1, 0x24334A, 0);
    Button(screen, "自动运行", 144, 194, 240, 92, COLOR_PANEL, AutoEvent);
    Button(screen, "遥控模式", 416, 194, 240, 92, COLOR_ACCENT, ManualEvent);
    return screen;
}

static lv_obj_t *CreateAutoPage(void)
{
    lv_obj_t *screen = Screen();
    Button(screen, "返回主界面", 24, 20, 180, 56, COLOR_PANEL, BackEvent);
    return screen; /* 按需求留白，只保留返回按钮。 */
}

void RemoteUi_ShowPage(RemotePage page)
{
    lv_indev_t *input = NULL;
    if ((unsigned)page >= REMOTE_PAGE_COUNT) return;
    RemoteControl_Stop();
    ResetStick();
    if (!pages[page]) pages[page] = page_create[page]();
    current_page = page;
    RemoteControl_SetManualMode(page == REMOTE_PAGE_MANUAL);
    lv_scr_load(pages[page]);
    Refresh(NULL);
    /* 换页结束旧页面的拖动，手指松开后才接受新页面的操作。 */
    lv_indev_reset(NULL, NULL);
    while ((input = lv_indev_get_next(input)) != NULL) lv_indev_wait_release(input);
}

RemotePage RemoteUi_GetPage(void)
{
    return current_page;
}

void RemoteUi_SetTxResult(int result)
{
    uint8_t failed;
    if (!result) return;
    failed = result < 0;
    if (failed == tx_failed) return;
    tx_failed = failed;
    if (!tx_error_label) return; /* 进入遥控页时再创建并显示状态。 */
    if (failed) lv_obj_clear_flag(tx_error_label, LV_OBJ_FLAG_HIDDEN);
    else lv_obj_add_flag(tx_error_label, LV_OBJ_FLAG_HIDDEN);
}

void RemoteUi_Init(void)
{
    RemoteUi_ShowPage(REMOTE_PAGE_HOME);
    lv_timer_create(Refresh, 50u, NULL);
}
