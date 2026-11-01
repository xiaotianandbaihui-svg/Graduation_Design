#include "remote_dashboard.h"
#include "remote_font.h"
#include <limits.h>
#include <math.h>

#define CYAN   0x2CD5FF
#define AMBER  0xF6BC68
#define MUTED  0x829AB3
#define TRACK  0x1B344E
#define WHITE  0xECF6FF

static lv_obj_t *speed_dial, *yaw_dial, *robot_scene;
static lv_obj_t *target_speed_label, *target_yaw_label, *speed_label, *yaw_label, *link_label;
static int target_speed_cms = INT_MAX, target_yaw_tenths = INT_MAX;
static int speed_cms = INT_MAX, yaw_tenths = INT_MAX, link_state = -1;
static uint32_t last_frame_ms, spin_ms;
static uint16_t road_phase;

static int Magnitude(int value) { return value < 0 ? -value : value; }

static int Scaled(float value, int scale)
{
    int rounded = (int)(fabsf(value) * scale + 0.5f);
    return value < 0.0f ? -rounded : rounded;
}

static int SpeedScale(void) { return (int)(REMOTE_DASHBOARD_SPEED_MAX_MPS * 100.0f); }

static lv_point_t Polar(int cx, int cy, int radius, int degrees)
{
    lv_point_t p;
    degrees %= 360;
    if (degrees < 0) degrees += 360;
    p.x = cx + (int32_t)lv_trigo_cos(degrees) * radius / LV_TRIGO_SIN_MAX;
    p.y = cy + (int32_t)lv_trigo_sin(degrees) * radius / LV_TRIGO_SIN_MAX;
    return p;
}

static void Line(lv_draw_ctx_t *ctx, lv_point_t a, lv_point_t b, int width, uint32_t color)
{
    lv_draw_line_dsc_t d;
    lv_draw_line_dsc_init(&d);
    d.color = lv_color_hex(color); d.width = width;
    d.round_start = d.round_end = 1;
    lv_draw_line(ctx, &d, &a, &b);
}

static void Arc(lv_draw_ctx_t *ctx, lv_point_t center, int radius,
                int first, int last, int width, uint32_t color)
{
    lv_draw_arc_dsc_t d;
    lv_draw_arc_dsc_init(&d);
    d.color = lv_color_hex(color); d.width = width;
    lv_draw_arc(ctx, &d, &center, radius, first, last);
}

static void Polygon(lv_draw_ctx_t *ctx, const lv_point_t *points, int count, uint32_t color)
{
    lv_draw_rect_dsc_t d;
    lv_draw_rect_dsc_init(&d);
    d.bg_color = lv_color_hex(color);
    lv_draw_polygon(ctx, &d, points, count);
}

static void DialText(lv_draw_ctx_t *ctx, int cx, int cy, const char *text, uint32_t color)
{
    lv_draw_label_dsc_t d;
    lv_point_t size;
    lv_area_t area;
    lv_draw_label_dsc_init(&d);
    d.font = &remote_font_20; d.color = lv_color_hex(color);
    lv_txt_get_size(&size, text, d.font, 0, 0, LV_COORD_MAX, LV_TEXT_FLAG_NONE);
    area.x1 = cx - size.x / 2; area.y1 = cy - size.y / 2;
    area.x2 = area.x1 + size.x - 1; area.y2 = area.y1 + size.y - 1;
    lv_draw_label(ctx, &d, &area, text, NULL);
}

static void DrawSpeed(lv_event_t *event)
{
    lv_draw_ctx_t *ctx = lv_event_get_draw_ctx(event);
    lv_area_t area;
    lv_point_t center, a, b, label;
    int i, value, scale = SpeedScale(), active = speed_cms == INT_MAX ? 0 : Magnitude(speed_cms);
    char text[16];
    lv_obj_get_coords(lv_event_get_target(event), &area);
    center.x = area.x1 + 188; center.y = area.y1 + 190;
    Arc(ctx, center, 180, 180, 360, 2, 0x396789);
    Arc(ctx, center, 174, 180, 360, 6, TRACK);
    if (active > scale) active = scale;
    if (active) Arc(ctx, center, 174, 180, 180 + active * 180 / scale, 6, CYAN);
    for (i = 0; i <= 50; ++i) {
        int angle = 180 + i * 180 / 50;
        uint32_t color = i >= 45 ? 0xF17E7E : i >= 38 ? AMBER : 0x379CD2;
        a = Polar(center.x, center.y, 162, angle);
        b = Polar(center.x, center.y, i % 10 ? 152 : 141, angle);
        Line(ctx, a, b, i % 10 ? 2 : 3, color);
        if (i % 10 == 0) {
            value = scale * i / 50;
            lv_snprintf(text, sizeof(text), "%d.%d", value / 100, (value % 100) / 10);
            label = Polar(center.x, center.y, 122, angle);
            DialText(ctx, label.x, label.y, text, WHITE);
        }
    }
}

static void DrawYaw(lv_event_t *event)
{
    lv_draw_ctx_t *ctx = lv_event_get_draw_ctx(event);
    lv_area_t area;
    lv_point_t center, a, b, arrow[3];
    int i, degrees;
    lv_obj_get_coords(lv_event_get_target(event), &area);
    center.x = area.x1 + 94; center.y = area.y1 + 94;
    Arc(ctx, center, 91, 0, 360, 2, 0x527A9C);
    Arc(ctx, center, 60, 0, 360, 1, TRACK);
    for (i = 0; i < 36; ++i) {
        a = Polar(center.x, center.y, 85, i * 10);
        b = Polar(center.x, center.y, i % 9 ? 81 : 76, i * 10);
        Line(ctx, a, b, i % 9 ? 1 : 2, MUTED);
    }
    DialText(ctx, center.x, center.y - 70, "0°", MUTED);
    DialText(ctx, center.x + 67, center.y, "90°", MUTED);
    DialText(ctx, center.x, center.y + 70, "180°", MUTED);
    DialText(ctx, center.x - 65, center.y, "-90°", MUTED);
    /* Yaw=0指向上方；只表示当前姿态参考系，不冒充地理方位。 */
    degrees = target_yaw_tenths / 10 - 90;
    arrow[0] = Polar(center.x, center.y, 91, degrees);
    arrow[1] = Polar(center.x, center.y, 79, degrees - 5);
    arrow[2] = Polar(center.x, center.y, 79, degrees + 5);
    Polygon(ctx, arrow, 3, AMBER);
    if (yaw_tenths != INT_MAX) {
        degrees = yaw_tenths / 10 - 90;
        a = Polar(center.x, center.y, 37, degrees);
        b = Polar(center.x, center.y, 70, degrees);
        Line(ctx, a, b, 3, CYAN);
        arrow[0] = Polar(center.x, center.y, 74, degrees);
        arrow[1] = Polar(center.x, center.y, 59, degrees - 6);
        arrow[2] = Polar(center.x, center.y, 59, degrees + 6);
        Polygon(ctx, arrow, 3, CYAN);
    }
}

/* 将机器人水平面旋转后压缩Y方向，再减去高度，形成简单的俯视立体效果。 */
static lv_point_t RobotPoint(int cx, int cy, int x, int y, int height, int angle)
{
    lv_point_t p;
    int sine = lv_trigo_sin(angle), cosine = lv_trigo_cos(angle);
    p.x = cx + (x * cosine - y * sine) / LV_TRIGO_SIN_MAX;
    p.y = cy + (x * sine + y * cosine) * 3 / (5 * LV_TRIGO_SIN_MAX) - height;
    return p;
}

static void RobotBox(lv_draw_ctx_t *ctx, int cx, int cy, int x1, int y1, int x2, int y2,
                     int height, int angle, uint32_t color, uint32_t edge)
{
    lv_point_t p[4];
    int i;
    p[0] = RobotPoint(cx, cy, x1, y1, height, angle);
    p[1] = RobotPoint(cx, cy, x2, y1, height, angle);
    p[2] = RobotPoint(cx, cy, x2, y2, height, angle);
    p[3] = RobotPoint(cx, cy, x1, y2, height, angle);
    Polygon(ctx, p, 4, color);
    for (i = 0; i < 4; ++i) Line(ctx, p[i], p[(i + 1) % 4], 1, edge);
}

static lv_point_t RoadPoint(int cx, int top, int side, int phase)
{
    lv_point_t p;
    int depth = phase * phase / 1024;
    p.x = cx + side * (22 + depth * 80 / 1024);
    p.y = top + depth * 128 / 1024;
    return p;
}

static void DrawRobot(lv_event_t *event)
{
    lv_draw_ctx_t *ctx = lv_event_get_draw_ctx(event);
    lv_area_t area, shadow;
    lv_draw_rect_dsc_t d;
    lv_point_t p[4], a, b, knee;
    int cx, cy, angle = speed_cms == 0 ? (spin_ms / 8u) % 360u : 0;
    int side, i;
    uint32_t accent = speed_cms == INT_MAX ? MUTED : CYAN;
    lv_obj_get_coords(lv_event_get_target(event), &area);
    cx = area.x1 + 140; cy = area.y1 + 94;
    /* 静止时地面不流动；行进时虚线从远处向近处移动，倒车时反向。 */
    for (side = -1; side <= 1; side += 2) {
        a = RoadPoint(cx, area.y1, side, 0); b = RoadPoint(cx, area.y1, side, 1024);
        Line(ctx, a, b, 1, TRACK);
        for (i = 0; i < 4; ++i) {
            int first = (i * 256 + road_phase) % 1024;
            int last = first + 90;
            if (last > 1024) last = 1024;
            p[0] = RoadPoint(cx, area.y1, side, first);
            p[1] = p[0]; p[1].x += side * (2 + first / 200);
            p[3] = RoadPoint(cx, area.y1, side, last);
            p[2] = p[3]; p[2].x += side * (2 + last / 200);
            Polygon(ctx, p, 4, speed_cms == INT_MAX ? TRACK : 0x2184AE);
        }
    }
    lv_draw_rect_dsc_init(&d); d.bg_color = lv_color_hex(0x10283C); d.radius = LV_RADIUS_CIRCLE;
    shadow.x1 = cx - 58; shadow.x2 = cx + 58; shadow.y1 = cy + 7; shadow.y2 = cy + 27;
    lv_draw_rect(ctx, &d, &shadow);
    /* 两个独立轮子及折叠腿，区别于四轮汽车图案。 */
    for (side = -1; side <= 1; side += 2) {
        int left = side < 0 ? -48 : 34;
        RobotBox(ctx, cx, cy, left, -23, left + 14, 23, 3, angle, 0x07111F, 0x4C6C8A);
        a = RobotPoint(cx, cy, left + 7, -17, 5, angle);
        b = RobotPoint(cx, cy, left + 7, 17, 5, angle);
        Line(ctx, a, b, 3, accent);
        a = RobotPoint(cx, cy, side * 17, 7, 18, angle);
        knee = RobotPoint(cx, cy, side * 28, 18, 10, angle);
        b = RobotPoint(cx, cy, side * 41, 0, 5, angle);
        Line(ctx, a, knee, 5, 0x7A9EC3); Line(ctx, knee, b, 5, 0x47749C);
    }
    RobotBox(ctx, cx, cy, -20, -23, 20, 18, 13, angle, 0x1B426C, 0x4F89BB);
    RobotBox(ctx, cx, cy, -18, -23, 18, 14, 20, angle, 0x285D94, accent);
    RobotBox(ctx, cx, cy, -12, -17, 12, 2, 22, angle, 0x081C30, 0x5D91C1);
    a = RobotPoint(cx, cy, -7, -10, 24, angle); b = RobotPoint(cx, cy, 7, -10, 24, angle);
    Line(ctx, a, b, 3, accent);
}

static lv_obj_t *Text(lv_obj_t *parent, const char *text, int x, int y,
                      int width, const lv_font_t *font, uint32_t color, lv_text_align_t align)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_obj_set_pos(label, x, y); lv_obj_set_width(label, width);
    lv_obj_set_style_text_font(label, font, 0); lv_obj_set_style_text_align(label, align, 0);
    lv_obj_set_style_text_color(label, lv_color_hex(color), 0);
    lv_label_set_text(label, text); lv_obj_clear_flag(label, LV_OBJ_FLAG_CLICKABLE);
    return label;
}

static lv_obj_t *Drawing(lv_obj_t *parent, int x, int y, int w, int h, lv_event_cb_t draw)
{
    lv_obj_t *obj = lv_obj_create(parent);
    lv_obj_remove_style_all(obj); lv_obj_set_pos(obj, x, y); lv_obj_set_size(obj, w, h);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(obj, draw, LV_EVENT_DRAW_MAIN, NULL);
    return obj;
}

static void SetSpeed(lv_obj_t *label, int value)
{
    int magnitude = Magnitude(value);
    lv_label_set_text_fmt(label, "%s%d.%02d", value < 0 ? "-" : "", magnitude / 100, magnitude % 100);
}

static void SetYaw(lv_obj_t *label, int value)
{
    int magnitude = Magnitude(value);
    lv_label_set_text_fmt(label, "%s%d.%d°", value < 0 ? "-" : "", magnitude / 10, magnitude % 10);
}

void RemoteDashboard_Create(lv_obj_t *parent)
{
    link_label = Text(parent, "等待回传", 290, 28, 220, &remote_font_20, MUTED, LV_TEXT_ALIGN_CENTER);
    Text(parent, "目标速度", 28, 110, 170, &remote_font_20, MUTED, LV_TEXT_ALIGN_LEFT);
    target_speed_label = Text(parent, "0.00", 26, 137, 178, &remote_digits_48, CYAN, LV_TEXT_ALIGN_LEFT);
    Text(parent, "m/s", 30, 192, 100, &remote_font_20, MUTED, LV_TEXT_ALIGN_LEFT);
    Text(parent, "目标偏航", 28, 229, 170, &remote_font_20, MUTED, LV_TEXT_ALIGN_LEFT);
    target_yaw_label = Text(parent, "0.0°", 28, 255, 170, &remote_font_24, AMBER, LV_TEXT_ALIGN_LEFT);

    speed_dial = Drawing(parent, 212, 82, 376, 232, DrawSpeed);
    Text(parent, "实际速度", 330, 181, 140, &remote_font_20, MUTED, LV_TEXT_ALIGN_CENTER);
    speed_label = Text(parent, "--", 280, 211, 240, &remote_digits_72, CYAN, LV_TEXT_ALIGN_CENTER);
    Text(parent, "m/s", 340, 291, 120, &remote_font_20, MUTED, LV_TEXT_ALIGN_CENTER);

    Text(parent, "实际偏航", 588, 98, 188, &remote_font_20, MUTED, LV_TEXT_ALIGN_CENTER);
    yaw_dial = Drawing(parent, 588, 128, 188, 188, DrawYaw);
    yaw_label = Text(parent, "--", 636, 208, 92, &remote_font_20, CYAN, LV_TEXT_ALIGN_CENTER);
    Text(parent, "实际", 612, 326, 70, &remote_font_20, CYAN, LV_TEXT_ALIGN_LEFT);
    Text(parent, "目标", 702, 326, 70, &remote_font_20, AMBER, LV_TEXT_ALIGN_LEFT);
    robot_scene = Drawing(parent, 260, 318, 280, 132, DrawRobot);
    last_frame_ms = lv_tick_get();
}

void RemoteDashboard_Update(const RemoteCommand *target, const RemoteFeedback *actual)
{
    uint32_t now = lv_tick_get(), elapsed = now - last_frame_ms;
    int next_speed = actual->valid ? Scaled(actual->speed_mps, 100) : INT_MAX;
    int next_yaw = actual->valid ? Scaled(actual->yaw_deg, 10) : INT_MAX;
    int next_target_speed = Scaled(target->speed_mps, 100), next_target_yaw = Scaled(target->yaw_deg, 10);
    int next_link = actual->valid ? 1 : actual->timed_out ? 2 : 0;
    last_frame_ms = now;
    if (elapsed > 100u) elapsed = 100u; /* 换页回来时不补播离开期间的动画 */
    if (next_target_speed != target_speed_cms) {
        target_speed_cms = next_target_speed; SetSpeed(target_speed_label, target_speed_cms);
    }
    if (next_target_yaw != target_yaw_tenths) {
        target_yaw_tenths = next_target_yaw; SetYaw(target_yaw_label, target_yaw_tenths);
        lv_obj_invalidate(yaw_dial);
    }
    if (next_speed != speed_cms) {
        speed_cms = next_speed;
        lv_obj_set_style_text_font(speed_label,
            actual->valid && Magnitude(speed_cms) >= 1000 ? &remote_digits_48 : &remote_digits_72, 0);
        lv_obj_set_style_text_color(speed_label,
            lv_color_hex(actual->valid && Magnitude(speed_cms) > SpeedScale() ? AMBER : CYAN), 0);
        if (actual->valid) SetSpeed(speed_label, speed_cms); else lv_label_set_text(speed_label, "--");
        lv_obj_invalidate(speed_dial); lv_obj_invalidate(robot_scene);
    }
    if (next_yaw != yaw_tenths) {
        yaw_tenths = next_yaw;
        if (actual->valid) SetYaw(yaw_label, yaw_tenths); else lv_label_set_text(yaw_label, "--");
        lv_obj_invalidate(yaw_dial);
    }
    if (next_link != link_state) {
        link_state = next_link;
        lv_label_set_text(link_label, link_state == 1 ? "已连接" : link_state == 2 ? "无数据传回" : "等待回传");
        lv_obj_set_style_text_color(link_label, lv_color_hex(link_state == 1 ? CYAN : link_state == 2 ? 0xFF9D7C : MUTED), 0);
    }
    if (actual->valid && elapsed) {
        /* 与显示精度一致：四舍五入为0.00m/s时原地转圈，非零时道路流动。 */
        if (speed_cms == 0) spin_ms = (spin_ms + elapsed) % 2880u;
        else {
            int speed = Magnitude(speed_cms), step;
            if (speed > 250) speed = 250;
            step = elapsed * (200 + speed * 4) / 1000u;
            road_phase = (road_phase + 1024 + (speed_cms < 0 ? -step : step)) % 1024;
        }
        lv_obj_invalidate(robot_scene);
    }
}
