#ifndef LEG_IK_H
#define LEG_IK_H
/*四个舵机偏置角度*/
#ifndef Servo_Bias_1
#define Servo_Bias_1 0.0f  /*小车前进方向俯视图，左上舵机；待实测标定*/
#endif
#ifndef Servo_Bias_2
#define Servo_Bias_2 0.0f  /*小车前进方向俯视图，左下舵机*/
#endif
#ifndef Servo_Bias_3
#define Servo_Bias_3 0.0f  /*小车前进方向俯视图，右上舵机*/
#endif
#ifndef Servo_Bias_4
#define Servo_Bias_4 0.0f  /*小车前进方向俯视图，右下舵机*/
#endif

#ifndef FOOT_SERVO_STEP_MAX_DEG
#define FOOT_SERVO_STEP_MAX_DEG 20.0f /* 每次四路各自的最大指令角变化；20ms带载实测后修改,单位是：° */
#endif

/* 同一条腿的机构：A--B--P--D--E--A。
 * A、E是机身上的固定舵机轴；P是两根从动杆的交点（假定也是轮轴）。
 * L1=AB，L2=BP，L3=PD，L4=ED，L5=AE。
 * 机身局部坐标：原点为AE中点，+x从A指向E，+y向腿的下方。
 * A=(-L5/2,0)，E=(+L5/2,0)。长度全部使用mm。
 * q1是A->B相对+x的角度，q4是E->D相对+x的角度。
 * 从+x朝+y转为正（按图观察为顺时针），输出角度范围[0,360)。
 */
typedef struct {
    float l1_mm, l2_mm, l3_mm, l4_mm, l5_mm;
} LegIK_Geometry;

typedef struct {
    float q1_deg; /* A->B与+x轴的夹角，单位：度 */
    float q4_deg; /* E->D与+x轴的夹角，单位：度 */
} LegIK_Angles;

typedef enum {
    LEG_IK_OK = 0,
    LEG_IK_ARGUMENT_ERROR = -1,
    LEG_IK_UNREACHABLE = -2,
    LEG_IK_SINGULAR = -3,
    LEG_IK_POSTURE_ERROR = -4, /* 几何可达，但不符合要求的钻石姿态 */
    LEG_IK_SERVO_ERROR = -5,
    LEG_IK_STEP_ERROR = -6
} LegIK_Result;

/*五连杆结构图
        A──────E
       /        \
    L1/          \L4
     B            D
      \          /
     L2\        /L3
           P
*/

           /* 目标轮轴P(x_mm,y_mm) -> 两根主动杆的几何角度。
 * 固定采用已选的张开姿态：A侧加acos项，E侧减acos项。
 * 成功时强制B在A左下、D在E右下，P在B/D之间且低于二者。
 * 左右两条腿分别在各自局部坐标下调用，不能把两条腿混为A/E两侧。
 * 只计算，不读写GPIO、不调用舵机驱动；失败时不修改out。
 * 只约束本次目标位形，不保证舵机实际执行过程；不检查实体碰撞和舵机限位。
 */

LegIK_Result LegIK_Solve(const LegIK_Geometry *geometry,
                         float x_mm, float y_mm, LegIK_Angles *out);

/* 统一设置左右足端：两腿分别逆解，沿用 servo=Servo_Bias-q 的安装映射。
 * 左右各逆解一次；舵机驱动检查四路角度并统一下发。
 * 超过单次角度步长返回LEG_IK_STEP_ERROR，上层应减小坐标步长后再调用。
 * x/y须为有效的两元素数组，本函数不修改它们；成功返回LEG_IK_OK。
 * 失败不修改舵机输出、逆解结果或指令历史。
 * 首次调用没有实际角度反馈，直接设置合法初始点，不保证20ms到位。
 * 必须先Servo_Init，仅主循环调用；不要与Jump等其他舵机控制器同时运行。
 * 这些坐标是指令位置，不能当作已经到位的实测位置。 */
LegIK_Result Leg_SetFootPositions(float x_mm[2], float y_mm[2]);
/* Servo_Init后或其他控制器接管过舵机后调用；下一次调用重新建立初始指令。 */
void Leg_ResetFootPositionHistory(void);


#endif
