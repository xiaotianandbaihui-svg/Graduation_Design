#ifndef LEG_IK_H
#define LEG_IK_H
/*四个舵机偏置角度*/
#define Servo_Bias_1 0.0f  /*小车前进方向俯视图，左上舵机*/
#define Servo_Bias_2 0.0f  /*小车前进方向俯视图，左下舵机*/
#define Servo_Bias_3 0.0f  /*小车前进方向俯视图，右上舵机*/
#define Servo_Bias_4 0.0f  /*小车前进方向俯视图，右下舵机*/

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
    LEG_IK_POSTURE_ERROR = -4 /* 几何可达，但不符合要求的钻石姿态 */
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


/*定义车的机械结构参数*/
LegIK_Geometry My_LegIK_Geometry = {
    .l1_mm = 60.0f,
    .l2_mm = 90.0f,
    .l3_mm = 90.0f,
    .l4_mm = 60.0f,
    .l5_mm = 37.5f
};
/*左轮的两个舵机计算出应该的角度*/
LegIK_Angles Left_LegIK_Angles = {
    .q1_deg = 0.0f,
    .q4_deg = 0.0f
};
/*右轮的两个舵机计算出应该的角度*/
LegIK_Angles Right_LegIK_Angles = {
    .q1_deg = 0.0f,
    .q4_deg = 0.0f
};
#endif
