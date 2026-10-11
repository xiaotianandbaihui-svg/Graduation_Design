#ifndef LEG_IK_H
#define LEG_IK_H

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
    LEG_IK_SINGULAR = -3
} LegIK_Result;

/* 目标轮轴P(x_mm,y_mm) -> 两根主动杆的几何角度。
 * 固定采用已选的张开姿态：A侧加acos项，E侧减acos项。
 * 只计算，不读写GPIO、不调用舵机驱动；失败时不修改out。
 * 检查两侧可达性和接近共线的奇异位形，不检查实体杆件/机身碰撞。
 */
LegIK_Result LegIK_Solve(const LegIK_Geometry *geometry,
                         float x_mm, float y_mm, LegIK_Angles *out);

#endif
