#ifndef __DT36_H
#define __DT36_H
#include "Location.h"
#include "Global.h"

#define DT35_NUM 4

struct __attribute__ ((packed)) DT35_Single{
    //真实的场地坐标
    struct Point pos;
    //四个阈值
    float frth;
    float flth;
    float blth;
    float brth;
    //相关参数
    float angle_offset;
    float center_dis;
    //相对于车体中轴线前方的硬件角度
    float center_angle;
};
struct __attribute__ ((packed)) DT35_Union {
    struct Point MiddlePos;
    struct DT35_Single dt35_single[DT35_NUM];
};

#endif

