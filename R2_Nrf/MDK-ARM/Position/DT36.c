#include "DT36.h"
#define Right_Start 1
struct DT35_Union DT35;
/////////////////////////////////////////计算每个DT35的场地坐标////////////////////////////////
//根据三角函数和距离车体中心的角度 

/*一些误解
1.陀螺仪的物理位置与概念位置没有任何关系 因为陀螺仪测量的是相对于上电时候的角度 可以定义向前为0度 也可以定义为向右为零度
2.陀螺仪的-180 - 180 与 0 - 360也没有任何的区别 因为-180 - 0 度的区间加上360度就可以重合 
3.DT35的真实角度是否需要归一化:不需要 因为不是为了好看 
*/
void DT35Single_Position_Cal(struct DT35_Single * DT35){
    //先计算相对于场地坐标系的角度 也就是车体角度 - center_angle 
    DT35->pos.r = (site.now.r - rad2ang(DT35->center_angle));
    //根据当前的角度和cenetr_dis计算出真实的DT35坐标
    DT35->pos.x = site.now.x + DT35->center_dis * cos(DT35->pos.r);
}



















