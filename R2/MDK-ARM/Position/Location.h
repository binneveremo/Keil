#ifndef __LOCATION_H
#define __LOCATION_H
#include "Global.h"
#include "DT35.h"
#define ladar_angle 0
#define gyro_angle 1

struct Site{
	struct Point target;
	struct Point now;
	//编码器计算出来的单独的位置以及速度
	struct {
		float row_x;
		float row_y;
		float row_vx;
		float row_vy;
	}enc_pos;
	//dt35矫正系统自己独享坐标系
	struct {
		float x;
		float y;
	  unsigned char x_credible_flag;
		unsigned char y_credible_flag;
	}dt35_pos;
	//x,y将采用码盘与雷达重定位的坐标
	struct {
		float row_x;
		float row_y;
		float with_odo_x;
		float with_odo_y;
		unsigned char get_flag;
	}ladar_pos;
	//x,y将采用码盘与雷达卡尔曼融合的坐标，vx,vy将采用码盘与加速度计融合的速度
	struct {
		float x;
		float y;
		float vx;
		float vy;
	}fuse_pos;
	//加速度计积分出来的速度
	struct {
		float with_odo_vx;
		float with_odo_vy;
		float r;
		float omiga;
		float accx;
		float accy;
	}gyro_pos;
	struct {
		float ax;
		float ay;
		float vx;
		float vy;
		float row_vx;
		float row_vy;
	}car_pos;
};
enum Location_Type{
	odometer_location,
	vision_location,
	dt35_location,
	mix_location
};

extern struct Site site;


void Location_Type_Choose(void);
void DT35_Fuse_With_Odometer(float dt);

void Enc_VXVY_Fuse_With_Gyro_AXAY(float dt);
//雷达与马盘数据融合
void Ladar_With_Odometer_Kalman(float dt);
void Ladar_With_Odometer_Relocation(float dt);
#endif