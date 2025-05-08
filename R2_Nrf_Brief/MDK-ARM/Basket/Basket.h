#ifndef __BASKET_H
#define __BASKET_H

#include "Television.h"
#include "Kalman.h"
#include "Global.h"
#include "math.h"

struct Basket_Lock{
	struct{
		float basketdis;
		float anglebetween_ladarandpole;
		float siteinterp_gain;
	}parameter;
	struct {
		float ladar2basketx;
		float ladar2baskety;
		float ladar2basketdis;
		float ladar2basketangle;
		struct Point now_interp_vfield;
		struct Point basket_target_vfield;
		struct Point backwardladar_field;
	}position;
	struct {
		char nearest_point;
	}flagof;
};
struct BasketPosition_Lock 
{
	float p;
	float i;
	float istart;
	float iend;
	float ilimit;
	float outlimit;
	float gain;
	float itotal_x;
	float itotal_y;
	float outx;
	float outy;
	///////新加的测试参数
	float total_dis;
	float brake_ilimit;
	float brake_outlimit;
	float brake_distance;
	float brake_gain;
	float brake_percent;
};
struct BasketAngle_Lock {
	float p;
	float i;
	float predict_step;
	float istart;
	
	float iend;
	float itotal;
	float ilimit;
	float outlimit;
};


extern struct Basket_Lock bl;
extern struct BasketAngle_Lock ba;
extern struct BasketPosition_Lock  bp;
void BasketPositionCal_AccordingVision(float dt);

void BasketAngleLock_ParInit(void);
float BasketAngleLock(void);

void BasketPositionLock_ParInit(void);
void BasketPositionLock(void);

#endif












