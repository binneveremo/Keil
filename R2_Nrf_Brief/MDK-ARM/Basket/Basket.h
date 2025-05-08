#ifndef __BASKET_H
#define __BASKET_H

#include "Television.h"
#include "Kalman.h"
#include "math.h"

struct Basket_Lock{
	enum{
		far,
		near,
		middle,
	}status;
	struct {
		float velocity_gain;
		float accel_gain;
		float p;
		float i;
		float d;
		float predict_step;
		float error_last;
		float itotal;
		float ilimit;
		float istart;
		float iend;
		float outlimit;
	}pid;
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
		struct Point now_interp;
		struct Point basket_target;
	}position;
	struct {
		char nearest_point;
	}flagof;
};
extern struct Basket_Lock bl;
void BasketPositionCal_AccordingVision(float dt);
void BasketLock_ParameterInit(void);
float BasketAngle_PIDOut(void);
void BasketRunPoint_PIDParInit(void);
void BasketRunPoint(void);
#endif












