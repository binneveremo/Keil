#ifndef __TELEVISION_H
#define __TELEVISION_H

#include "Location.h"
#include "Chassis.h"
#include "Can_Bsp.h"
#include "Kalman.h"
#include "Guard.h"
#include "string.h"


#define guard_id 0x0110
#define ladar_id 0x1001 //0x104

#define clock_id 0x202
#define site_id  0x201
#define point_send_id  0x203
#define motion_send_id 0x204

extern char guard_stop_flag;


struct Ladar{
	float x;
	float y;
	float r;
	
	char get_flag;
	char angle_get_flag;
	
	float rowx;
	float rowy;
	float rowr;
	
	float dx;
	float dy;
	
	float fuse_x;
	float fuse_y;
	
	float ladar_offset;
};
struct Guard{
	struct Point target;
	float vx;
	float vy;
};


extern struct Ladar ladar;
extern struct Guard guard;

void Get_Vision_Data(unsigned char * data);
void Ladar_Decode(void);
void Vision_Data_Decode(void);
void Send_Velocity_Vision(void);
void Send_Clock_Vision(void);
void Television_Control_With_Guard(void);
#endif


