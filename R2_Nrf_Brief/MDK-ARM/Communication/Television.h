#ifndef __TELEVISION_H
#define __TELEVISION_H

#include "Location.h"
#include "Chassis.h"
#include "Can_Bsp.h"
#include "Kalman.h"
#include "Guard.h"
#include "string.h"


#define clock_id 0x202
#define site_id  0x201
#define point_send_id  0x203
#define motion_send_id 0x204

struct Vision{
	struct {
		struct Point row;
		struct Point cal;
		unsigned char data[12];
	}ladar;
	struct {
		struct Point target;
		unsigned char data[12];
	}guard;
	struct {
		
	}basket;
	uint8_uint32_float_union convert;
	unsigned char send[12];
};
extern struct Vision vision;

void Get_Vision_Data(int header,unsigned char * data);
void Ladar_Decode(void);
void Send_Velocity_Vision(void);
#endif


