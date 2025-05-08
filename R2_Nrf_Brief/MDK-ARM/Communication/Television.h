#ifndef __TELEVISION_H
#define __TELEVISION_H

#include "Location.h"
#include "Chassis.h"
#include "Can_Bsp.h"
#include "Kalman.h"
#include "Guard.h"
#include "string.h"


#define site_id  0x201
#define ladar_id 0xA1

struct Vision{
	struct {
		unsigned char data[20];
		char get_flag;
		struct Point now;
		struct Point basket;
	}ladar;
	struct {
		struct Point target;
	}guard;
	uint8_uint32_float_union convert;
	unsigned char send[16];
	float reset_flag;
};
extern struct Vision vision;
void Vision_Basket_Decode(void);
void Get_Vision_Data(int header,unsigned char * data);
void Ladar_Decode(void);
void Send_Velocity_Vision(void);
#endif


