#ifndef __TELEVISION_H
#define __TELEVISION_H

#include "Location.h"
#include "Chassis.h"
#include "Can_Bsp.h"
#include "Kalman.h"
#include "Guard.h"
#include "string.h"


#define site_id  0x201
#define basket_id 0xA1

struct Vision{
	struct {
		struct Point row;
		struct Point cal;
	}ladar;
	struct {
		struct Point target;
	}guard;
	struct {
		unsigned char data[8];
		char get_flag;
		float ladar2basketangle;
		float ladar2basketdis;
		struct Point basket_pos;
		struct Point backward_ladarpos;
		struct Point backward_carpos;
	}basket;
	uint8_uint32_float_union convert;
	unsigned char send[12];
};
extern struct Vision vision;
void Vision_Basket_Decode(void);
void Get_Vision_Data(int header,unsigned char * data);
void Ladar_Decode(void);
void Send_Velocity_Vision(void);
#endif


