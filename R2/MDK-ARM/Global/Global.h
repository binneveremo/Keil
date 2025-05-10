#ifndef __GLOBAL_H
#define __GLOBAL_H

#include "string.h"
#include "math.h"




#define PI 3.1415926f
struct Point {
	float x;
	float y;
	float r;
};
typedef union __attribute__((packed))
{
	unsigned char uint8_data[32];
	unsigned int uint32_data[8];
	float float_data[8];
} uint8_uint32_float_union;
float Limit(float x,float y, float z);
float rad2ang(float rad);
float ang2rad(float ang);
float Point_Distance(struct Point a,struct Point b);
char Point_Difference(struct Point a,struct Point b);
char If_Rad_Oppsite(float rad1, float rad2, float ang);
float char2float(unsigned char * data);
float NormalizeAng_Single(float ang);
#endif
