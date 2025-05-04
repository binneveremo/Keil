#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H
#include "Location.h"

struct R1_Data{
	float x;
	float y;
	float oppsite_angle;
	char receive_ball_flag;
};

extern struct R1_Data r1;
void Tell_Yao_Xuan(char *message);
void Test_Communication(int * test);
void Get_R1_Data(unsigned char * data);



#endif
