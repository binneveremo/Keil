#include "Global.h"

float ang2rad(float ang){
	return ang*3.1415926f/180.0f;
}
float rad2ang(float rad){
	return rad*180.0f/3.1415926f;
}
char If_Rad_Oppsite(float rad1, float rad2, float ang){
	float differ = fabs(rad1 - rad2);
	if(differ > 3.14)
		differ -= 3.14;
	if(differ > ang2rad(ang))
		return 1;
	return 0;
}



float Limit(float x,float y, float z){
	if(x < y)
		return y;
	else if(x > z)
		return z;
	return x;
}
float Point_Distance(struct Point a,struct Point b){
	return hypot(a.x - b.x,a.y - b.y);
}
char Point_Difference(struct Point a,struct Point b){
	if((fabs(a.x - b.x) > 0.1) || (fabs(a.y - b.y) > 0.1) || (fabs(a.r - b.r) > 0.1))
		return 1;
	return 0;
}
float char2float(unsigned char * data){
	float num;
	memcpy((unsigned char *)&num,data,sizeof(float));
	return num;
}

