#ifndef __ENCODER_H
#define __ENCODER_H
struct Odometer {
	int o1_pre;
	int o2_pre;
	int o1_row;
	int o2_row;
	
	float do1;
	float do2;
	float dx;
	float dy;
	float x;
	float y;
	//true differential 1 真正的微分 也就是旋转补偿之后的结果
	double tx;
	float ty;

	//x轴行驶过的里程
	float xdis;
	float ydis;
	
	
};
extern float offset_angle;
extern struct Odometer odometer;
void Encoder_XY_VX_VY_Cal(int dt);
void Encoder_Init(void);
void Odometer_Clear(void);
void Odometer_Clear_X(void);
void Odometer_Clear_Y(void);
void Get_Encoder_Data(int id,unsigned char * data);
void Set_Odometer_X(float x);
void Set_Odometer_Y(float y);
#endif
