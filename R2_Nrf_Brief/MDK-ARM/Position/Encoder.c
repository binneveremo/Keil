#include "Can_Bsp.h"
#include "Encoder.h"
#include "Global.h"
#include "string.h"
#include "math.h"
#include "Gyro.h"
//定义enc1为记录x轴方向的码盘，调节sign1使得向前的时候增量为正数
//定义enc2为记录y轴方向的码盘，调节sign2使得向左的时候增量为正数
#define odo_can hfdcan3
#define circle_num 50
#define signx -1
#define signy 1
#define encx_id 1
#define ency_id 2
#define ratio 25.321f
unsigned char encoder_data[2][8];
unsigned char encoder_send[8];
struct Odometer odometer;
void Get_Encoder_Data(int id,unsigned char * data){
	if(id == encx_id)
		memcpy(&encoder_data[0],data,7);
	else if(id == ency_id)
		memcpy(&encoder_data[1],data,7);
}
void Diff_Odometer(void){
	odometer.o1_row = ((encoder_data[0][6] << 24) + (encoder_data[0][5] << 16) + (encoder_data[0][4] << 8) + encoder_data[0][3]);
	odometer.o2_row = ((encoder_data[1][6] << 24) + (encoder_data[1][5] << 16) + (encoder_data[1][4] << 8) + encoder_data[1][3]);
	odometer.do1 = signx*(odometer.o1_row - odometer.o1_pre);
	odometer.do2 = signy*(odometer.o2_row - odometer.o2_pre);
	if(odometer.do1 > circle_num * 2048)
    odometer.do1 -= circle_num * 4096;
	else if(odometer.do1 < -circle_num * 2048)
		odometer.do1 += circle_num * 4096;
	if(odometer.do2 > circle_num * 2048)
    odometer.do2 -= circle_num * 4096;
	else if(odometer.do2 < -circle_num * 2048)
		odometer.do2 += circle_num * 4096;
	odometer.o1_pre = odometer.o1_row;
	odometer.o2_pre = odometer.o2_row;
}
void Set_ZeroPoint(unsigned char ID){
	encoder_send[0]=0x04;
	encoder_send[1]=ID;
	encoder_send[2]=0x06;
	encoder_send[3]=0x00;
	FDCAN_Send(&odo_can,ID,"STD",encoder_send,"CLASSIC",4,"OFF"); 
}
void Encoder_XY_VX_VY_Cal(int dt){
  Diff_Odometer();
	//计算旋转补偿
	//计算车体速度
	float row_dx = odometer.do1 / ratio;
	float row_dy = odometer.do2 / ratio;
	//计算车体坐标系的速度
	site.car_pos.row_vx = row_dx / dt;
	site.car_pos.row_vy = row_dy / dt;
	//初步计算场地坐标系的dx dy
	float dx = row_dx* cos(ang2rad(site.now.r)) - row_dy*sin(ang2rad(site.now.r));
	float dy = row_dx* sin(ang2rad(site.now.r)) + row_dy*cos(ang2rad(site.now.r));
	//计算误差角度之后的dx dy
	odometer.dx = dx* cos(odometer.offset_angle) - dy*sin(odometer.offset_angle);
	odometer.dy = dy* cos(odometer.offset_angle) + dx*sin(odometer.offset_angle);
	//对 dx dy 进行积分
	odometer.x = odometer.dx + odometer.x;
	odometer.y = odometer.dy + odometer.y;
	//与定位系统交互
	site.enc_pos.row_x = odometer.x + 400;
	site.enc_pos.row_y = odometer.y - 375;
	//计算位置微分 也就是速度
	site.enc_pos.row_vx = odometer.x / dt;
	site.enc_pos.row_vy = odometer.y / dt;
	//计算行驶过的累计历程
	odometer.xdis += fabs(odometer.dx);
	odometer.ydis += fabs(odometer.dy);
}
void Encoder_Init(void){
	Set_ZeroPoint(0x01);
	HAL_Delay(100);
	Set_ZeroPoint(0x02);
	HAL_Delay(100);
	Encoder_XY_VX_VY_Cal(1);
	Odometer_Clear();
	odometer.offset_angle = 0;
}
void Odometer_Clear(void){
	odometer.x = 0;
	odometer.y = 0;
}
void Set_Odometer_X(float x){
	odometer.x = x * 25.32;
}
void Set_Odometer_Y(float y){
	odometer.y = y * 25.32;
}







