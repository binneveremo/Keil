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
	float dx = odometer.do1;
	float dy = odometer.do2;
	
	site.car_pos.row_vx = dx / dt / 25.32;
	site.car_pos.row_vy = dy / dt / 25.32;
	
	odometer.dx = dx* cos(ang2rad(site.now.r)) - dy*sin(ang2rad(site.now.r));
	odometer.dy = dx* sin(ang2rad(site.now.r)) + dy*cos(ang2rad(site.now.r));
	odometer.y = odometer.dy + odometer.y;
	odometer.x = odometer.dx + odometer.x;
	
	site.enc_pos.row_y = odometer.y/ 25.32f;
	site.enc_pos.row_x = odometer.x/ 25.32f;

	//计算位置微分 也就是速度
	site.enc_pos.row_vy = odometer.dy / dt / 25.32f;
	site.enc_pos.row_vx = odometer.dx / dt / 25.32f;
	//计算行驶过的累计历程
	odometer.xdis += fabs(odometer.dx) / 25.32;
	odometer.ydis += fabs(odometer.dy) / 25.32;
}
void Encoder_Init(void){
	Set_ZeroPoint(0x01);
	HAL_Delay(100);
	Set_ZeroPoint(0x02);
	HAL_Delay(100);
	Encoder_XY_VX_VY_Cal(1);
	Odometer_Clear();
}
void Odometer_Clear(void){
	odometer.x = 0;
	odometer.y = 0;
}









