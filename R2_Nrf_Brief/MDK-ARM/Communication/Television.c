#include "Television.h"
struct Vision vision;
//获取视觉数据 
void Get_Vision_Data(int header,unsigned char * data){

}
void Ladar_Decode(void){
	memcpy(vision.convert.uint8_data,vision.ladar.data,12);
	vision.ladar.row.x = vision.convert.float_data[0] * 1000;
	vision.ladar.row.y = vision.convert.float_data[1] * 1000;
 	vision.ladar.row.r = vision.convert.float_data[2] * 2;
	//计算偏置的坐标轴
//	vision.ladar.cal.r = rad2ang(vision.ladar.row.r;
//	vision.ladar.cal.x = vision.ladar.row.x - 196.39*(sin(2 * PI * 0.16 * ladar.rowr + 1.46) - sin( 1.49)) + 400;
//	vision.ladar.cal.y = vision.ladar.row.x - 182.20*(sin(2 * PI * 0.16 * ladar.rowr  -0.12) - sin(-0.12)) + 375;
}
//视觉控制时候的流程

//向视觉发送速度的函数
void Send_Velocity_Vision(void){
	memset(vision.convert.uint8_data,0,sizeof(vision.convert));
	vision.convert.float_data[0] = site.now.x / 1000;
	vision.convert.float_data[1] = site.now.y / 1000;
	vision.convert.float_data[2] = ang2rad(site.now.r);

	memcpy(vision.send,vision.convert.uint8_data,sizeof(vision.send));
	FDCAN_Send(&hfdcan1,site_id,"STD",vision.send,"FD",12,"OFF");
}
//向视觉发送避障目标点的函数




