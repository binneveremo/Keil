#include "Television.h"
struct Vision vision;
struct Point basket_position = {
	.x = 14050,
	.y = -3990,
};
//获取视觉数据 
void Vision_Basket_Decode(void){
	memcpy(vision.convert.uint8_data,vision.ladar.data,20);
	//视觉坐标系下车体当前坐标
	vision.ladar.now.x = vision.convert.float_data[0] * 1000;
	vision.ladar.now.y = vision.convert.float_data[1] * 1000;
	vision.ladar.now.r = vision.convert.float_data[2] * rad2ang(1);
	//获取视觉坐标系下的篮筐坐标
	vision.ladar.basket.x = vision.convert.float_data[3] * 1000;
	vision.ladar.basket.y = vision.convert.float_data[4] * 1000;
	
	
//	//将视觉坐标转化成毫米单位
//	vision.basket.basket_pos.x = vision.convert.float_data[0] * 1000;
//	vision.basket.basket_pos.y = vision.convert.float_data[1] * 1000;
//	//反推出雷达坐标
//	vision.basket.ladar2basketangle = atan2f(vision.basket.basket_pos.y,vision.basket.basket_pos.x);
//	vision.basket.ladar2basketdis = hypot(vision.basket.basket_pos.x,vision.basket.basket_pos.y);
//	vision.basket.backward_ladarpos.x = basket_position.x - cos(ang2rad(site.now.r) + vision.basket.ladar2basketangle) * vision.basket.ladar2basketdis;
//  vision.basket.backward_ladarpos.y = basket_position.y - sin(ang2rad(site.now.r) + vision.basket.ladar2basketangle) * vision.basket.ladar2basketdis;
//	//反推出车体坐标
//	vision.basket.backward_carpos.x = vision.basket.backward_ladarpos.x - 190 * cos(ang2rad(site.now.r));
//	vision.basket.backward_carpos.y = vision.basket.backward_ladarpos.y - 190 * sin(ang2rad(site.now.r));
}
void Get_Vision_Data(int header,unsigned char * data){
	switch(header){
		case ladar_id:
			memcpy(vision.ladar.data,data,20);
			vision.ladar.get_flag = 1;
		break;
		default:
			break;
	}
}
void Ladar_Decode(void){
//	memcpy(vision.convert.uint8_data,vision.ladar.data,12);
//	vision.ladar.row.x = vision.convert.float_data[0] * 1000;
//	vision.ladar.row.y = vision.convert.float_data[1] * 1000;
// 	vision.ladar.row.r = vision.convert.float_data[2] * 2;
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
	vision.convert.float_data[3] = vision.reset_flag;
	memcpy(vision.send,vision.convert.uint8_data,sizeof(vision.send));
	FDCAN_Send(&hfdcan1,site_id,"STD",vision.send,"FD",16,"OFF");
	vision.reset_flag = (vision.reset_flag > 0.5)?0:vision.reset_flag;
}
//向视觉发送避障目标点的函数




