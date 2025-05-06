#include "Television.h"
struct Vision vision;
struct Point basket_position = {
	.x = 14050,
	.y = -3990,
};
//获取视觉数据 
void Vision_Basket_Decode(void){
	memcpy(vision.convert.uint8_data,vision.basket.data,8);
	//将视觉坐标转化成毫米单位
	vision.basket.basket_pos.x = vision.convert.float_data[0] * 1000;
	vision.basket.basket_pos.y = vision.convert.float_data[1] * 1000;
	//反推出雷达坐标
	vision.basket.ladar2basketangle = atan2f(vision.basket.basket_pos.y,vision.basket.basket_pos.x);
	vision.basket.ladar2basketdis = hypot(vision.basket.basket_pos.x,vision.basket.basket_pos.y);
	vision.basket.backward_ladarpos.x = basket_position.x - cos(ang2rad(site.now.r) + vision.basket.ladar2basketangle) * vision.basket.ladar2basketdis;
  vision.basket.backward_ladarpos.y = basket_position.y - sin(ang2rad(site.now.r) + vision.basket.ladar2basketangle) * vision.basket.ladar2basketdis;
	//反推出车体坐标
	vision.basket.backward_carpos.x = vision.basket.backward_ladarpos.x - 190 * cos(ang2rad(site.now.r));
	vision.basket.backward_carpos.y = vision.basket.backward_ladarpos.y - 190 * sin(ang2rad(site.now.r));
}
void Get_Vision_Data(int header,unsigned char * data){
	switch(header){
		case basket_id:
			memcpy(vision.basket.data,data,8);
			vision.basket.get_flag = 1;
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

	memcpy(vision.send,vision.convert.uint8_data,sizeof(vision.send));
	FDCAN_Send(&hfdcan1,site_id,"STD",vision.send,"FD",12,"OFF");
}
//向视觉发送避障目标点的函数




