#include "Television.h"
struct Vision vision;
//获取视觉数据 
void Vision_Basket_Decode(void){
	memcpy(vision.convert.uint8_data,vision.ladar.data,20);
	//视觉坐标系下车体当前坐标
	vision.ladar.now_vfield.x = vision.convert.float_data[0] * 1000;
	vision.ladar.now_vfield.y = vision.convert.float_data[1] * 1000;
	vision.ladar.now_vfield.r = vision.convert.float_data[2] * rad2ang(1);
	//获取视觉坐标系下的篮筐坐标
	vision.ladar.basket.x = vision.convert.float_data[3] * 1000;
	vision.ladar.basket.y = vision.convert.float_data[4] * 1000;
}
void Get_Vision_Data(int header,unsigned char * data){
	switch(header){
		case ladar_id:
			memcpy(vision.ladar.data,data,20);
		break;
		case online_id:
			vision.ladar.get_flag = 1;
		break;
		case offline_id:
		
		break;
		default:
			break;
	}
}

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






