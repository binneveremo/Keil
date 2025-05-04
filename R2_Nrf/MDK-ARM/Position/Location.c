#include "Location.h"
#include "Encoder.h"
#include "Television.h"
#include <string.h>
#include "Global.h"
#include "Kalman.h"
#include "math.h"

struct Site site;
struct EKF ekf_x = {
	.q = 0.0002,
	.r = 1.8,
};
struct EKF ekf_y = {
	.q = 0.0002,
	.r = 1.8,
};
void Location_Type_Choose(void){
//	site.now.x = ladar.x;
//	site.now.y = ladar.y;
	site.now.x = (dt35.xcredible_flag == 1)?dt35.x:site.enc_pos.row_x;
	site.now.y = (dt35.ycredible_flag == 1)?dt35.y:site.enc_pos.row_y;
//	if(dt35.xcredible_flag == 1)
//		Set_Odometer_X(dt35.x);
	if(dt35.ycredible_flag == 1)
		Set_Odometer_Y(dt35.y);
	
	
	site.ladar_pos.row_x = ladar.x;
	site.ladar_pos.row_y = ladar.y;
  site.ladar_pos.with_odo_x = ladar.fuse_x;
	site.ladar_pos.with_odo_y = ladar.fuse_y;
	site.now.r = site.gyro_pos.r;
	//site.now.r = ladar.r;
}
//////////////////////////////////////////////像雷达一样在dt35的基础上做积分
void DT35_Fuse_With_Odometer(float dt){
	if((site.dt35_pos.x_credible_flag ==  2)){
		site.fuse_pos.x = site.dt35_pos.x; 
	}
	if((site.dt35_pos.y_credible_flag ==  2)){
		site.fuse_pos.y = site.dt35_pos.y;
	}
	site.fuse_pos.x+=site.gyro_pos.with_odo_vx*dt;
	site.fuse_pos.y+=site.gyro_pos.with_odo_vy*dt;
}


///////////////////////////////////////////////////////码盘与陀螺仪速度融合计算///////////////////////////////////////////////////////////////
//码盘的y轴是陀螺仪的-x轴
void Enc_VXVY_Fuse_With_Gyro_AXAY(float dt){
	site.car_pos.vx = EKF_Filter(&ekf_x,site.car_pos.row_vx,site.car_pos.ax*dt / 1000);
	site.car_pos.vy = EKF_Filter(&ekf_y,site.car_pos.row_vy,site.car_pos.ay*dt / 1000);
	site.gyro_pos.with_odo_vx  = site.car_pos.vx * cos(ang2rad(site.now.r)) - site.car_pos.vy * sin(ang2rad(site.now.r));
	site.gyro_pos.with_odo_vy  = site.car_pos.vx * sin(ang2rad(site.now.r)) + site.car_pos.vy * cos(ang2rad(site.now.r));

	//site.gyro_pos.with_odo_vx = EKF_Filter(&ekf_x,site.enc_pos.row_vx,site.gyro_pos.accx*dt / 1000);
  //site.gyro_pos.with_odo_vy = EKF_Filter(&ekf_y,site.enc_pos.row_vy,site.gyro_pos.accy*dt / 1000);
}
//////////////////////////////////////////////////////码盘与陀螺仪与雷达速度融合计算/////////////////////////////////////////////////////////////
struct EKF odo_ladar_ekf_x = {
	.q = 0.002,
	.r = 1.2,
};
struct EKF odo_ladar_ekf_y = {
	.q = 0.002,
	.r = 1.2,
};
//当前的思路：
/*
1.有雷达数据的时候直接把雷达数据输入滤波器；
2.没有雷达数据的时候，根据卡尔曼滤波的先验估计*0.4 + 马盘数据的0.6作为输入
3.卡尔曼滤波的增益一直是滤波过后的速度
4.当马盘的累计误差达到5000的时候，马盘的数据重新定位
*/
struct EKF odo_ladar_kalman_x = {
	.q = 0.002,
	.r = 1.2,
};
struct EKF odo_ladar_kalman_y = {
	.q = 0.002,
	.r = 1.2,
};

//////////////////////////////////////////////////////////////////////////////////雷达与马盘的数据融合/////////////////////////////////////////////////////////////////////
void Ladar_With_Odometer_Kalman(float dt){
	//给结构体赋值
	if(ladar.get_flag == 1){
		ladar.get_flag = 0;
		ladar.dx = 0;
		ladar.dy = 0;
	}
	else if(ladar.get_flag == 0){
		ladar.dx += site.enc_pos.row_vx * dt ;
		ladar.dy += site.enc_pos.row_vy * dt ;
	}
	ladar.fuse_x = ladar.x + ladar.dx;
	ladar.fuse_y = ladar.y + ladar.dy;
	site.ladar_pos.row_x = ladar.x;
	site.ladar_pos.row_y = ladar.y;
	site.ladar_pos.with_odo_x = EKF_Filter(&odo_ladar_kalman_x,ladar.fuse_x,site.gyro_pos.with_odo_vx * dt);;
	site.ladar_pos.with_odo_y = EKF_Filter(&odo_ladar_kalman_y,ladar.fuse_y,site.gyro_pos.with_odo_vy * dt);
}
void Ladar_With_Odometer_Relocation(float dt){
	if(ladar.get_flag == 1){
		ladar.get_flag = 0;
		ladar.dx = 0;
		ladar.dy = 0;
	}
	else if(ladar.get_flag == 0){
		ladar.dx += site.enc_pos.row_vx * dt * 1.00;
		ladar.dy += site.enc_pos.row_vy * dt * 1.00;
	}
	ladar.fuse_x = ladar.x + ladar.dx;
	ladar.fuse_y = ladar.y + ladar.dy;
	
	site.ladar_pos.row_x = ladar.x;
	site.ladar_pos.row_y = ladar.y;
	site.ladar_pos.with_odo_x = ladar.fuse_x;
	site.ladar_pos.with_odo_y = ladar.fuse_y;
}	











