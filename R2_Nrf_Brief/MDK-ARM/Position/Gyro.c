#include "Kalman.h"
#include "Global.h"
#include "Gyro.h"
#include "usart.h"
#include "dma.h"
#include "string.h"
#include "math.h"
#include "fdcan.h"
unsigned char YIS512_Feedback[8];
struct YIS506 yis506;
struct Kalman_filter acc_x_filter = {
	.Q = 0.002f,
	.R = 1.0f,
	.C_last = 1,
};
struct Kalman_filter acc_y_filter ={
	.Q = 0.002f,
	.R = 1.0f,
	.C_last = 1,
};
struct Kalman_filter site_acc_x_filter ={
	.Q = 0.002f,
	.R = 1.0f,
	.C_last = 1,
};
struct Kalman_filter site_acc_y_filter ={
	.Q = 0.002f,
	.R = 1.0f,
	.C_last = 1,
};
struct EKF ekf_omiga = {
	.q = 0.0001,
	.r = 1.8,
};


void Gyro_AX_AY_Cal(void){
	site.car_pos.ax = Kalman_Filter(&site_acc_x_filter,-(yis506.X.accel - yis506.X.accel_offset));
	site.car_pos.ay = Kalman_Filter(&site_acc_y_filter,-(yis506.Y.accel - yis506.Y.accel_offset));
	site.gyro_pos.accx = site.car_pos.ax * cos(ang2rad(site.now.r)) - site.car_pos.ay * sin(ang2rad(site.now.r));
  site.gyro_pos.accy = site.car_pos.ax * sin(ang2rad(site.now.r)) + site.car_pos.ay * cos(ang2rad(site.now.r));
}
void Get_Gyro_Data(int header,unsigned char * data){
	memcpy(YIS512_Feedback,data,8);
  yis506.header = header;
	YIS506_Decode();
}
void Accel_Cal(void)
{
  //把指向一个八位的指针转成指向16位的指针，以达到位运算的目的
  yis506.X.accel = *(const uint16_t*)&(YIS512_Feedback[0])*accel_ratio-_accel_offset;  
  yis506.Y.accel = *(const uint16_t*)&(YIS512_Feedback[2])*accel_ratio-_accel_offset;  
  yis506.Z.accel = *(const uint16_t*)&(YIS512_Feedback[4])*accel_ratio-_accel_offset;  
}
/////////////////////////////////////////////////////////////////扩展卡尔曼滤波计算角速度////////////////////////////////////
struct{
  struct{
		int dt;
		float delta;
		float last;
		float delta_dt;
	}angle;
	struct{
		int dt;
		float delta;
		float last;
		float delta_dt;
	}omega;
}omega_filter;
float Omiga_Filter(float omega){
	static float last;
	float o = 0.7*last + 0.3*omega;
	last = omega;
	return o;
}
void Angle_Delta_Fuse_Get_Da_Dt(void){
	static int last;
	omega_filter.angle.delta = yis506.euler.yaw - omega_filter.angle.last;
	//计算dangle
	if(omega_filter.angle.delta > 90)
		omega_filter.angle.delta -= 360;
	else if(omega_filter.angle.delta < -90)
		omega_filter.angle.delta += 360;
	//计算dt
	omega_filter.angle.dt = HAL_GetTick() - last;
  //计算角速度
	omega_filter.angle.delta_dt = omega_filter.angle.delta / omega_filter.angle.dt * 1000;
	//存储上一次的值
	omega_filter.angle.last = yis506.euler.yaw;
	last = HAL_GetTick();
}
void Angle_Delta_Fuse_Get_Do_Dt(void){
	static int last;
	//计算角速度微分
	omega_filter.omega.delta = yis506.Z.omega - omega_filter.omega.last;
	//计算dt
	omega_filter.omega.dt = HAL_GetTick() - last;
  //计算角速度
	omega_filter.omega.delta_dt = omega_filter.omega.delta / omega_filter.omega.dt;
	//存储上一次的值
	omega_filter.omega.last = yis506.Z.omega;
	last = HAL_GetTick();
}

void Angle_Delta_Fuse_With_Omiga(void){
	static float angle_last;
	static int last;
	//计算角度差值 差值过大 进行结算
	float angle_delta = yis506.euler.yaw - angle_last;
	if(angle_delta > 90)
		angle_delta -= 180;
	else if(angle_delta < -90)
		angle_delta += 180;
	//计算时间微分
	int dt = HAL_GetTick() - last;
	float da_dt = angle_delta / dt * 1000; // 根据角度插值计算角速度 单位是角度每秒
	//存储上一次的数据
	angle_last = yis506.euler.yaw;
	last = HAL_GetTick();
	//进行拓展卡尔曼融合
	
}
//获取角度以及角速度
void Omega_Cal(void)
{
  unsigned char temp[4];
  //注意大端小端模式
  memmove(temp,&YIS512_Feedback[0],3);
  yis506.X.omega = ((*(uint32_t*)temp) &0x000fffff)*omega_ratio - _omega_offset;
  memmove(temp,&YIS512_Feedback[2],3);                
  yis506.Y.omega = ((*(uint32_t*)temp) >> 4)*omega_ratio - _omega_offset;
  memmove(temp,&YIS512_Feedback[5],3);
  yis506.Z.omega = ((*(uint32_t*)temp) &0x000fffff)*omega_ratio - _omega_offset;
	site.gyro_pos.omiga = yis506.Z.omega;
}
void Euler_Cal(void)
{
	yis506.euler.pitch = *(const unsigned short int*)&(YIS512_Feedback[0])*euler_radio -_euler_offset;
	yis506.euler.roll =  *(const unsigned short int*)&(YIS512_Feedback[2])*euler_radio -_euler_offset;
	yis506.euler.yaw  =  *(const unsigned short int*)&(YIS512_Feedback[4])*euler_radio -_euler_offset;
	float angle = yis506.euler.yaw - yis506.euler.yaw_offset;
	site.gyro_pos.r = (angle > 180)?angle - 360:(angle < -180 ? angle + 360:angle);
}
/////////////////////////////////////////////////计算陀螺仪的初始加速度偏置/////////////////////////////////////
char Gyro_Accel_Offset_Cal_NVIC(char time){
	static char init_flag,cnt;
	static float totalx,totaly;
	if(init_flag == 1)
		return 1;
	//前五帧数据可能没有加速度 保险起见 收集多帧数据
	if(cnt < time + 5){
		cnt++;
		if(cnt > 5){
			totalx += yis506.X.accel;
			totaly += yis506.Y.accel;
		}
		if(cnt >= time + 5){
			yis506.X.accel_offset = totalx / time;
		  yis506.Y.accel_offset = totaly / time;
		  init_flag = 1;
		}
	}
	return 0;
}
///////////////////////////////////////////t陀螺仪相关数据初始化//////////////////////////////
void Gyro_Init(void){
	static char cnt;
	char flag;
  for(int i = 0; i < 20; i++){
		flag = Gyro_Accel_Offset_Cal_NVIC(5);
		if(flag == 1)
			break;
		HAL_Delay(100);
	}
}
//////////////////////////////////////////陀螺仪中断解算//////////////////////////////////
void YIS506_Decode(void){
		memcpy(YIS512_Feedback,YIS512_Feedback,8);
		if(yis506.header == ExtID_yaw)
			Euler_Cal();
		else if(yis506.header == ExtID_accel)
			Accel_Cal();
		else if(yis506.header == ExtID_omega)
			Omega_Cal();
}
/////////////////////////////////////////////定期修正陀螺仪角度/////////////////////////////
void Gyro_Reset(void){
	yis506.euler.yaw_offset = yis506.euler.yaw;
	MX_FDCAN2_Init();
}




























