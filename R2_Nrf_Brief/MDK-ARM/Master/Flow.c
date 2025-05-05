#include "Communication.h"
#include "Television.h"
#include "Fake_Rtos.h"
#include "Chassis.h"
#include "Can_Bsp.h"
#include "string.h"
#include "Flow.h"
struct Car car;
struct Point home_point = {                              
  .x = 600,
  .y = -400,
  .r = 0
};
struct Point dunk_point = {
  .x = 12975,
	.y = -4000,
	.r = 0.4
};
struct Point basket_point = {
	.x = 13050,
	.y = -4000,
	.r = 0
};

void Flow(void){
	if(car.state == start){
		car.state = start2dunkpoint;
	}
	else if(car.state == start2dunkpoint){
		Set_Target_Point(dunk_point);
		Position_With_Mark_PID_Run();
		if((Arrive_Point(dunk_point) == 1)){
			Tell_Yao_Xuan("catch");
			car.state = wait_r1_ball;
		}
	}
	else if(car.state == wait_r1_ball){
		Self_Lock_Out();
		if(r1.receive_ball_flag == 1){
			Tell_Yao_Xuan("predunk");
			car.state = dunk;
		}
	}
	else if(car.state == dunk){
		//Tell_Yao_Xuan("jump");
		car.state = back;
	}
	else if(car.state == back){
		if(car.flag_of.back == 1){
			Set_Target_Point(home_point);
			Position_With_Mark_PID_Run();
		}
		if((car.flag_of.back == 1) && (Near_Point(home_point) == 1)){
			car.state = end;
			car.flag_of.back = 0;  
		}
	}
	else if(car.state == end){
	
	}
}
void Flow_Reset(void){
	car.state = start;
	car.flag_of.received = 0;
	car.flag_of.defend_send = 0;
	car.flag_of.jump_send = 0;
}
void Run_Point_Test(void){
	Set_Target_Point(dunk_point);
	Position_With_Mark_PID_Run();
}


///////////////////////////////////////雷达偏置测试////////////////////////////////////////
char ladar_offset_finish_flag;
float ladar_last_r;
void Ladar_Offset_Flow(void){
	Chassis_Velocity_Out(0,0,920);
	if(ladar_offset_finish_flag == 0){
		Send_Put_Data(0,ladar.rowr);
	}
	if(fabs(ladar.rowr - ladar_last_r) > 3)
		ladar_offset_finish_flag = 1;
	ladar_last_r =  ladar.rowr;
}
void Ladar_Offset_Test_Clear(void){
	ladar_offset_finish_flag = 0;
  ladar_last_r = ladar.rowr;
}
////////////////////////////////////编码器偏置测试//////////////////////////////////////////////
char odometer_offset_finish_flag;
float odometer_last_r;
void Odometer_Offset_Flow(void){
	Chassis_Velocity_Out(0,0,920);
	if(odometer_offset_finish_flag == 0){
		Send_Put_Data(0,site.now.r);
		send_flag = 1;
	}
	if(ang2rad(site.now.r) - odometer_last_r < -3)
		odometer_offset_finish_flag = 1;
	odometer_last_r = ang2rad(site.now.r);
}
void Odometer_Offset_Test_Clear(void){
	odometer_offset_finish_flag = 0;
  odometer_last_r = ang2rad(site.now.r);
}

void Car_State_Decode(int id,unsigned char * data){
	if(id == 0xA1)
		car.flag_of.received = 1;
}
void Back(void){
	Set_Target_Point(home_point);
	Position_With_Mark_PID_Run();
}































