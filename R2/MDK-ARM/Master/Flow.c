#include "Communication.h"
#include "Television.h"
#include "Fake_Rtos.h"
#include "Chassis.h"
#include "Can_Bsp.h"
#include "string.h"
#include "Flow.h"
struct Car car;
struct Point receive_point = {                              
 .x = 2000,
 .y = 2000,
 .r = 0
};
struct Point dunk_point = {
 .x = 12975,.y = -4000,.r = 0.4
};
float dunk_angle = 117;
struct Point test_point = {
	.x = 2000,
	.y = 1000,
	.r = 0
};

void Flow_Test(void){
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
		Tell_Yao_Xuan("jump");
		car.state = end;
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
void Flow_Test2(void){
	Chassis_Velocity_Out(0,0,Correct_Angle(dunk_angle));
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
struct Point home_point = {.x = 500,.y = -450,.r = -0.5};
void Back_Home(void){
	Set_Target_Point(home_point);
	Position_With_Mark_PID_Run();
	if(Near_Point(home_point) == 1)
		Self_Lock_Out();
}































