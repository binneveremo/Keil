#include "Communication.h"
#include "Television.h"
#include "Fake_Rtos.h"
#include "Chassis.h"
#include "Can_Bsp.h"
#include "string.h"
#include "Basket.h"
#include "Flow.h"

/*///////////////////////////////////
1.找到最近的篮筐半径
2.设置死区 只有到达死去才会自锁
3.动态设置所篮筐角度死区 距离越近 并且速度越慢 角度死区越小


*/////////////////////////////////////////
struct Flow flow;
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
	if(flow.state == start){
		flow.state = start2dunkpoint;
	}
	else if(flow.state == start2dunkpoint){
		Set_Target_Point(dunk_point);
		Position_With_Mark_PID_Run("default");
		if((Arrive_Point(dunk_point) == 1)){
			Tell_Yao_Xuan("catch");
			flow.state = wait_r1_ball;
		}
	}
	else if(flow.state == wait_r1_ball){
		Self_Lock_Out();
		if(R1.receive_ball_flag == 1){
			Tell_Yao_Xuan("predunk");
			flow.state = dunk;
		}
	}
	else if(flow.state == dunk){
		Tell_Yao_Xuan("jump");
	}
	else if(flow.state == end){
	
	}
}
void Flow_Reset(void){
	flow.state = start;
	flow.flag_of.received = 0;
	flow.flag_of.defend_send = 0;
	flow.flag_of.jump_send = 0;
}
void Run_Point_Test(void){
	Set_Target_Point(dunk_point);
	Position_With_Mark_PID_Run("default");
}

////////////////////////////////////编码器偏置测试//////////////////////////////////////////////


void Car_State_Decode(int id,unsigned char * data){
	if(id == 0xA1)
		flow.flag_of.received = 1;
}
void Back(void){
	Set_Target_Point(home_point);
	Position_With_Mark_PID_Run("default");
}

void ControlStatus_Detect(void){
	static char last;
	flow.flag_of.dribble = ((last != dribble) && (chassis.Control_Status == dribble))?1:flow.flag_of.dribble;
	flow.flag_of.dribble = ((last == dribble) && (chassis.Control_Status != dribble))?0:flow.flag_of.dribble;
	
	bl.flagof.nearest_point = ((last != progress) && (chassis.Control_Status == progress))?0:bl.flagof.nearest_point;
	
	last = chassis.Control_Status;
}

void Dribbble_Flow(void){
	static int begin;
	if(flow.flag_of.dribble == 1){
		Tell_Yao_Xuan("dribble");
		begin = HAL_GetTick();
		flow.flag_of.dribble = 0;
	}
	if(HAL_GetTick() - begin > 2000)
		Chassis_Velocity_Out(0,8000,0);
	else 
		Self_Lock_Out();
}






