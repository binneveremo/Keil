#include "Communication.h"
#include "Television.h"
#include "Fake_Rtos.h"
#include "Chassis.h"
#include "Can_Bsp.h"
#include "string.h"
#include "Flow.h"

/*///////////////////////////////////
1.找到最近的篮筐半径
2.设置死区 只有到达死去才会自锁
3.动态设置所篮筐角度死区 距离越近 并且速度越慢 角度死区越小


*//////////////////////////////////////////











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
		if(R1.receive_ball_flag == 1){
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

////////////////////////////////////编码器偏置测试//////////////////////////////////////////////
#define BaksetNearest_Dis 1030
struct {
	float velocity_gain;
	float accel_gain;

	
	float p;
	float i;
	float d;

	float predict_step;
	
	float error_last;
	float itotal;
	float ilimit;
	float istart;
	float iend;
	float outlimit;
}bp;
void Basket_PIDInit(void){
	bp.p = 70;
	bp.d = 8;
	bp.i = 3;
	bp.istart = 1.5;
	bp.iend = 7;
	bp.ilimit = 780;
	bp.outlimit = 4500;
	bp.accel_gain = 0.3;
	bp.velocity_gain = 0.15;
	bp.predict_step = 0.23;
}
/*/////////////////////////////////////////
一些小技巧
1.当error小于一定程度的时候 会限制P的输出P会乘较小的增益 i会直接等于0
2.设置I的起始积分和终止积分 
3.
*///////////////////////////////////////////

float BasketAngle_PIDOut(void){
	float error = rad2ang(vision.basket.ladar2basketangle) - bp.predict_step * site.gyro_pos.omiga;
	float out;
	float p = bp.p * error;
	float gain = Limit(bp.velocity_gain * hypot(site.gyro_pos.with_odo_vx, site.gyro_pos.with_odo_vy) + bp.accel_gain * hypot(site.gyro_pos.accx, site.gyro_pos.accy), 1, 4);
	float d = bp.d * (error - bp.error_last);
	
	bp.error_last = error;                                                                                                                                                                          
	if(fabs(error) < bp.istart){
		gain *= Limit(pow(error / bp.istart,3),0,1);
		bp.itotal *= gain;
	}
	else if ((fabs(error) > bp.istart) && (fabs(error) < bp.iend))
		bp.itotal = Limit(bp.itotal + bp.i * error, -bp.ilimit, bp.ilimit);
	
	out = Limit(gain*p + bp.itotal, -bp.outlimit, bp.outlimit);
	return out;
}
void GoToNearest_BasketPoint(void){
	//动态设置参数 1.设置死区  
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	float PreDictDis = vision.basket.ladar2basketdis + 60 * site.car_pos.row_vx;
	float v = Limit(2 *PreDictDis,1200,10000);
	float angle = vision.basket.ladar2basketangle;
	if(PreDictDis > BaksetNearest_Dis)
		Chassis_Velocity_Out(v * sin(angle),v * cos(angle),BasketAngle_PIDOut());
	else 
		Self_Lock_Out();
}

void Car_State_Decode(int id,unsigned char * data){
	if(id == 0xA1)
		car.flag_of.received = 1;
}
void Back(void){
	Set_Target_Point(home_point);
	Position_With_Mark_PID_Run();
}































