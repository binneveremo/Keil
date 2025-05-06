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
		Tell_Yao_Xuan("jump");
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
#define BaksetNearest_Dis 1015
struct Basket_Lock{
	enum{
		far,
		near,
		middle,
	}status;
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
	}pid;
	struct{
		float basketdis;
		float nearbasketdis;
		float farbasketdis;
		float anglebetween_ladarandpole;
	} parameter;
	struct {
		float angle;
		float precent;
		float omiga;
		float gain;
		float basket_angle;
	}prediction;
};
struct Basket_Lock bl;
void BasketLock_ParameterInit(void){
	bl.pid.p = 70;
	bl.pid.d = 8;
	bl.pid.i = 2;
	bl.pid.istart = 0.7;
	bl.pid.iend = 7;
	bl.pid.ilimit = 1000;
	bl.pid.outlimit = 4500;
	bl.pid.accel_gain = 0.3;
	bl.pid.velocity_gain = 0.15;
	bl.pid.predict_step = 0.15;
	//想让车身偏左 就给大
	bl.parameter.anglebetween_ladarandpole = 5;
	//非常非常极限扣进的距离
//	bl.parameter.basketdis = 830;
//	bl.parameter.farbasketdis = 860;
//	bl.parameter.nearbasketdis = 800;
	//比较稳的能扣进
	bl.parameter.basketdis = 810;
	bl.parameter.farbasketdis = 840;
	bl.parameter.nearbasketdis = 780;
	//
	bl.prediction.gain = 20;
}
void JudgeBasketPos(void){
	bl.status = (vision.basket.ladar2basketdis > bl.parameter.farbasketdis)?far:((vision.basket.ladar2basketdis < bl.parameter.nearbasketdis)?near:middle);
}
void Basket_PIDInitReSet(void){



}


/*/////////////////////////////////////////
一些小技巧
1.当error小于一定程度的时候 会限制P的输出P会乘较小的增益 i会直接等于0
2.设置I的起始积分和终止积分 
3.
*///////////////////////////////////////////
float PreDiction_BasketAngle(void){
	bl.prediction.angle = sin(vision.basket.ladar2basketangle - atan2f(site.gyro_pos.with_odo_vy,site.gyro_pos.with_odo_vx));
	bl.prediction.precent = hypot(site.car_pos.row_vx,site.car_pos.row_vy) / vision.basket.ladar2basketdis;
	bl.prediction.omiga = bl.prediction.angle * bl.prediction.precent * bl.prediction.gain;
	
	bl.prediction.basket_angle = (vision.basket.get_flag == 1)?rad2ang(vision.basket.ladar2basketangle):(bl.prediction.basket_angle += bl.prediction.omiga);
	vision.basket.get_flag = (vision.basket.get_flag == 1)?0:vision.basket.get_flag; 
	return bl.prediction.basket_angle;
}
float BasketAngle_PIDOut(void){
	float error = rad2ang(vision.basket.ladar2basketangle) + PreDiction_BasketAngle() - bl.pid.predict_step * site.gyro_pos.omiga +   bl.parameter.anglebetween_ladarandpole ;
	float out;
	float p = bl.pid.p * error;
	float gain = Limit(bl.pid.velocity_gain * hypot(site.gyro_pos.with_odo_vx, site.gyro_pos.with_odo_vy) + bl.pid.accel_gain * hypot(site.gyro_pos.accx, site.gyro_pos.accy), 1, 4);
	float d = bl.pid.d * (error - bl.pid.error_last);
	
	bl.pid.error_last = error;                                                                                                                                                                          
	if(fabs(error) < bl.pid.istart){
		gain *= Limit(pow(error / bl.pid.istart,3),0,1);
		bl.pid.itotal *= gain;
	}
	else if ((fabs(error) > bl.pid.istart) && (fabs(error) < bl.pid.iend))
		bl.pid.itotal = Limit(bl.pid.itotal + bl.pid.i * error, -bl.pid.ilimit, bl.pid.ilimit);
	
	out = Limit(gain*p + bl.pid.itotal, -bl.pid.outlimit, bl.pid.outlimit);
	return out;
}
void GoToNearest_BasketPoint(void){
	//动态设置参数 1.设置死区 
	JudgeBasketPos();
	float PreDictDis = vision.basket.ladar2basketdis + 60 * site.car_pos.row_vx;
	float v = Limit(2 *(PreDictDis - bl.parameter.basketdis),VESC_START_VELOCITY,10000);
	float angle = vision.basket.ladar2basketangle;
	switch(bl.status){
		case far:
			Chassis_Velocity_Out(v * sin(angle),v * cos(angle),BasketAngle_PIDOut());
			break;
		case middle:
			Self_Lock_Out();
			break;
		case near:
			Chassis_Velocity_Out(-v * sin(angle),-v * cos(angle),BasketAngle_PIDOut());
			break;
	}
}

void Car_State_Decode(int id,unsigned char * data){
	if(id == 0xA1)
		car.flag_of.received = 1;
}
void Back(void){
	Set_Target_Point(home_point);
	Position_With_Mark_PID_Run();
}































