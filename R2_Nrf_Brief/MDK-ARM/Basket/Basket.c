#include "Basket.h"
struct EKF ladarx_interp = {
	.q = 0.002,
	.r = 1.8,
};
struct EKF ladary_interp = {
	.q = 0.002,
	.r = 1.8,
};
struct EKF ladarr_interp = {
	.q = 0.002,
	.r = 1.8,
};

struct Basket_Lock bl;
void BasketPositionCal_AccordingVision(float dt){
	//卡尔曼预测
	bl.position.now_interp.x = EKF_Filter(&ladarx_interp,vision.ladar.now.x,bl.parameter.siteinterp_gain*dt * site.field.vx_gyro);
	bl.position.now_interp.y = EKF_Filter(&ladary_interp,vision.ladar.now.y,bl.parameter.siteinterp_gain*dt * site.field.vy_gyro);
	bl.position.now_interp.r = NormalizeAng_Single(EKF_Filter(&ladarr_interp,vision.ladar.now.r,ang2rad(site.gyro.omiga)));
	
	bl.position.ladar2basketx = vision.ladar.basket.x - bl.position.now_interp.x;
	bl.position.ladar2baskety = vision.ladar.basket.y - bl.position.now_interp.y;
	bl.position.ladar2basketangle = atan2f(bl.position.ladar2baskety , bl.position.ladar2basketx);
	bl.position.ladar2basketdis = hypot(bl.position.ladar2basketx,bl.position.ladar2baskety);
}
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
	//比较稳的能扣进
	bl.parameter.basketdis = 810;
	bl.parameter.siteinterp_gain = 1.2;
}
void JudgeBasketPos(void){
	//bl.status = (bl.position.ladar2basketdis > bl.parameter.farbasketdis)?far:((bl.position.ladar2basketdis < bl.parameter.nearbasketdis)?near:middle);
}
void Basket_PIDInitReSet(void){



}



















/*/////////////////////////////////////////
一些小技巧
1.当error小于一定程度的时候 会限制P的输出P会乘较小的增益 i会直接等于0
2.设置I的起始积分和终止积分 
3.
*///////////////////////////////////////////
float BasketAngle_PIDOut(void){
	float error = rad2ang(bl.position.ladar2basketangle) - bl.pid.predict_step * site.gyro.omiga + bl.parameter.anglebetween_ladarandpole;
	float out;
	float p = bl.pid.p * error;
	float gain = Limit(bl.pid.velocity_gain * site.car.velocity_totalenc + bl.pid.accel_gain * site.car.accel_totalgyro, 1, 4);
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
	float targetbasketangle_field = bl.position.ladar2basketangle;
	if(bl.flagof.nearest_point == 0){
		bl.position.basket_target.x = vision.ladar.basket.x - bl.parameter.basketdis * cos(targetbasketangle_field);
		bl.position.basket_target.y = vision.ladar.basket.y - bl.parameter.basketdis * sin(targetbasketangle_field);
		bl.flagof.nearest_point = 1;
	}
	Set_Target_Point(bl.position.basket_target);
	Position_With_Mark_PID_Run("basket");
}

struct 
{
	float p;
	float i;
	float istart;
	float iend;
	float ilimit;
	float outlimit;
	float gain;
	
	float itotal_x;
	float itotal_y;
	float outx;
	float outy;

	
	///////新加的测试参数
	float total_dis;
	float brake_ilimit;
	float brake_outlimit;
	float brake_distance;
	float brake_gain;
	float brake_percent;
}br;

void BasketRunPoint_PIDParInit(void){
	br.p = 13;
	br.i = 5;
	br.istart = 6;
	br.iend = 400;
	br.ilimit = 900 / 1.414;
	br.outlimit = 8000;
	br.brake_distance = 460;
	br.brake_percent = Limit(0.1 + 0.00001 * br.outlimit,0,0.7);
	br.brake_gain = 0.05;
	br.brake_outlimit = 900;
	br.brake_ilimit = 1000;
}
void BasketRunPoint()
{
	float targetbasketangle_field = bl.position.ladar2basketangle;
	static struct Point last;
	if(bl.flagof.nearest_point == 0){
		bl.position.basket_target.x = vision.ladar.basket.x - bl.parameter.basketdis * cos(targetbasketangle_field);
		bl.position.basket_target.y = vision.ladar.basket.y - bl.parameter.basketdis * sin(targetbasketangle_field);
		bl.position.basket_target.r = rad2ang(bl.position.ladar2basketangle) + bl.parameter.anglebetween_ladarandpole - site.now.r;
		bl.flagof.nearest_point = 1;
	}
	if(Point_Distance(last,bl.position.now_interp) > 100){
		br.total_dis = Point_Distance(bl.position.basket_target,bl.position.now_interp);
		br.brake_distance = Limit(br.brake_percent*br.total_dis,800, br.outlimit * br.brake_percent);
		memcpy(&last,&bl.position.now_interp,sizeof(last));
	}
	float xerror = bl.position.basket_target.x - bl.position.now_interp.x;
	float yerror = bl.position.basket_target.y - bl.position.now_interp.y;

	br.gain = (hypot(xerror,yerror) > br.brake_distance)?1:br.brake_gain;
	float outlimit = br.outlimit;
	float ilimit = br.ilimit;
	
	float xp = br.p * xerror;
	float xi = ((fabs(xerror) < br.iend) && (fabs(xerror) > br.istart))?br.i * xerror:0;
	br.itotal_x = (fabs(xerror) < br.istart)?0:Limit(br.itotal_x + xi, -ilimit, ilimit);
	br.outx = br.gain * xp + br.itotal_x;

	float yp = br.p * yerror;
	float yi = ((fabs(yerror) < br.iend) && (fabs(yerror) > br.istart))?br.i * yerror:0;
	br.itotal_y = (fabs(yerror) < br.istart)?0:Limit(br.itotal_y + yi, -ilimit, ilimit);
	br.outy = br.gain * yp + br.itotal_y;

	// 请记住 第一项为front left 角速度
	float vnow = Limit(hypot(br.outx, br.outy), -outlimit,outlimit);
	float angle = atan2f(yerror, xerror) - ang2rad(bl.position.now_interp.r);

	Chassis_Velocity_Out(vnow * sin(angle),vnow * cos(angle),Correct_Angle(bl.position.basket_target.r));
	
	if((fabs(xerror) < 15) && (fabs(yerror) < 15))
		Self_Lock_Out();
}












