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
struct Point basket_point = {
	.x = 12050,
	.y = -4000,
	.r = 0
};
struct Basket_Lock bl;
void BasketPositionCal_AccordingVision(float dt){
	//卡尔曼预测
	bl.position.now_interp_vfield.x = EKF_Filter(&ladarx_interp,vision.ladar.now_vfield.x,bl.parameter.siteinterp_gain*bl.parameter.siteinterp_gain*dt * site.field.vx_gyro);
	bl.position.now_interp_vfield.y = EKF_Filter(&ladary_interp,vision.ladar.now_vfield.y,bl.parameter.siteinterp_gain*bl.parameter.siteinterp_gain*dt * site.field.vy_gyro);
	bl.position.now_interp_vfield.r = NormalizeAng_Single(EKF_Filter(&ladarr_interp,vision.ladar.now_vfield.r,bl.parameter.angleinterp_gain*ang2rad(site.gyro.omiga)));
	
	bl.position.ladar2basketx = vision.ladar.basket.x - bl.position.now_interp_vfield.x;
	bl.position.ladar2baskety = vision.ladar.basket.y - bl.position.now_interp_vfield.y;
	bl.position.ladar2basketangle = atan2f(bl.position.ladar2baskety , bl.position.ladar2basketx);
	bl.position.ladar2basketdis = hypot(bl.position.ladar2basketx,bl.position.ladar2baskety);
	
	bl.position.backwardladar_field.x = basket_point.x - bl.position.ladar2basketdis*cos(bl.position.ladar2basketangle + bl.parameter.ladar_offsetrad - ang2rad(site.gyro.r - bl.position.now_interp_vfield.r));
	bl.position.backwardladar_field.y = basket_point.y - bl.position.ladar2basketdis*sin(bl.position.ladar2basketangle + bl.parameter.ladar_offsetrad - ang2rad(site.gyro.r - bl.position.now_interp_vfield.r));
	
	bl.position.backwardcar_field.x = bl.position.backwardladar_field.x - 261.31 * (sin(2 * PI * 0.16 * ang2rad(vision.ladar.now_vfield.r) + 1.61) - sin(1.61));
	bl.position.backwardcar_field.y = bl.position.backwardladar_field.y - 258.26 * (sin(2 * PI * 0.16 * ang2rad(vision.ladar.now_vfield.r) + 0.04) - sin(0.04));
	
}
struct BasketAngle_Lock ba;
struct BasketPosition_Lock bp;
void BasketAngleLock_ParInit(void){
	ba.p = 55;
	ba.d = 3;
	ba.i = 3;
	ba.ilimit = 1000;
	ba.istart = 0.2;
	ba.iend = 6;
	ba.outlimit = 5000;
	ba.accel_gain = 0.35;
	ba.velocity_gain = 0.2;
	//想让车身偏左 就给大
	bl.parameter.anglebetween_ladarandpole = 4;
	bl.parameter.basketdis = 780;
	bl.parameter.siteinterp_gain = 1.1;
	bl.parameter.angleinterp_gain = 0.6;
	bl.parameter.ladar_offsetrad = 0;
}
float BasketAngleLock(void){
	float error = bl.position.basket_target_vfield.r - bl.position.now_interp_vfield.r;
	float out;
	float p = ba.p * error;
	float gain = Limit(ba.velocity_gain * site.car.velocity_totalenc + ba.accel_gain * site.car.accel_totalgyro, 1, 4);
	float d = ba.d * (error - ba.error_last);
	
	ba.error_last = error;                                                                                                                                                                          
	if(fabs(error) < ba.istart){
		gain *= Limit(pow(error / ba.istart,3),0,1);
		ba.itotal *= gain;
	}
	else if (fabs(error) > ba.istart)
		ba.itotal = Limit(ba.itotal + ba.i * error, -ba.ilimit, ba.ilimit);
	
	out = Limit(gain*p + ba.itotal, -ba.outlimit, ba.outlimit);
	return out;
}
void BasketPositionLock_ParInit(void){
	
	bp.p = 15;
	bp.i = 2;
	bp.istart = 8;
	bp.iend = 400;
	bp.ilimit = 900 / 1.414;
	bp.outlimit = 8000;
	bp.slow_circle = 26;

	bp.brake_distance = 460;
	bp.brake_percent = Limit(0.1 + 0.000012 * bp.outlimit,0,0.7);
	bp.brake_gain = 0.1;
	bp.brake_outlimit = 900;
	bp.brake_ilimit = 1000;
	
}
void BasketPoint_Init(void){
	if(bl.flagof.nearest_point != 0) return;
	bl.position.basket_target_vfield.x = vision.ladar.basket.x - bl.parameter.basketdis * cos(bl.position.ladar2basketangle);
	bl.position.basket_target_vfield.y = vision.ladar.basket.y - bl.parameter.basketdis * sin(bl.position.ladar2basketangle);
	bl.position.basket_target_vfield.r = rad2ang(bl.position.ladar2basketangle) + bl.parameter.anglebetween_ladarandpole;
	char repeat = (bl.position.basket_target_vfield.x > vision.ladar.basket.x)?1:0;
	bl.position.basket_target_vfield.x = (repeat == 1)?vision.ladar.basket.x + bl.parameter.basketdis * cos(bl.position.ladar2basketangle):bl.position.basket_target_vfield.x;
	bl.position.basket_target_vfield.r = (repeat == 1)?NormalizeAng_Single(180 - rad2ang(bl.position.ladar2basketangle) - bl.parameter.anglebetween_ladarandpole):bl.position.basket_target_vfield.r;
	
	bp.itotal_x = 0;
	bp.itotal_y = 0;
	bl.flagof.nearest_point = 1;
	bp.total_dis = Point_Distance(bl.position.basket_target_vfield,bl.position.now_interp_vfield);
	bp.brake_distance = Limit(bp.brake_percent*bp.total_dis, 1000, bp.outlimit * bp.brake_percent);
}
char BsaketPoint_SelfLockAuto(void){
	/////////////////////////////判断是否距离目标点更远了 收集历史数据 如果比历史的三个点都远 就认为更远了//////////////////
	static float dis_history[5];
	for(int i = 4; i > 0;i--)
		dis_history[i] = dis_history[i - 1];
	char lagernum = (char)NULL;
	float disnow = Point_Distance(bl.position.basket_target_vfield,bl.position.now_interp_vfield);
	for(int i = 0; i<5;i++)
		lagernum = (disnow > dis_history[i])?lagernum + 1:lagernum;
	dis_history[0] = disnow;
	//距离目标点变远
	bp.Lock_Flag = ((lagernum >= 5) && (hypot(bl.position.basket_target_vfield.x - bl.position.now_interp_vfield.x,bl.position.basket_target_vfield.y - bl.position.now_interp_vfield.y) < 25))?1:bp.Lock_Flag;
	//到达死区
	bp.Lock_Flag = ((hypot(bl.position.basket_target_vfield.x - bl.position.now_interp_vfield.x,bl.position.basket_target_vfield.y - bl.position.now_interp_vfield.y) < 15))?1:bp.Lock_Flag;
	
	bp.Lock_Flag = ((hypot(bl.position.basket_target_vfield.x - bl.position.now_interp_vfield.x,bl.position.basket_target_vfield.y - bl.position.now_interp_vfield.y) > 200))?0:bp.Lock_Flag;
	if(bp.Lock_Flag == 1)
		strcpy(chassis.Flagof.lock_reason,"BasketNear         ");
	return bp.Lock_Flag;
}

void BasketPositionLock()
{
	BasketPoint_Init();
	float xerror = bl.position.basket_target_vfield.x - bl.position.now_interp_vfield.x;
	float yerror = bl.position.basket_target_vfield.y - bl.position.now_interp_vfield.y;
	bp.gain = (hypot(xerror,yerror) > bp.brake_distance)?1:bp.brake_gain;
	float igain = (hypot(xerror,yerror) < bp.slow_circle)?0.8:1;
	float outlimit = bp.outlimit;
	float ilimit = bp.ilimit;
	
	float xp = bp.p * xerror;
	float xi = ((fabs(xerror) < bp.iend) && (fabs(xerror) > bp.istart))?bp.i * xerror:0;
	bp.itotal_x = (fabs(xerror) < bp.istart)?0:Limit(bp.itotal_x + xi, -ilimit, ilimit);
	bp.outx = bp.gain * xp + bp.itotal_x*igain;

	float yp = bp.p * yerror;
	float yi = ((fabs(yerror) < bp.iend) && (fabs(yerror) > bp.istart))?bp.i * yerror:0;
	bp.itotal_y = (fabs(yerror) < bp.istart)?0:Limit(bp.itotal_y + yi, -ilimit, ilimit);
	bp.outy = bp.gain * yp + bp.itotal_y*igain;

	// 请记住 第一项为front left 角速度
	float vnow = Limit(hypot(bp.outx, bp.outy), -outlimit,outlimit);
	float angle = atan2f(yerror, xerror) - ang2rad(bl.position.now_interp_vfield.r);

	Chassis_Velocity_Out(vnow * sin(angle),vnow * cos(angle),BasketAngleLock());	
	
	BsaketPoint_SelfLockAuto();
}











void Vfield_Convert_Field(struct Point * field,struct Point vfield){
	float diff_angle = ang2rad(vision.ladar.now_vfield.r - site.gyro.r);
	float diff_x = basket_point.x - (vision.ladar.basket.x*  cos(diff_angle) - vision.ladar.basket.y * sin(diff_angle));
	float diff_y = basket_point.y - (vision.ladar.basket.y*  cos(diff_angle) + vision.ladar.basket.x * sin(diff_angle));
	field->x = vfield.x * cos(diff_angle) - vfield.y * sin(diff_angle) + diff_x;
	field->y = vfield.y * cos(diff_angle) + vfield.x * sin(diff_angle) + diff_y;
	field->r = site.gyro.r;
} 










