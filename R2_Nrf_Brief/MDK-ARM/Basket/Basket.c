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
	.x = 13050,
	.y = -4000,
	.r = 0
};
struct Basket_Lock bl;
void BasketPositionCal_AccordingVision(float dt){
	//卡尔曼预测
	bl.position.now_interp_vfield.x = EKF_Filter(&ladarx_interp,vision.ladar.now.x,bl.parameter.siteinterp_gain*dt * site.field.vx_gyro);
	bl.position.now_interp_vfield.y = EKF_Filter(&ladary_interp,vision.ladar.now.y,bl.parameter.siteinterp_gain*dt * site.field.vy_gyro);
	bl.position.now_interp_vfield.r = NormalizeAng_Single(EKF_Filter(&ladarr_interp,vision.ladar.now.r,ang2rad(site.gyro.omiga)));
	
	bl.position.ladar2basketx = vision.ladar.basket.x - bl.position.now_interp_vfield.x;
	bl.position.ladar2baskety = vision.ladar.basket.y - bl.position.now_interp_vfield.y;
	bl.position.ladar2basketangle = atan2f(bl.position.ladar2baskety , bl.position.ladar2basketx);
	bl.position.ladar2basketdis = hypot(bl.position.ladar2basketx,bl.position.ladar2baskety);
	
	bl.position.backwardladar_field.x = basket_point.x - cos(bl.position.ladar2basketangle + ang2rad(site.now.r));
	bl.position.backwardladar_field.y = basket_point.y - sin(bl.position.ladar2basketangle + ang2rad(site.now.r));
}
struct BasketAngle_Lock ba;
struct BasketPosition_Lock bp;
void BasketAngleLock_ParInit(void){
	ba.p = 30;
	ba.i = 2;
	ba.istart = 0.2;
	ba.iend = 4;
	ba.ilimit = 900;
	ba.outlimit = 2000;
	//想让车身偏左 就给大
	bl.parameter.anglebetween_ladarandpole = 5;
	bl.parameter.basketdis = 810;
	bl.parameter.siteinterp_gain = 1.2;
}
float BasketAngleLock(void){
	static float error_last;
	float error = bl.position.basket_target_vfield.r - ba.predict_step * site.gyro.omiga + bl.parameter.anglebetween_ladarandpole;
	float out;
	float p = ba.p * error;
	error_last = error;
	ba.itotal = ((fabs(error) > ba.istart) && (fabs(error) < ba.iend))?Limit(ba.itotal + ba.i * error, -ba.ilimit, ba.ilimit):((fabs(error) < ba.istart)?0:ba.itotal);
	out = Limit(p + ba.itotal, -ba.outlimit, ba.outlimit);
	return out;
}
void BasketPositionLock_ParInit(void){
	
	bp.p = 8;
	bp.i = 0.8;
	bp.istart = 7;
	bp.iend = 500;
	bp.ilimit = 900 / 1.414;
	bp.outlimit = 8000;

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
	bl.position.basket_target_vfield.r = (repeat == 1)?NormalizeAng_Single(180 - rad2ang(bl.position.ladar2basketangle) + bl.parameter.anglebetween_ladarandpole):bl.position.basket_target_vfield.r;
	
	bp.itotal_x = 0;
	bp.itotal_y = 0;
	bl.flagof.nearest_point = 1;
	bp.total_dis = Point_Distance(bl.position.basket_target_vfield,bl.position.now_interp_vfield);
	bp.brake_distance = Limit(bp.brake_percent*bp.total_dis, 1000, bp.outlimit * bp.brake_percent);
}
void BasketPositionLock()
{
	BasketPoint_Init();
	float xerror = bl.position.basket_target_vfield.x - bl.position.now_interp_vfield.x;
	float yerror = bl.position.basket_target_vfield.y - bl.position.now_interp_vfield.y;
	bp.gain = (hypot(xerror,yerror) > bp.brake_distance)?1:bp.brake_gain;
	float outlimit = bp.outlimit;
	float ilimit = bp.ilimit;
	
	float xp = bp.p * xerror;
	float xi = ((fabs(xerror) < bp.iend) && (fabs(xerror) > bp.istart))?bp.i * xerror:0;
	bp.itotal_x = (fabs(xerror) < bp.istart)?0:Limit(bp.itotal_x + xi, -ilimit, ilimit);
	bp.outx = bp.gain * xp + bp.itotal_x;

	float yp = bp.p * yerror;
	float yi = ((fabs(yerror) < bp.iend) && (fabs(yerror) > bp.istart))?bp.i * yerror:0;
	bp.itotal_y = (fabs(yerror) < bp.istart)?0:Limit(bp.itotal_y + yi, -ilimit, ilimit);
	bp.outy = bp.gain * yp + bp.itotal_y;

	// 请记住 第一项为front left 角速度
	float vnow = Limit(hypot(bp.outx, bp.outy), -outlimit,outlimit);
	float angle = atan2f(yerror, xerror) - ang2rad(bl.position.now_interp_vfield.r);

	Chassis_Velocity_Out(vnow * sin(angle),vnow * cos(angle),Correct_Angle(bl.position.basket_target_vfield.r) + site.now.r);	
}












