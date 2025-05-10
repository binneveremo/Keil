#include "Chassis.h"
#include "Basket.h"
#include "Flow.h"
// 跑点以及底盘状态的结构体
struct Chassis chassis;
/////////////////////////////////////////////舵轮输出相关
void VectorWheel_SetSpeed(void)
{
	VESC_SetRPM(chassis.motor.drive[front_wheel].dir * chassis.motor.drive[front_wheel].rpm, front_drive_id);
	VESC_SetRPM(chassis.motor.drive[left_wheel].dir * chassis.motor.drive[left_wheel].rpm, left_drive_id);
	VESC_SetRPM(chassis.motor.drive[right_wheel].dir * chassis.motor.drive[right_wheel].rpm, right_drive_id);
	VESC_SetRPM(chassis.motor.drive[behind_wheel].dir * chassis.motor.drive[behind_wheel].rpm, behind_drive_id);
}
void VectorWheel_SetAngle(void)
{
	// Motor_Control_byFDCN(id,力矩, 速度 , 位置, 模式 , 使能, p, d, )
	Motor_Control_byFDCN(front_turn_send_id, 2.5,  30, chassis.motor.turn[front_wheel].target_angle + front_offset, 2, 1, 150,  55, &hfdcan1);
	Motor_Control_byFDCN(left_turn_send_id,  2.5,  30, chassis.motor.turn[left_wheel].target_angle  + left_offset,  2, 1, 150,  55, &hfdcan1);
	Motor_Control_byFDCN(right_turn_send_id, 2.5,  30, chassis.motor.turn[right_wheel].target_angle + right_offset, 2, 1, 150,  55, &hfdcan1);
	Motor_Control_byFDCN(behind_turn_send_id, 2.5, 30,chassis.motor.turn[behind_wheel].target_angle + behind_offset, 2, 1, 150, 55, &hfdcan1);
}
void Min_Angle_Cal(struct HO7213 *turn, struct VESC *drive, float target)
{
	float delta = turn->angle_now - target;
	int k = (int)floor((delta + 100.00) / 180.0f);
	float target_temp = target + 180.0f * k;
	int cycles = (int)(fabs(target_temp - target) / 180.0f);
	drive->dir = (cycles % 2 == 1) ? -1 : 1;
	turn->target_angle = target_temp;
}
// 只是计算出来每个电机期望的速度，并还没有输出
void Chassis_Velocity_Out(float left, float front, float anticlock)
{
	// 请记住 默认Y轴正方向为0度 也就是atan计算出来的角度是 相对于y轴的角度
	//  vx是left vy是front
	chassis.motor.drive[front_wheel].front = front;
	chassis.motor.drive[front_wheel].left = left + anticlock * 1.03;

	chassis.motor.drive[left_wheel].front = front - anticlock * 0.96547 ;
	chassis.motor.drive[left_wheel].left = left - anticlock * 0.2605;

	chassis.motor.drive[right_wheel].front = front + anticlock * 0.96547;
	chassis.motor.drive[right_wheel].left = left - anticlock * 0.2605;
	
	chassis.motor.drive[behind_wheel].front = front;
	chassis.motor.drive[behind_wheel].left = left - anticlock * 1.03;

	chassis.motor.drive[front_wheel].rpm = hypot(chassis.motor.drive[front_wheel].left,chassis.motor.drive[front_wheel].front);
	chassis.motor.drive[left_wheel].rpm =  hypot(chassis.motor.drive[left_wheel].left,chassis.motor.drive[left_wheel].front);
	chassis.motor.drive[right_wheel].rpm = hypot(chassis.motor.drive[right_wheel].left,chassis.motor.drive[right_wheel].front);
	chassis.motor.drive[behind_wheel].rpm = hypot(chassis.motor.drive[behind_wheel].left,chassis.motor.drive[behind_wheel].front);

	Min_Angle_Cal(&chassis.motor.turn[front_wheel], &chassis.motor.drive[front_wheel], rad2ang(atan2f(chassis.motor.drive[front_wheel].left, chassis.motor.drive[front_wheel].front)));
	Min_Angle_Cal(&chassis.motor.turn[left_wheel],  &chassis.motor.drive[left_wheel],  rad2ang(atan2f(chassis.motor.drive[left_wheel].left, chassis.motor.drive[left_wheel].front)));
	Min_Angle_Cal(&chassis.motor.turn[right_wheel], &chassis.motor.drive[right_wheel], rad2ang(atan2f(chassis.motor.drive[right_wheel].left, chassis.motor.drive[right_wheel].front)));
	Min_Angle_Cal(&chassis.motor.turn[behind_wheel], &chassis.motor.drive[behind_wheel], rad2ang(atan2f(chassis.motor.drive[behind_wheel].left, chassis.motor.drive[behind_wheel].front)));
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////手柄控制相关//////////////////////////////////////////////////////////////////////////////
void GamePad_Velocity_Standard(void)
{
	float rocker_x, rocker_y, rocker_r;
	rocker_x = (chassis.Flagof.GamePad_Inverse == 1)?-Game_Pad_Rocker_Data[1]:-Game_Pad_Rocker_Data[0];
	rocker_y = (chassis.Flagof.GamePad_Inverse == 1)?-Game_Pad_Rocker_Data[0]:Game_Pad_Rocker_Data[1];
	rocker_r = -Game_Pad_Rocker_Data[2];
	if (fabs(rocker_r) < 6)
		rocker_r = 0;
	float Rocker_GainT = chassis.Flagof.GamePad_Accel?127:(chassis.Flagof.GamePad_Slow?12:68);
	float Rocker_GainR = chassis.Flagof.GamePad_Accel?35: (chassis.Flagof.GamePad_Slow?(600 / 128):23);
	Chassis_Velocity_Out(Limit(rocker_x * Rocker_GainT,-20000,20000), Limit(rocker_y * Rocker_GainT,-20000,20000), Limit(rocker_r * Rocker_GainR,-9000,9000));
}
void GamePad_Velocity_FreeNoheader(void)
{
	float rocker_x, rocker_y,rocker_r;
	rocker_x = -Game_Pad_Rocker_Data[0];
	rocker_y = Game_Pad_Rocker_Data[1];
	rocker_r = -Game_Pad_Rocker_Data[2];
	// 陀螺仪逆时针为正
	float Rocker_GainT = chassis.Flagof.GamePad_Accel?130:(chassis.Flagof.GamePad_Slow?12:63);
	float Rocker_GainR = chassis.Flagof.GamePad_Accel?35: (chassis.Flagof.GamePad_Slow?(600 / 128):23);
	float r = (chassis.Flagof.GamePad_Inverse == 1)? site.now.r + 90:site.now.r;
	float y = (float)rocker_y * cos(ang2rad(r)) + rocker_x * sin(ang2rad(r));
	float x = (float)rocker_x * cos(ang2rad(r)) - rocker_y * sin(ang2rad(r));
	Chassis_Velocity_Out(x * Rocker_GainT, y * Rocker_GainT, rocker_r * Rocker_GainR);
}
void GamePad_Velocity_R1DirNoheader(void)
{
	float rocker_x, rocker_y,rocker_r;
	rocker_x = -Game_Pad_Rocker_Data[0];
	rocker_y = Game_Pad_Rocker_Data[1];
	rocker_r = -Game_Pad_Rocker_Data[2];
	// 陀螺仪逆时针为正
	float Rocker_GainT = chassis.Flagof.GamePad_Accel?115:(chassis.Flagof.GamePad_Slow?12:63);
	float Rocker_GainR = chassis.Flagof.GamePad_Accel?35: (chassis.Flagof.GamePad_Slow?(600 / 128):23);
	float r = (chassis.Flagof.GamePad_Inverse == 1)? site.now.r + 90:site.now.r;
	float y = (float)rocker_y * cos(ang2rad(r)) + rocker_x * sin(ang2rad(r));
	float x = (float)rocker_x * cos(ang2rad(r)) - rocker_y * sin(ang2rad(r));
	Chassis_Velocity_Out(x * Rocker_GainT, y * Rocker_GainT, BasketAngleLock());
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////跑点相关///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////三种跑法：1.位置PID 2.速度PID 3.位置PID+速度PID////////////////////////////////////////////////////////////////////////////////////
void Set_Target_Point(struct Point point){
	memcpy(&site.target, &point, sizeof(point));
}
struct cr_parameter{
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
};
struct cr_parameter cr;
float Correct_Angle(float target){
	float error = target - site.now.r;
	float out;
	float p = cr.p * error;
	float gain = Limit(cr.velocity_gain * site.car.velocity_totalenc + cr.accel_gain * site.car.accel_totalgyro, 1, 4);
	float d = cr.d * (error - cr.error_last);
	
	cr.error_last = error;                                                                                                                                                                          
	if(fabs(error) < cr.istart){
		gain *= Limit(pow(error / cr.istart,3),0,1);
		cr.itotal *= gain;
	}
	else if (fabs(error) > cr.istart)
		cr.itotal = Limit(cr.itotal + cr.i * error, -cr.ilimit, cr.ilimit);
	
	out = Limit(gain*p + cr.itotal, -cr.outlimit, cr.outlimit);
	return out;
}
void Correct_Angle_Par_Init(void){
	//////////////默认参数
	cr.p = 55;
	cr.d = 3;
	cr.i = 3;
	cr.ilimit = 900;
	cr.istart = 4;
	cr.outlimit = 5000;
	cr.accel_gain = 0.35;
	cr.velocity_gain = 0.2;
	/////////////锁篮筐参数
}

/////////////////////////////////////////4.位置闭环单向PID
struct Mark mark;
void Mark_PID_Par_Init(void){
	mark.p = 25;
	mark.i = 2;
	mark.istart = 6;
	mark.iend = 400;
	mark.ilimit = 900 / 1.414;
	mark.outlimit = 13700;

	mark.brake_distance = 460;
	
	mark.brake_percent = Limit(0.1 + 0.00001 * mark.outlimit,0,0.7);
	mark.brake_gain = 0.05;
	mark.brake_outlimit = 900;
	mark.brake_ilimit = 1000;
}
//////////////////////跑点的速度限制       根据最大速度最大速度 10000  那么就会限制刹车距离为 mark.brake_percent * 10000
void Position_With_Mark_PID_Run(char * anglelockdir)
{
	static struct Point last;
	if(Point_Distance(last,site.target) > 100){
		mark.total_dis = Point_Distance(site.target,site.now);
		mark.brake_distance = Limit(mark.brake_percent*mark.total_dis,800, mark.outlimit * mark.brake_percent);
		memcpy(&last,&site.target,sizeof(last));
	}
	float xerror = site.target.x - site.now.x;
	float yerror = site.target.y - site.now.y;
	float rerror = site.target.r - site.now.r;

	mark.gain = (hypot(xerror,yerror) > mark.brake_distance)?1:mark.brake_gain;
	float outlimit = mark.outlimit;
	float ilimit = mark.ilimit;
	
	float xp = mark.p * xerror;
	float xi = ((fabs(xerror) < mark.iend) && (fabs(xerror) > mark.istart))?mark.i * xerror:0;
	mark.itotal_x = (fabs(xerror) < mark.istart)?0:Limit(mark.itotal_x + xi, -ilimit, ilimit);
	mark.outx = mark.gain * xp + mark.itotal_x;

	float yp = mark.p * yerror;
	float yi = ((fabs(yerror) < mark.iend) && (fabs(yerror) > mark.istart))?mark.i * yerror:0;
	mark.itotal_y = (fabs(yerror) < mark.istart)?0:Limit(mark.itotal_y + yi, -ilimit, ilimit);
	mark.outy = mark.gain * yp + mark.itotal_y;

	// 请记住 第一项为front left 角速度
	float vnow = Limit(hypot(mark.outx, mark.outy), -outlimit,outlimit);
	float angle = atan2f(yerror, xerror);

	Chassis_Velocity_Out(vnow * sin(angle),vnow * cos(angle),Correct_Angle(site.target.r));
	
}
void Self_Lock_Out(void){
	for (int i = 0; i < VESC_NUM; i++)
		chassis.motor.drive[i].rpm = 0;
	Min_Angle_Cal(&chassis.motor.turn[front_wheel],   &chassis.motor.drive[front_wheel], 0);
	Min_Angle_Cal(&chassis.motor.turn[left_wheel],     &chassis.motor.drive[left_wheel], -83);
	Min_Angle_Cal(&chassis.motor.turn[right_wheel],   &chassis.motor.drive[right_wheel], 83);
	Min_Angle_Cal(&chassis.motor.turn[behind_wheel], &chassis.motor.drive[behind_wheel], 0);
}
#define xdeath 10
#define ydeath 10
#define rdeath 3
char GamePadControl_SelfLockAuto(void){
	char flag = (((chassis.Control_Status == gamepad_standard) || (chassis.Control_Status == gamepad_free_noheader) || (chassis.Control_Status == gamepad_r1dir_noheader)) && (fabs((float)Game_Pad_Rocker_Data[0]) <= 1) && (fabs((float)Game_Pad_Rocker_Data[1]) <= 1) && (fabs(Game_Pad_Rocker_Data[2]) <= 5))?1:0;
	if(flag == 1)
		strcpy(chassis.Flagof.lock_reason,"GamePadDataMin");
	return flag;
}
char GamePadLostConnection_SelfLockAuto(void){
	char flag = (GamepadLostConnection)?1:chassis.Flagof.self_lock;
	if(GamepadLostConnection == 1)
		strcpy(chassis.Flagof.lock_reason,"GamePadLossConnect");
	return flag;
}
char SafeMode_SelfLockAuto(void){
	char flag = (chassis.Control_Status == safe)?1:0;
	if(flag == 1)
		strcpy(chassis.Flagof.lock_reason,"SafeMode");
	return flag;
}
// 自动自锁函数
void Self_Lock_Auto(void){
	memset(chassis.Flagof.lock_reason,(char)NULL,sizeof(chassis.Flagof.lock_reason));
	switch(chassis.Control_Status){
		case safe:
			chassis.Flagof.self_lock = SafeMode_SelfLockAuto();
		break;
		case progress:
			chassis.Flagof.self_lock = BsaketPoint_SelfLockAuto();
		break;
		case back:
		break;
		case dribble:
		break;
		default:
			chassis.Flagof.self_lock = GamePadControl_SelfLockAuto();
		break;
	}
	chassis.Flagof.self_lock = GamePadLostConnection_SelfLockAuto();
	if(chassis.Flagof.self_lock == 1)
		Self_Lock_Out();
}
unsigned char Arrive_Point(struct Point point){
	if ((fabs(point.x - site.now.x) < 5 * xdeath) && (fabs(point.y - site.now.y) < 5 * ydeath) && (fabs(point.r - site.now.r) < 4 * rdeath))
		return 1;
	return 0;
}
/////////////////////////////////////////////////////////////////////////////////////转向电机解算//////////////////////////////////////////////////////////////////////////
void Turn_Motor_Decode(int id, unsigned char *data){
	if (id == front_turn_receive_id)
		chassis.motor.turn[front_wheel].angle_now = HO7213_Decode_for_FDC(data) - front_offset;
	else if (id == right_turn_receive_id)
		chassis.motor.turn[right_wheel].angle_now = HO7213_Decode_for_FDC(data) - right_offset;
	else if (id == left_turn_receive_id)
		chassis.motor.turn[left_wheel].angle_now = HO7213_Decode_for_FDC(data) - left_offset;
	else if (id == behind_turn_receive_id)
		chassis.motor.turn[behind_wheel].angle_now = HO7213_Decode_for_FDC(data) - behind_offset;
	else return;
}