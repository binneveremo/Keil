#include "Television.h"
#include "Chassis.h"


#define xdeath 14
#define ydeath 14
#define rdeath 2

// 跑点以及底盘状态的结构体
struct Chassis chassis;
/////////////////////////////////////////////舵轮输出相关
void VectorWheel_SetSpeed(void)
{
#if Small_Green
	VESC_SetRPM(chassis.motor.drive[frontleft].dir *  chassis.motor.drive[frontleft].rpm, front_left_drive_id);
	VESC_SetRPM(chassis.motor.drive[frontright].dir * chassis.motor.drive[frontright].rpm, front_right_drive_id);
	VESC_SetRPM(chassis.motor.drive[behindleft].dir * c
	hassis.motor.drive[behindleft].rpm, behind_left_drive_id);
	VESC_SetRPM(chassis.motor.drive[behindright].dir * chassis.motor.drive[behindright].rpm, behind_right_drive_id);
#else 
	VESC_SetRPM(chassis.motor.drive[front_wheel].dir * chassis.motor.drive[front_wheel].rpm, front_drive_id);
	VESC_SetRPM(chassis.motor.drive[left_wheel].dir * chassis.motor.drive[left_wheel].rpm, left_drive_id);
	VESC_SetRPM(chassis.motor.drive[right_wheel].dir * chassis.motor.drive[right_wheel].rpm, right_drive_id);
	VESC_SetRPM(chassis.motor.drive[behind_wheel].dir * chassis.motor.drive[behind_wheel].rpm, behind_drive_id);
#endif
}
void VectorWheel_SetAngle(void)
{
	// Motor_Control_byFDCN(id,力矩, 速度 , 位置, 模式 , 使能, p, d, )
#if Small_Green
	Motor_Control_byFDCN(front_left_turn_send_id, 2.5, 30, chassis.motor.turn[frontleft].target_angle + frontleft_offset, 2, 1, 100, 40, &hfdcan1);
	Motor_Control_byFDCN(front_right_turn_send_id, 2.5, 30, chassis.motor.turn[frontright].target_angle + frontright_offset, 2, 1, 100, 40, &hfdcan1);
	Motor_Control_byFDCN(behind_left_turn_send_id, 2.5, 30, chassis.motor.turn[behindleft].target_angle + behindleft_offset, 2, 1, 100, 40, &hfdcan1);
	Motor_Control_byFDCN(behind_right_turn_send_id, 2.5, 30, chassis.motor.turn[behindright].target_angle + behindright_offset, 2, 1, 100, 40, &hfdcan1);
#else 
	Motor_Control_byFDCN(front_turn_send_id, 2.5, 30, chassis.motor.turn[front_wheel].target_angle + front_offset, 2, 1, 70, 30, &hfdcan1);
	Motor_Control_byFDCN(left_turn_send_id,  2.5, 30, chassis.motor.turn[left_wheel].target_angle  + left_offset,  2, 1, 70, 30, &hfdcan1);
	Motor_Control_byFDCN(right_turn_send_id, 2.5, 30, chassis.motor.turn[right_wheel].target_angle + right_offset, 2, 1, 60, 27, &hfdcan1);
	Motor_Control_byFDCN(behind_turn_send_id, 2.5, 30,chassis.motor.turn[behind_wheel].target_angle + behind_offset, 2, 1, 60, 27, &hfdcan1);

#endif
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
#if Small_Green
	chassis.motor.drive[frontright].front =  front + anticlock * 0.707;
	chassis.motor.drive[frontright].left =  left  + anticlock * 0.707;

	chassis.motor.drive[frontleft].front =   front - anticlock * 0.707;
	chassis.motor.drive[frontleft].left =   left  + anticlock * 0.707;

	chassis.motor.drive[behindright].front = front + anticlock * 0.707;
	chassis.motor.drive[behindright].left = left  - anticlock * 0.707;

	chassis.motor.drive[behindleft].front =  front - anticlock * 0.707;
	chassis.motor.drive[behindleft].left =  left  - anticlock * 0.707;

	chassis.motor.drive[frontright].rpm = hypot(chassis.motor.drive[frontright].front, chassis.motor.drive[frontright].left);
	chassis.motor.drive[frontleft].rpm =  hypot(chassis.motor.drive[frontleft].front, chassis.motor.drive[frontleft].left);   
	chassis.motor.drive[behindright].rpm = hypot(chassis.motor.drive[behindright].front, chassis.motor.drive[behindright].left);   
	chassis.motor.drive[behindleft].rpm =  hypot(chassis.motor.drive[behindleft].front, chassis.motor.drive[behindleft].left);   

	Min_Angle_Cal(&chassis.motor.turn[frontright],  &chassis.motor.drive[frontright],  rad2ang(atan2f(chassis.motor.drive[frontright].left,  chassis.motor.drive[frontright].front)));
	Min_Angle_Cal(&chassis.motor.turn[frontleft],   &chassis.motor.drive[frontleft],   rad2ang(atan2f(chassis.motor.drive[frontleft].left,   chassis.motor.drive[frontleft].front)));
	Min_Angle_Cal(&chassis.motor.turn[behindright], &chassis.motor.drive[behindright], rad2ang(atan2f(chassis.motor.drive[behindright].left, chassis.motor.drive[behindright].front)));
	Min_Angle_Cal(&chassis.motor.turn[behindleft],  &chassis.motor.drive[behindleft],  rad2ang(atan2f(chassis.motor.drive[behindleft].left,  chassis.motor.drive[behindleft].front)));
#else
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
#endif
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////手柄控制相关//////////////////////////////////////////////////////////////////////////////
char GamePad_Velocity_Standard_InverseFlag;
char GamePad_Velocity_Standard_SlowFlag;
char GamePad_Velocity_Standard_AccelFlag;
void GamePad_Velocity_Standard(void)
{
	float rocker_x, rocker_y, rocker_r;
	rocker_x = (GamePad_Velocity_Standard_InverseFlag == 1)? Game_Pad_Rocker_Data[1]:-Game_Pad_Rocker_Data[0];
	rocker_y = (GamePad_Velocity_Standard_InverseFlag == 1)?-Game_Pad_Rocker_Data[0]:Game_Pad_Rocker_Data[1];
	rocker_r = -Game_Pad_Rocker_Data[2];
	if (fabs(rocker_r) < 6)
		rocker_r = 0;
	float Rocker_GainT = GamePad_Velocity_Standard_AccelFlag?127:(GamePad_Velocity_Standard_SlowFlag?12:68);
	float Rocker_GainR = GamePad_Velocity_Standard_AccelFlag?35: (GamePad_Velocity_Standard_SlowFlag?(600 / 128):23);
	Chassis_Velocity_Out(Limit(rocker_x * Rocker_GainT,-20000,20000), Limit(rocker_y * Rocker_GainT,-20000,20000), Limit(rocker_r * Rocker_GainR,-9000,9000));
	chassis.self_control = none;
}
char GamePad_NoHeader_R1Dir_Flag;
void GamePad_Velocity_Noheader(void)
{
	float rocker_x, rocker_y,rocker_r;
	rocker_x = -Game_Pad_Rocker_Data[0];
	rocker_y = Game_Pad_Rocker_Data[1];
	rocker_r = -Game_Pad_Rocker_Data[2];
	// 陀螺仪逆时针为正
	float Rocker_GainT = GamePad_Velocity_Standard_AccelFlag?115:(GamePad_Velocity_Standard_SlowFlag?12:63);
	float Rocker_GainR = GamePad_Velocity_Standard_AccelFlag?35: (GamePad_Velocity_Standard_SlowFlag?(600 / 128):23);
	float r = (GamePad_Velocity_Standard_InverseFlag == 1)? site.now.r + 90:site.now.r;
	float y = (float)rocker_y * cos(ang2rad(r)) + rocker_x * sin(ang2rad(r));
	float x = (float)rocker_x * cos(ang2rad(r)) - rocker_y * sin(ang2rad(r));
	Chassis_Velocity_Out(x * Rocker_GainT, y * Rocker_GainT, (GamePad_NoHeader_R1Dir_Flag)?Correct_Angle(r1.oppsite_angle):rocker_r * Rocker_GainR);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////跑点相关///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////三种跑法：1.位置PID 2.速度PID 3.位置PID+速度PID////////////////////////////////////////////////////////////////////////////////////
void Set_Target_Point(struct Point point)
{
	memcpy(&site.target, &point, sizeof(point));
}
///////////////////////////////////1.单纯速度PID
//carxerror与y相关 输出项是left 结算没有问题 left恰好是y轴坐标
//float carxerrornow = yerror * cos(ang2rad(site.rnow))  + xerror * sin(ang2rad(site.rnow));
//float caryerrornow = yerror * -sin(ang2rad(site.rnow)) + xerror * cos(ang2rad(site.rnow));
struct
{
	float target_x;
	float p_x;
	float i_x;
	float ix_start; // 积分起始点
	float ix_end;	// 积分结束点
	float ix_limit;
	float d_x;
	float error_x;
	float error_x_pre;
	float error_sum_x;
	float gain_x;
	float xout;
	float xlimit;

	float target_y;
	float p_y;
	float i_y;
	float iy_start;
	float iy_end;
	float iy_limit;
	float d_y;
	float error_y;
	float error_y_pre;
	float error_sum_y;
	float yout;
	float ylimit;

	float target_r;
	float p_r;
	float i_r;
	float ir_start;
	float ir_end;
	float ir_limit;
	float d_r;
	float error_r;
	float error_r_pre;
	float error_sum_r;
	float rout;
	float rlimit;

	float feed_p;
} velocity;
void Velocity_Run_Par_Init(void)
{
	// 比例项
	velocity.p_x = 3200;
	velocity.p_y = 3200;
	velocity.p_r = 31;
	// 积分项
	velocity.i_x = 50;
	velocity.i_y = 50;
	velocity.i_r = 5;
	//
	velocity.d_x = 1;
	velocity.d_y = 1;
	velocity.d_r = 1;
	// 积分分离
	velocity.ix_start = 0.005;
	velocity.ix_end = 2.0;
	velocity.iy_start = 0.005;
	velocity.iy_end = 2.0;
	velocity.ir_start = 0.6;
	velocity.ir_end = 40;
	// 积分限幅
	velocity.ix_limit = 2000;
	velocity.iy_limit = 2000;
	velocity.ir_limit = 200;
	// 输出限幅
	velocity.xlimit = 40000;
	velocity.ylimit = 40000;
	velocity.rlimit = 6000;
	// 前馈比例
	velocity.feed_p = 2000;
}
// 速度闭环PID 角度是相对于中轴线的角度 逆时针为正
void Velocity_Run(float vx, float vy, float angle){
	//选择误差来源
	velocity.error_x = vx - site.car_pos.vx;
	velocity.error_y = vy - site.car_pos.vy;
	velocity.error_r = angle - site.now.r;

	float px = velocity.p_x * velocity.error_x;
	velocity.error_sum_x = Limit(velocity.error_sum_x + velocity.error_x, -velocity.ix_limit / velocity.i_x, velocity.ix_limit / velocity.i_x);
	if (fabs(velocity.error_x) < velocity.ix_start)
		velocity.error_sum_x *= 0.85;
	float ix = velocity.i_x * velocity.error_sum_x;
	float dx = velocity.d_x * (velocity.error_x - velocity.error_x_pre);
	velocity.error_x_pre = velocity.error_x;
	velocity.xout = Limit(px + ix + dx + vx * velocity.feed_p, -velocity.xlimit, velocity.xlimit);

	float py = velocity.p_y * velocity.error_y;
	velocity.error_sum_y = Limit(velocity.error_sum_y + velocity.error_y, -velocity.iy_limit / velocity.i_y, velocity.iy_limit / velocity.i_y);
	if (fabs(velocity.error_y) < velocity.iy_start)
		velocity.error_sum_y *= 0.85;
	float iy = velocity.i_y * velocity.error_sum_y;
	float dy = velocity.d_y * (velocity.error_y - velocity.error_y_pre);
	velocity.error_y_pre = velocity.error_y;
	velocity.yout = Limit(py + iy + dy + vy * velocity.feed_p, -velocity.ylimit, velocity.ylimit);

	float pr = velocity.p_r * velocity.error_r;
	velocity.error_sum_r += velocity.error_r;
	if ((fabs(velocity.error_r) < velocity.ir_start))
		velocity.error_sum_r *= 0.8;
	float ir = velocity.i_r * velocity.error_sum_r;
	velocity.rout = Limit(pr + ir, -velocity.rlimit, velocity.rlimit);

	Chassis_Velocity_Out(velocity.yout, velocity.xout, 0);
}
void Velocity_Run_Test(float vx,float vy,float angle){
	velocity.error_x = vx - site.car_pos.vx;
	velocity.error_y = vy - site.car_pos.vy;
	velocity.error_r = angle - site.now.r;
	//计算
	float px = velocity.p_x * velocity.error_x;
	velocity.error_sum_x = Limit(velocity.error_sum_x + velocity.error_x, -velocity.ix_limit / velocity.i_x, velocity.ix_limit / velocity.i_x);
	if (fabs(velocity.error_x) < velocity.ix_start)
		velocity.error_sum_x *= 0.98;
	float ix = velocity.i_x * velocity.error_sum_x;
	float dx = velocity.d_x * (velocity.error_x - velocity.error_x_pre);
	velocity.error_x_pre = velocity.error_x;
	velocity.xout = Limit(px + ix + dx + vx * velocity.feed_p, -velocity.xlimit, velocity.xlimit);

	float py = velocity.p_y * velocity.error_y;
	velocity.error_sum_y = Limit(velocity.error_sum_y + velocity.error_y, -velocity.iy_limit / velocity.i_y, velocity.iy_limit / velocity.i_y);
	if (fabs(velocity.error_y) < velocity.iy_start)
		velocity.error_sum_y *= 0.98;
	float iy = velocity.i_y * velocity.error_sum_y;
	float dy = velocity.d_y * (velocity.error_y - velocity.error_y_pre);
	velocity.error_y_pre = velocity.error_y;
	velocity.yout = Limit(py + iy + dy + vy * velocity.feed_p, -velocity.ylimit, velocity.ylimit);

	float accel = hypot(site.gyro_pos.accx,site.gyro_pos.accy);
	
	float pr = velocity.p_r * velocity.error_r;
	velocity.error_sum_r = Limit(velocity.error_sum_r + velocity.error_r, -velocity.ir_limit / velocity.i_r, velocity.ir_limit / velocity.i_r);
	if (fabs(velocity.error_r) < velocity.ir_start)
		velocity.error_sum_r *= 0.95;
	float ir = velocity.i_r * velocity.error_sum_r;
	float dr = velocity.d_r * (velocity.error_r - velocity.error_r_pre);
	velocity.error_r_pre = velocity.error_r;
	velocity.rout = Limit(pr + ir + dr - site.gyro_pos.omiga* velocity.feed_p * 0.005, -velocity.rlimit , velocity.rlimit);

	Chassis_Velocity_Out(velocity.yout, velocity.xout, velocity.rout);
}
////////////////////////////////////////////////2.单纯跑点PID

struct {
	float velocity_gain;
	float accel_gain;

	float p;
	float i;
	float d;

	float error_last;
	float itotal;
	float ilimit;
	float istart;
	float outlimit;
}cr;
float Correct_Angle(float target){
	float error = target - site.now.r;
	float out;
	float p = cr.p * error;
	float gain = Limit(cr.velocity_gain * hypot(site.gyro_pos.with_odo_vx, site.gyro_pos.with_odo_vy) + cr.accel_gain * hypot(site.gyro_pos.accx, site.gyro_pos.accy), 1, 4);
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
	cr.p = 55;
	cr.d = 3;
	cr.i = 3;
	cr.ilimit = 800;
	cr.istart = 4;
	cr.outlimit = 5000;
	                   
	cr.accel_gain = 0.35;
	cr.velocity_gain = 0.2;
}

/////////////////////////////////////////4.位置闭环单向PID
struct Mark mark;
void Mark_PID_Par_Init(void){
	mark.p = 18;
	mark.i = 5;
	mark.istart = 3;
	mark.iend = 500;
	mark.ilimit = 900 / 1.414;
	mark.outlimit = 13700;

	mark.brake_distance = 460;
	
	mark.brake_percent = Limit(0.23 + 0.00001 * mark.outlimit,0,0.7);
	mark.brake_gain = 0.1;
	mark.brake_outlimit = 900;
	mark.brake_ilimit = 1000;
}
//////////////////////跑点的速度限制       根据最大速度最大速度 10000  那么就会限制刹车距离为 mark.brake_percent * 10000
void Position_With_Mark_PID_Run(void)
{
	static struct Point last;
	if(Point_Distance(last,site.target) > 100){
		mark.total_dis = Point_Distance(site.target,site.now);
		mark.brake_distance = Limit(mark.brake_percent*mark.total_dis,300, mark.outlimit * mark.brake_percent);
		memcpy(&last,&site.target,sizeof(last));
	}
	chassis.self_control = run_point;
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
	float angle = atan2f(yerror, xerror) - ang2rad(site.now.r);

	
	Chassis_Velocity_Out(vnow * sin(angle),vnow * cos(angle),Correct_Angle(site.target.r));
}
//////////////////////////////////////////////////////////////跑点自动自锁 ： 距离越来越远 、 速度很小不足以驱动  /////////////////////////////////////////////////////////////////////
unsigned char Velocity_Equal_Zero(void){
	static float vx_past[10], vy_past[10];
	unsigned char x_num = 0, y_num = 0;
	for (int i = 0; i < 9; i++)
	{
		vx_past[i] = vx_past[i + 1];
		vy_past[i] = vy_past[i + 1];
	} 
	vx_past[9] = site.enc_pos.row_vx;
	vy_past[9] = site.enc_pos.row_vy;
	for (int i = 0; i < 10; i++)
	{
		if (vx_past[i] < 0.01)
			x_num++;
		if (vy_past[i] < 0.01)
			y_num++;
	}
	if ((x_num == 10) && (y_num == 10))
		return 1;
	return 0;
}
unsigned char Becoming_Longer(void){
	unsigned char num = 14;
	unsigned char bigger_num = 0;
	static float site_past[20];
	// 处理距离变远
	float dis_now = hypot(site.now.x - site.target.x,site.now.y - site.target.y);
	for (int i = 0; i < num - 1; i++)
	{
		if (site_past[i + 1] > site_past[i])
			bigger_num++;
		site_past[i] = site_past[i + 1];
	}
	site_past[num - 1] = dis_now;
	// 处理速度为零
	if (bigger_num > num - 3)
		return 1;
	return 0;
}
void Self_Lock_Out(void){
	for (int i = 0; i < 4; i++)
		chassis.motor.drive[i].rpm = 0;
#if Small_Green
	Min_Angle_Cal(&chassis.motor.turn[frontright], &chassis.motor.drive[frontright], -45);
	Min_Angle_Cal(&chassis.motor.turn[frontleft], &chassis.motor.drive[frontleft], 45);
	Min_Angle_Cal(&chassis.motor.turn[behindright], &chassis.motor.drive[behindright], 45);
	Min_Angle_Cal(&chassis.motor.turn[behindleft], &chassis.motor.drive[behindleft], -45);
#else 
	Min_Angle_Cal(&chassis.motor.turn[front_wheel], &chassis.motor.drive[front_wheel], 0);
	Min_Angle_Cal(&chassis.motor.turn[left_wheel], &chassis.motor.drive[left_wheel], -83);
	Min_Angle_Cal(&chassis.motor.turn[right_wheel], &chassis.motor.drive[right_wheel], 83);
	Min_Angle_Cal(&chassis.motor.turn[behind_wheel], &chassis.motor.drive[behind_wheel], 0);
#endif
}
void Self_Brake_Out(void){
	for (int i = 0; i < 4; i++)
		chassis.motor.drive[i].rpm = 0;
#if Small_Green
	Min_Angle_Cal(&chassis.motor.turn[behindright], &chassis.motor.drive[behindright], 90);
	Min_Angle_Cal(&chassis.motor.turn[behindleft], &chassis.motor.drive[behindleft], -90);
#else 
	Min_Angle_Cal(&chassis.motor.turn[right_wheel], &chassis.motor.drive[right_wheel], 90);
	Min_Angle_Cal(&chassis.motor.turn[left_wheel], &chassis.motor.drive[left_wheel], -90);
#endif
}
// 自动自锁函数
void Self_Lock_Auto(void){
		///////////////////////手柄断链安全模式////////////////////
	if (GamepadLostConnection == 1)
	{
		chassis.self_lock_flag = 1;
		if (GamepadLostConnection == 0)
			chassis.self_lock_flag = 0;
	}
	// 手柄控制时候的自锁
	else if (chassis.control_status == GamePad_Control)
	{
		if ((fabs((float)Game_Pad_Rocker_Data[0]) <= 1) && (fabs((float)Game_Pad_Rocker_Data[1]) <= 1) && (fabs(Game_Pad_Rocker_Data[2]) <= 5))
			chassis.self_lock_flag = 1;
		else
			chassis.self_lock_flag = 0;
	}
	// 跑点时候的自动自锁
	else if (chassis.self_control == run_point)
	{
		if (((fabs(site.target.x - site.now.x) < xdeath) && (fabs(site.target.y - site.now.y) < ydeath) && (fabs(site.target.r - site.now.r) < rdeath)))
			chassis.self_lock_flag = 1;
//		if ((fabs(site.target.x - site.now.x) < 2 * xdeath) && (fabs(site.target.y - site.now.y) < 2 * ydeath) && (fabs(site.target.r - site.now.r) < rdeath) && (Becoming_Longer() == 1))
//			chassis.self_lock_flag = 1;
//		if ((fabs(site.target.x - site.now.x) < 2 * xdeath) && (fabs(site.target.y - site.now.y) < 2 * ydeath) && (fabs(site.target.r - site.now.r) < rdeath) && (Velocity_Equal_Zero() == 1))
//			chassis.self_lock_flag = 1;
		if ((fabs(site.target.x - site.now.x) > 10 * xdeath) || (fabs(site.target.y - site.now.y) > 6 * ydeath) || (fabs(site.target.r - site.now.r) > 5 * rdeath))
			chassis.self_lock_flag = 0;
	}
	// 视觉控制时候自动自锁
	else if(chassis.control_status == Safe_Control)
		chassis.self_lock_flag = 1;
	else
		chassis.self_lock_flag = 0;
	///////////////////////////////标志位自锁/////////////////////////////
	if (chassis.self_lock_flag == 1)
		Self_Lock_Out();
}
unsigned char Arrive_Point(struct Point point){
	if ((fabs(point.x - site.now.x) < 5 * xdeath) && (fabs(point.y - site.now.y) < 5 * ydeath) && (fabs(point.r - site.now.r) < 4 * rdeath))
		return 1;
	return 0;
}
#define Near_Dis 2000
unsigned char Near_Point(struct Point point){
	if (hypot(point.x - site.now.x, point.y - site.now.y) < Near_Dis)
		return 1;
	return 0;
}

/////////////////////////////////////////////////////////////////////////////////////转向电机解算//////////////////////////////////////////////////////////////////////////
void Turn_Motor_Decode(int id, unsigned char *data){
#if Small_Green
	if (id == front_left_turn_receive_id)
		chassis.motor.turn[frontleft].angle_now = HO7213_Decode_for_FDC(data) - frontleft_offset;
	else if (id == front_right_turn_receive_id)
		chassis.motor.turn[frontright].angle_now = HO7213_Decode_for_FDC(data) - frontright_offset;
	else if (id == behind_left_turn_receive_id)
		chassis.motor.turn[behindleft].angle_now = HO7213_Decode_for_FDC(data) - behindleft_offset;
	else if (id == behind_right_turn_receive_id)
		chassis.motor.turn[behindright].angle_now = HO7213_Decode_for_FDC(data) - behindright_offset;
	else return;
#else 
	if (id == front_turn_receive_id)
		chassis.motor.turn[front_wheel].angle_now = HO7213_Decode_for_FDC(data) - front_offset;
	else if (id == right_turn_receive_id)
		chassis.motor.turn[right_wheel].angle_now = HO7213_Decode_for_FDC(data) - right_offset;
	else if (id == left_turn_receive_id)
		chassis.motor.turn[left_wheel].angle_now = HO7213_Decode_for_FDC(data) - left_offset;
	else if (id == behind_turn_receive_id)
		chassis.motor.turn[behind_wheel].angle_now = HO7213_Decode_for_FDC(data) - behind_offset;
	else return;
#endif

}

/////////////////////////////////////////////////////////////////////////安全模式/////////////////////////////////////////////////////
struct {
	char will_hit_flag;
	



}safe_flag;

void will_be_hiting(void){



} 
////////////////////////
float Dribble_Velocity = 9400;
int Dribble_Begin;
int last;
void Dribble_Flow(void){
	if(HAL_GetTick() - last > 2050)
		Chassis_Velocity_Out(0,Dribble_Velocity,0);
}
void Debug_Detect(void){
	static char status_last;
	if((status_last == GamePad_Control) && (chassis.control_status == Debug_Control)){
		Dribble_Begin = 1;
		Tell_Yao_Xuan("dribble");
	}
	if(Dribble_Begin == 0)
		last = HAL_GetTick();
	status_last = chassis.control_status;
}