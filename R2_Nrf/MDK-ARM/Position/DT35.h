#ifndef __DT35_H
#define __DT35_H
#include "Location.h"
#include "Fake_Rtos.h"
#include "Send.h"
enum Site_Base{
	incredible,
	left_base,
	behind_base,
	right_base,
	front_base
};

void DT35_Parameter_Init(void);

struct DT35_Cla_Data {

	float angle;
	float row_dis;
	float its_base_angle; 

	float its_base_dis;

	//因为误差所导致 DT35偏移的变量
	float offset_angle;
	float center_angle;
	float center_dis;
	float constant;

	float its_site_range1;
	float its_site_range2;
	 enum Site_Base site_base;

	float x;
	float xflag;
	float y;
	float yflag;
	
};
struct DT35{
	float right_angle_threshold1;
	float right_angle_threshold2;
	float behind_angle_threshold1;
	float behind_angle_threshold2;
	float left_angle_threshold1;
	float left_angle_threshold2;
	float front_angle_threshold1;
	float front_angle_threshold2;

	struct DT35_Cla_Data ldt35;
	struct DT35_Cla_Data rdt35;
	struct DT35_Cla_Data bdt35;
	struct DT35_Cla_Data fdt35;
	
  float x;
	float xcredible_flag; 
	float y;
	float ycredible_flag; 
};
struct DT35_Fuse{
	unsigned char x_data_num;
	unsigned char y_data_num;
  unsigned char x_num1;
	unsigned char x_num2;
	unsigned char y_num1;
	unsigned char y_num2;
};
extern struct DT35 dt35;
extern char DT35_Send_Flag;


void DT35_Cla_Flow(void);
void Get_DT35_Data(int id,unsigned char * data);
void DT35_Parameter_Init(void);
void Send_DT35_Data(void);
void DT35_Send_Pycharm(void);
#endif