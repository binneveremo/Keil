#include "DT35.h"
#include "string.h"
#include "math.h"

//默认的数据最大的抖动范围
#define longth 5000
#define wideth 8000
#define max_wave 0.1f
//下面是定义接收DT35的id
#define front_dt35_id 0xD1
#define right_dt35_id  0xD2
#define behind_dt35_id 0xD3
#define left_dt35_id 0xD4
//车体中心距离车体边界的距离 也就是中心和定位的数据之差
#define right2car_dis 298.44f
#define behind2car_dis 253.9f
//真实车体中心距离DT35中心的距离
#define left_go 0
#define right_go 1
//模糊角度
#define rough_angle 9
/*///////////////////////////////////////////
DT35计算流程：
1.判断dt35打在了哪个边界上
*/////////////////////////////////////
unsigned char DT35_Data[4][2];
struct DT35 dt35;
struct DT35_Fuse dt35_fuse_data;
//右侧 "angle_offset":  3.1893,"center_dis": -247.0620 ,"center_angle": 0.0582, "constant": 0,"
//前面 "angle_offset": -0.0281,"center_dis":337.6807 ,"center_angle":  -0.3577, "constant": 0
//左侧 "angle_offset": -0.0317,"center_dis":223.8846 ,"center_angle":  0.5374,
//后面 "angle_offset": -0.0277,"center_dis":315.4189 ,"center_angle": -0.3377
//左侧数据可能需要重新标定
void DT35_Parameter_Init(void){
	//右侧参数
	dt35.rdt35.center_angle = 0.0582;
	dt35.rdt35.center_dis = -247.0620;
	dt35.rdt35.constant =  0;
	dt35.rdt35.offset_angle = 3.1893;
	//后面参数
	dt35.bdt35.center_angle = -0.3377;
	dt35.bdt35.center_dis = 315.4189;
	dt35.bdt35.constant = 0;
	dt35.bdt35.offset_angle = -0.0277;
	//左侧参数
	dt35.ldt35.center_angle = -0.4671;
	dt35.ldt35.center_dis = 365.3687;
	dt35.ldt35.constant = -130.4025;
	dt35.ldt35.offset_angle = -0.0317;
	//前面参数
	dt35.fdt35.center_angle = -0.3577;
	dt35.fdt35.center_dis = 337.6807;
	dt35.fdt35.constant = 0;
	dt35.fdt35.offset_angle = -0.0281;
}
void Get_DT35_Data(int id,unsigned char * data){
    if(id == front_dt35_id)
      memcpy(&DT35_Data[0],data,2);	
    else if(id == right_dt35_id)
      memcpy(&DT35_Data[1],data,2);
    else if(id == behind_dt35_id)
      memcpy(&DT35_Data[2],data,2);
    else if(id == left_dt35_id)
      memcpy(&DT35_Data[3],data,2);
}
enum Site_Base Judge_In_Range(float angle){
	if((angle >= dt35.left_angle_threshold1) && (angle <= dt35.left_angle_threshold2))
		return left_base;
	else if((angle >= dt35.behind_angle_threshold1) && (angle <= dt35.behind_angle_threshold2))
		return behind_base;
	else if((angle >= dt35.right_angle_threshold1) && (angle <= dt35.right_angle_threshold2))
		return right_base;
	else if((angle >= dt35.front_angle_threshold1) || (angle <= dt35.front_angle_threshold2))
		return front_base;
    return incredible;
}
float Normallize_Angle(float angle){
	angle = angle > 360 ? angle - 360 : angle;
	angle = angle < 0 ? angle + 360 : angle;
	return angle;
}
void Each_DT35_Site_Cla(struct DT35_Cla_Data *dt35){
	switch(dt35->site_base){
		case left_base:
		  dt35->y = wideth - dt35->its_base_dis;
		  dt35->yflag = 1;
		  dt35->x = 0;
		  dt35->xflag = 0; 
		break;
		case right_base:
			dt35->y = dt35->its_base_dis;
		  dt35->yflag = 1;
		  dt35->x = 0;
		  dt35->xflag = 0;
		break;
		case behind_base:
			dt35->x = dt35->its_base_dis;
		  dt35->xflag = 1;
			dt35->y = 0;
		  dt35->yflag = 0;
		break;
		case front_base:
			dt35->x = longth - dt35->its_base_dis;
		  dt35->xflag = 1;
			dt35->y = 0;
		  dt35->yflag = 0;
		break;
		default:
			dt35->xflag = 0;
			dt35->yflag = 0;
			dt35->x = 0;
			dt35->y = 0;
		break;
	}
}
void Each_DT35_Wave(struct DT35_Cla_Data *dt35){
	if(dt35->site_base != incredible){
		float wave_rate = dt35->row_dis /  65535 * max_wave;
		if(dt35->xflag == 1){
			dt35->its_site_range1 = dt35->x*(1 - wave_rate);
			dt35->its_site_range2 = dt35->x*(1 + wave_rate);
		}
		if(dt35->yflag == 1){
			dt35->its_site_range1 = dt35->y*(1 - wave_rate);
			dt35->its_site_range2 = dt35->y*(1 + wave_rate);
		}
	}
}
int Judge_OverLap(float range11,float range12,float range21,float range22){
	if((range12 < range21) || (range22 < range11))
		return 0;
	return 1;
}
struct DT35_Cla_Data DT35_Search(int id){
	if(id == 0)
		return dt35.fdt35;
	else if(id == 1)
		return dt35.rdt35;
	else if(id == 2)
		return dt35.bdt35;
	else
		return dt35.ldt35;
}
void The_Number_Of_DT35_Which_Has_Data_Flag(void){
	memset(&dt35_fuse_data,0,sizeof(dt35_fuse_data));
	for(int i = 0; i < 4 ; i++){
		if(DT35_Search(i).xflag== 1){
			dt35_fuse_data.x_data_num++;
			if(dt35_fuse_data.x_num1 != 0)
				dt35_fuse_data.x_num2 = i+1;
			if(dt35_fuse_data.x_num2 == 0)
			  dt35_fuse_data.x_num1 = i + 1;
		}
		if(DT35_Search(i).yflag== 1){
			dt35_fuse_data.y_data_num++;
			if(dt35_fuse_data.y_num1 != 0)
				dt35_fuse_data.y_num2 = i+1;
			if(dt35_fuse_data.y_num2 == 0)
			 dt35_fuse_data.y_num1 = i + 1;
		}
	}
}
void DT35_Data_Fuse(float mix_x,float mix_y){
	float credible_percent;
	dt35.x = 0;
	dt35.xcredible_flag = 0;
	dt35.y = 0;
	dt35.ycredible_flag = 0;
	if(dt35_fuse_data.x_data_num == 2){
		 if(Judge_OverLap(DT35_Search(dt35_fuse_data.x_num1 - 1).its_site_range1,DT35_Search(dt35_fuse_data.x_num1 - 1).its_site_range2,DT35_Search(dt35_fuse_data.x_num2 - 1).its_site_range1,DT35_Search(dt35_fuse_data.x_num2 - 1).its_site_range2) == 1){                                               
				credible_percent = DT35_Search(dt35_fuse_data.x_num1 - 1).x / (DT35_Search(dt35_fuse_data.x_num1 - 1).x + DT35_Search(dt35_fuse_data.x_num2 - 1).x);
				dt35.x = DT35_Search(dt35_fuse_data.x_num1 - 1).x*(1 - credible_percent) + DT35_Search(dt35_fuse_data.x_num2 - 1).x* credible_percent;
			  if(Judge_OverLap(dt35.x*(1 - max_wave),dt35.x*(1 + max_wave),mix_x*(1 - max_wave),mix_x*(1 + max_wave)))  
				dt35.xcredible_flag = 2;
     }
		 if(dt35.xcredible_flag == 0){
			 if(Judge_OverLap(DT35_Search(dt35_fuse_data.x_num1 - 1).its_site_range1,DT35_Search(dt35_fuse_data.x_num1 - 1).its_site_range2,mix_x*(1 - 0.3*max_wave),mix_x*(1 + 0.3*max_wave)) == 1){
					 dt35.x = DT35_Search(dt35_fuse_data.x_num1 - 1).x;
					 dt35.xcredible_flag = 1;
	    }
			 else if(Judge_OverLap(DT35_Search(dt35_fuse_data.x_num2 - 1).its_site_range1,DT35_Search(dt35_fuse_data.x_num2 - 1).its_site_range2,mix_x*(1 - 0.3*max_wave),mix_x*(1 + 0.3*max_wave)) == 1){
					 dt35.x = DT35_Search(dt35_fuse_data.x_num2 - 1).x;
					 dt35.xcredible_flag = 1;
	    }
		 }
	 }
	if(dt35_fuse_data.y_data_num == 2){
		if(Judge_OverLap(DT35_Search(dt35_fuse_data.y_num1 - 1).its_site_range1,DT35_Search(dt35_fuse_data.y_num1 - 1).its_site_range2,DT35_Search(dt35_fuse_data.y_num2 - 1).its_site_range1,DT35_Search(dt35_fuse_data.y_num2 - 1).its_site_range2) == 1){                                               
			credible_percent = DT35_Search(dt35_fuse_data.y_num1 - 1).y / (DT35_Search(dt35_fuse_data.y_num1 - 1).y + DT35_Search(dt35_fuse_data.y_num2 - 1).y);
			dt35.y = DT35_Search(dt35_fuse_data.x_num1 - 1).y*(1 - credible_percent) + DT35_Search(dt35_fuse_data.x_num2 - 1).y* credible_percent;
			if(Judge_OverLap(dt35.y*(1 - max_wave),dt35.y*(1 + max_wave),mix_y*(1 - max_wave),mix_y*(1 + max_wave)))
			dt35.ycredible_flag = 2;
		}
		if(dt35.ycredible_flag == 0){
			if(Judge_OverLap(DT35_Search(dt35_fuse_data.y_num1 - 1).its_site_range1,DT35_Search(dt35_fuse_data.y_num1 - 1).its_site_range2,mix_y*(1 - 0.3*max_wave),mix_y*(1 + 0.3*max_wave)) == 1){
			 dt35.y = DT35_Search(dt35_fuse_data.y_num1 - 1).y;
			 dt35.ycredible_flag = 1;
	    }
			else if(Judge_OverLap(DT35_Search(dt35_fuse_data.y_num2 - 1).its_site_range1,DT35_Search(dt35_fuse_data.y_num2 - 1).its_site_range2,mix_y*(1 - 0.3*max_wave),mix_y*(1 + 0.3*max_wave)) == 1){
			 dt35.y = DT35_Search(dt35_fuse_data.y_num2 - 1).y;
			 dt35.ycredible_flag = 1;
	    }
		}
	}
	 if(dt35_fuse_data.x_data_num == 1){
		 if(Judge_OverLap(DT35_Search(dt35_fuse_data.x_num1 - 1).its_site_range1,DT35_Search(dt35_fuse_data.x_num1 - 1).its_site_range2,mix_y*(1 - 0.3*max_wave),mix_y*(1 + 0.3*max_wave)) == 1){
			 dt35.x = DT35_Search(dt35_fuse_data.x_num1 - 1).x;
			 dt35.xcredible_flag = 1;
	    }
	 }
	 if(dt35_fuse_data.y_data_num == 1){
		 if(Judge_OverLap(DT35_Search(dt35_fuse_data.y_num1 - 1).its_site_range1,DT35_Search(dt35_fuse_data.y_num1 - 1).its_site_range2,mix_y*(1 - 0.3*max_wave),mix_y*(1 + 0.3*max_wave)) == 1){
			 dt35.y = DT35_Search(dt35_fuse_data.y_num1 - 1).y;
			 dt35.ycredible_flag = 1;
	    }
	  }
}
void RowDis_To_CenterDis(struct DT35_Cla_Data * dt35){
	dt35->its_base_dis = dt35->row_dis * cos(ang2rad(site.now.r) + dt35->offset_angle) + dt35->center_dis * cos(dt35->center_angle - ang2rad(site.now.r)) + dt35->constant;
}
void DT35_Cla_Flow(void){
	//第一步 获取原生数据
	dt35.fdt35.row_dis = ((DT35_Data[0][1] << 8) + DT35_Data[0][0]) * 0.1;
  dt35.rdt35.row_dis = ((DT35_Data[1][1] << 8) + DT35_Data[1][0]) * 0.1;
	dt35.bdt35.row_dis = ((DT35_Data[2][1] << 8) + DT35_Data[2][0]) * 0.1;
	dt35.ldt35.row_dis = ((DT35_Data[3][1] << 8) + DT35_Data[3][0]) * 0.1;
    //归一化           从0到360度
	//计算每个DT35相对于场地坐标系的角度
//	float x = site.now.y + 386 , y = site.now.x + 396;
//	dt35.fdt35.angle =  Normallize_Angle(site.now.r);
//	dt35.ldt35.angle =  Normallize_Angle(site.now.r + 90);
//	dt35.bdt35.angle =  Normallize_Angle(site.now.r + 180);
//	dt35.rdt35.angle =  Normallize_Angle(site.now.r + 270);
//	//根据当前定位计算阈值
//	dt35.left_angle_threshold1 = dt35.front_angle_threshold2 + 2 * rough_angle;
//	dt35.left_angle_threshold2 = rad2ang(atan(x / (wideth - y))) + 90 - rough_angle;
//	dt35.behind_angle_threshold1 = dt35.left_angle_threshold2 + 2 * rough_angle;
//	dt35.behind_angle_threshold2 = rad2ang(atan(y / x)) + 180 - rough_angle;
//	dt35.right_angle_threshold1 = dt35.behind_angle_threshold2 + 2 * rough_angle;
//	dt35.right_angle_threshold2 = rad2ang(atan((longth - x) / y)) + 270 - rough_angle;
//	dt35.front_angle_threshold1 = dt35.right_angle_threshold2 + 2 * rough_angle;
//	dt35.front_angle_threshold2 = rad2ang(atan((wideth - y) / (longth - x))) - 5;
//	//计算四个dt35分别在场地的哪个边界
//	dt35.fdt35.site_base = Judge_In_Range(dt35.fdt35.angle);
//	dt35.ldt35.site_base = Judge_In_Range(dt35.ldt35.angle);
//	dt35.bdt35.site_base = Judge_In_Range(dt35.bdt35.angle);
//	dt35.rdt35.site_base = Judge_In_Range(dt35.rdt35.angle);
//	//计算dt35和它的边界所对应的角度    
//	dt35.fdt35.its_base_angle = dt35.fdt35.angle - dt35.fdt35.site_base*90;
//	dt35.ldt35.its_base_angle = dt35.ldt35.angle - dt35.ldt35.site_base*90;
//	dt35.bdt35.its_base_angle = dt35.bdt35.angle - dt35.bdt35.site_base*90;
//	dt35.rdt35.its_base_angle = dt35.rdt35.angle - dt35.rdt35.site_base*90;
	
	
//	dt35.fdt35.its_base_dis = dt35.fdt35.row_dis*cos(ang2rad(dt35.fdt35.its_base_angle));
//	dt35.ldt35.its_base_dis = dt35.ldt35.row_dis*cos(ang2rad(dt35.ldt35.its_base_angle));
//	dt35.bdt35.its_base_dis = dt35.bdt35.row_dis*cos(ang2rad(dt35.bdt35.its_base_angle));
	//dt35.rdt35.its_base_dis = dt35.rdt35.row_dis*cos(ang2rad(dt35.rdt35.its_base_angle));
	RowDis_To_CenterDis(&dt35.rdt35);
	RowDis_To_CenterDis(&dt35.bdt35);
	RowDis_To_CenterDis(&dt35.ldt35);
	RowDis_To_CenterDis(&dt35.fdt35);
//	//判断数据是否有效
//	if((dt35.fdt35.row_dis < 1) ||  (dt35.fdt35.row_dis > 6550))
//		dt35.fdt35.site_base = incredible;
//	if((dt35.ldt35.row_dis < 1) ||  (dt35.ldt35.row_dis > 6550))
//		dt35.ldt35.site_base = incredible;
//	if((dt35.bdt35.row_dis < 1) ||  (dt35.bdt35.row_dis > 6550))
//		dt35.bdt35.site_base = incredible;
//	if((dt35.rdt35.row_dis < 1) ||  (dt35.rdt35.row_dis > 6550))
//		dt35.rdt35.site_base = incredible;
//	//its_base_dis 是当前选择的边界距离车体中心的距离
//	if(dt35.fdt35.site_base != incredible)
//		RowDis_To_CenterDis(&dt35.fdt35);
//	if(dt35.ldt35.site_base != incredible)
//		RowDis_To_CenterDis(&dt35.ldt35);
//	if(dt35.bdt35.site_base != incredible)
//		RowDis_To_CenterDis(&dt35.bdt35);
//	if(dt35.rdt35.site_base != incredible)
//    	RowDis_To_CenterDis(&dt35.rdt35);
//	//计算每一个dt35距离当前场地边界的距离
//	Each_DT35_Site_Cla(&dt35.fdt35);
//	Each_DT35_Site_Cla(&dt35.rdt35);
//	Each_DT35_Site_Cla(&dt35.bdt35);
//	Each_DT35_Site_Cla(&dt35.ldt35);
//	//得到了每一个dt35所对应的坐标系 现在要对四个数据进行融合 
//	//1.根据每个dt35的距离计算其可信度以及其波动范围
//	Each_DT35_Wave(&dt35.fdt35);
//	Each_DT35_Wave(&dt35.rdt35);
//	Each_DT35_Wave(&dt35.bdt35);
//	Each_DT35_Wave(&dt35.ldt35);
//	//判断有几个dt35含有xflag
//	The_Number_Of_DT35_Which_Has_Data_Flag();
//	
//	DT35_Data_Fuse(x,y);
/////////////////////////////////////////简易判断///////////////////////////////////
	if((-dt35.rdt35.its_base_dis + dt35.ldt35.its_base_dis < 8350) && (-dt35.rdt35.its_base_dis + dt35.ldt35.its_base_dis > 7650) && (fabs(site.gyro_pos.with_odo_vy) < 0.8)){
		dt35.ycredible_flag = 1;
		dt35.y = -(dt35.ldt35.its_base_dis*0.5 + 0.5*(8000 + dt35.rdt35.its_base_dis));
	}
	else{
		dt35.ycredible_flag = 0;
		dt35.y = 0;
	}
	if((site.enc_pos.row_x > 12000) && (dt35.bdt35.its_base_dis < 4000) && (fabs(site.gyro_pos.with_odo_vx) < 0.8)){
		dt35.xcredible_flag = 1;
		dt35.x = 15000 - dt35.fdt35.its_base_dis;
	}
	else {
		dt35.xcredible_flag = 0;
		dt35.x = 0;
	}
}


/////////////////////////////////////////////////////DT35 发送测试程序 //////////////////////////////////////////////////////
char DT35_Send_Flag;
void DT35_Send_Pycharm(void){
	if(DT35_Send_Flag == 1){
		for(int i = 0;i < 10; i++){
			Send_DT35_Data();
			osDelay(2);
		}
		DT35_Send_Flag = 0;
	}
}
void Send_DT35_Data(void){
	if((dt35.fdt35.row_dis != 0) && (dt35.fdt35.row_dis <= 63000)){
		Send_Put_Data(0,ang2rad(site.gyro_pos.r));
		Send_Put_Data(1,dt35.fdt35.row_dis);
		Send_Put_Data(2,ladar.y);
		Send_Put_Data(3,15000 - ladar.x);
		Send_Float_Data(4);
	}
}


