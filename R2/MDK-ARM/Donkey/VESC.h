#ifndef __VESC_H__
#define __VESC_H__

#include "stdint.h"
#include "main.h"
#include "stdbool.h"
#include "string.h"
#define VESC_START_VELOCITY 900


struct VESC {
	float front;
	float left;
	float rpm;
	int dir;
};
#define CLAMP(x, lower, upper) (x >= upper ? upper : (x <= lower ? lower : x))

#define ALL_VESC 255    
#define MAX_CURRENT 70  
#define MAX_MULTITURN 100000 
// #define FLOAT_TRANSMITTED
#define BUFFER_MAX_LENTH 8

#define DUTY_CYCLE_SCALE                    1e5
#define CURRENT_SCALE                       1e3
#define VOLTAGE_SCALE                       1e3
#define RPM_SCALE                           1e0   
#define POS_SCALE                           1e6
#define MULTITURN_POS_SCALE                 1e3
#define SUBAREA_PARAMETER_KP_SCALE          1e4 
#define SUBAREA_PARAMETER_DEADBAND_SCALE    1e2 

typedef enum {                              //VESC�ﶨ���CAN���ݰ�����
	CAN_PACKET_SET_DUTY						= 0,
	CAN_PACKET_SET_CURRENT					= 1,
	CAN_PACKET_SET_CURRENT_BRAKE			= 2,
	CAN_PACKET_SET_RPM						= 3,
	CAN_PACKET_SET_POS						= 4,
	CAN_PACKET_SET_POS_MULTITURN			= 74,  //2.15.2024����
	CAN_PACKET_SET_ACCEL_CURRENT			= 63,  
	CAN_PACKET_SET_TARGET_SPEED			    = 65,  
	CAN_PACKET_SET_BRAKE_CURRENT			= 68,
	CAN_PACKET_SET_CUSTOM_MODE				= 69,
	CAN_PACKET_SET_HOME			            = 75,
	CAN_PACKET_HOMING			            = 76,
	CAN_PACKET_STATUS                       = 9,
	CAN_PACKET_STATUS_2						= 14,
	CAN_PACKET_STATUS_3						= 15,
	CAN_PACKET_STATUS_4						= 16,
	CAN_PACKET_SHUTDOWN						= 31,
	CAN_PACKET_GET_SUBAREA_PARA1			= 77,  //2.18.2024���� ����PID������ȡ������
	CAN_PACKET_GET_SUBAREA_PARA2			= 78,
	CAN_PACKET_GET_SUBAREA_PARA3			= 79,
	CAN_PACKET_SET_SUBAREA_PARA1			= 80,
	CAN_PACKET_SET_SUBAREA_PARA2			= 81,
	CAN_PACKET_SET_SUBAREA_PARA3			= 82,
	CAN_PACKET_STORE_MC_CONFIGURATION		= 83,
	CAN_PACKET_ENABLE_SUBAREA_PID	        = 84,
	CAN_PACKET_SELFLOCK	                    = 85,
	CAN_PACKET_SELFLOCK_RELEASE	            = 86,
	CAN_PACKET_RELEASE_MOTER                = 94,
	CAN_PACKET_SET_ZERO_POS                = 95,
} CAN_PACKET_ID;

typedef struct {                            // VESC�ش���Ϣ           
	float rpm;	   	  	                    // ת��
	float duty_cycle;                       // ռ�ձ�
	float pos;         	                    // λ��
	float mul_pos;                          // ��Ȧλ��
	float tot_current_in;                   // �ܵ���
	float tot_voltage;                      // �ܵ�ѹ
	float power;                            // �ܹ��� = V*I
} motor_info_t;

typedef struct {
	bool para1_received;
	bool para2_received;
	bool para3_received;
}subarea_PID_parameter_communication_t;

typedef struct {
	float subarea_1;
	float subarea_2;
	float kp1;
	float ki1;
	float kd1;
	float kd_proc1;
	float kp2;
	float ki2;
	float kd2;
	float kd_proc2;
	float deadband;
	subarea_PID_parameter_communication_t com_status;
}subarea_PID_parameter_t;


bool VESC_SetRPM(float value, uint8_t id);
bool VESC_SetMultiturnPos(float value, uint8_t id);
bool VESC_SetPos(float value, uint8_t id);

#endif
