#pragma once
#include "main.h"
#include <stdio.h>
#include "recDecode.h"
#define INFINITE_LOOP_START for(;;){osDelay(1);
#define INFINITE_LOOP_END }

// #define USE_UART
// // #define USE_UDP
#define USE_NRF


#ifdef USE_UART
#include "usart.h"
#include "mySerial.h"
#define TX_UART huart1
extern myUartStruct commuUart;
#endif

#ifdef USE_UDP
#include "myUdp.h"
#define DES_IP_DEF 	{192,168,1,11}
#define DES_PORT	11
#define MY_PORT		12
#endif

#ifdef USE_NRF
#include "Nrf.h"
extern Nrf_t nrf;
#define NRF_INIT (Nrf_t) {                                                \
    .hspi = &hspi4,                                                     \
    .ce  = {.port = GPIOE, .pin = GPIO_PIN_15},                          \
    .nss = {.port = GPIOE, .pin = GPIO_PIN_11},                         \
    .irq = {.port = GPIOB, .pin = GPIO_PIN_10},                         \
    .rst = {.port = GPIOE, .pin = GPIO_PIN_8},                          \
    .address_transmit = {0x03, 0xAA, 0xBB, 0xCC, 0xDD},                      \
    .address_receive =  {                                               \
        .high_addr = {0xA, 0xB, 0xC, 0xD},                              \
        .p1 = 0x0, .p2 = 0x1, .p3 = 0x2, .p4 = 0x3, .p5 = 0x4           \
    },                                                                  \
    .rf_channel = 5,                                                    \
    .nrf_rx_callback = NrfCommu_EXTI_Callback,                          \
}
#endif

struct transmit_package_struct
{
	uint8_t crc;						//0 crc校验码
	uint8_t lenth;						//1 总长度-2
	//以下部分使用crc校验
	uint8_t rec_cnt;				//2 接收包的编号
										//3+
	float debug_data[5];				// 任意回传数据
	struct{
		uint8_t GamepadlostConnection : 1;
		uint8_t RepellostConnection : 1;

		uint8_t HolostConnection_lf : 1;
		uint8_t HolostConnection_rf : 1;
		uint8_t HolostConnection_rb : 1;
		uint8_t HolostConnection_lb : 1;

		uint8_t U8lostConnect_lf : 1;
		uint8_t U8lostConnect_rf : 1;
		uint8_t U8lostConnect_rb : 1;
		uint8_t U8lostConnect_lb : 1;

		uint8_t DT35lostConnection_f : 1;
		uint8_t DT35lostConnection_r : 1;
		uint8_t DT35lostConnection_b : 1;
		uint8_t DT35lostConnection_l : 1;

		uint8_t GyrolostConnect : 1;
		uint8_t WheelodometerlostConnect : 1;
		uint8_t LidarlostConnection : 1;
		uint8_t Master2lostConnection : 1;
	}Chassis_err;
	struct{
		uint8_t GO_offline : 1;
		uint8_t ELE_L_offline : 1;
		uint8_t ELE_R_offline : 1;
		uint8_t GO_locked : 1;
		uint8_t ELE_L_locked : 1;
		uint8_t ELE_R_locked : 1;
		uint8_t Go_hot : 1;
		uint8_t COMMU_err : 1;
	}Dribble_err;
	struct{
		uint8_t VESC : 1;
		uint8_t HighTorque : 1;
		uint8_t pos_lidar : 1;
		uint8_t basket_yaw:1;
		uint8_t pos_chassis : 1;
		uint8_t R2_pos : 1;
	}Shoot_err;
} __attribute__((packed));	// 对齐设置,强制不进行补位操作
extern struct transmit_package_struct debugData_pkg;

extern uint8_t GamepadLostConnection;

void NrfCommu_EXTI_Callback(uint8_t channel, uint8_t *data, uint8_t len);
void Commu_init(void);

