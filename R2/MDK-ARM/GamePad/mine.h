#ifndef __MINE_H
#define __MINE_H

#include "mngSerial.h"
#include "Chassis.h"

extern float Game_Pad_Rocker_Data[4];
extern unsigned char Game_Pad_Key_Data[22];
extern unsigned char Game_Pad_Switch_Data[10];

extern char debug;
extern char long_no_connect;
void Get_GamePad_Data(void);
void GamePad_Data_Cla(void);
void GamePad_Init(void);
char * Show_Control_Status(void);
char * Show_Car_Status(void);
void GamePad_Detect(char time);
#endif
