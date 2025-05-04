#include "Communication.h"
#include "Can_Bsp.h"
#include "Global.h"
#include "string.h"
#include "Interact.h"
#define Com_Can hfdcan1

#define poledown_id 0xCA
#define dribble_id 0xCB
#define lift_id 0xCC
#define jump_id 0xCD
unsigned char Send[8];

struct R1_Data r1;



void Tell_Yao_Xuan(char *message){
	  ////////////防守指令//////////////////
    if(strcmp(message, "fold") == 0)
		   interact.defend_status = fold;
		else if(strcmp(message, "catch") == 0)
       interact.defend_status = catch_ball;
    else if(strcmp(message, "defend") == 0) 
       interact.defend_status = defend;
		else if(strcmp(message, "predunk") == 0)
       interact.defend_status = predunk;
		
		////////////////////
		else if(strcmp(message, "down") == 0) 
      FDCAN_Send(&Com_Can,poledown_id,"STD",NULL,"FD",0,"OFF");
		else if(strcmp(message, "dribble") == 0)
      FDCAN_Send(&Com_Can,dribble_id,"STD",NULL,"FD",0,"OFF");
    else if(strcmp(message, "lift") == 0) {
      FDCAN_Send(&Com_Can,lift_id,"STD",NULL,"FD",0,"OFF");
			interact.defend_status = (interact.defend_status == predunk)?interact.defend_status:predunk;
		}
		else if(strcmp(message, "jump") == 0) 
        FDCAN_Send(&Com_Can,jump_id,"STD",NULL,"FD",0,"OFF");
		strcpy(message,"nothing"); 
}

void Get_R1_Data(unsigned char * data){
	if(data[0] != 0xA5)
		return;
	r1.x = char2float(data + 1) * 1000;
	r1.y = char2float(data + 5) * 1000;
	r1.receive_ball_flag = data[9];
	r1.oppsite_angle = Limit(rad2ang(atan2f(site.now.y - 275 - r1.y ,site.now.x - r1.x  + 275)), -90,90);
}