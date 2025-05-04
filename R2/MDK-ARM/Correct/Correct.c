#include "Correct.h"
#include "string.h"
#include "fdcan.h"

char wrong_code[10];
char * can_wrong_code = "CanTxFull";

void Can_Detect(void){
	if(FDCAN1 ->TXFQS & 1 << 21){
	 strcpy(wrong_code, can_wrong_code);
	 MX_FDCAN1_Init();
	}
	if(FDCAN2 ->TXFQS & 1 << 21){
	 strcpy(wrong_code, can_wrong_code);
	 MX_FDCAN2_Init();
	}
	if(FDCAN3 ->TXFQS & 1 << 21){
	 strcpy(wrong_code, can_wrong_code);
	 MX_FDCAN3_Init();
	}
}




               







