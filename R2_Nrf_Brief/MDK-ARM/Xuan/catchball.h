// --- catchball.h contents (update your header file accordingly) ---

#ifndef CATCHBALL_H
#define CATCHBALL_H

#include "Interact.h"
#include "stdbool.h"
#include "HighTorque.h"


// flags
typedef struct
{
    uint8_t Overall_State;
    int PG_State;
    uint8_t IfNeedCheck; 

    // Flags indicating the state is currently in its velocity-controlled movement phase
    uint8_t InMoveToCatchingBall;
    uint8_t InMoveToDefend;
    uint8_t InMoveToPreDunk;
    uint8_t InMoveToBackToFold;
		uint8_t IsError;

    // Consider adding other status flags like AtTargetPositionSettled, Folded, etc.
    // uint8_t AtTargetPositionSettled;
    // uint8_t Folded;
} CatchState;

typedef struct
{
    uint8_t Catch;
    uint8_t Defend;
    uint8_t Follow;
    uint8_t Fold;
} TO;


typedef struct
{
    CatchState stateflag;
    TO to;
} CatchStateFlag;

// Overall state machine states
typedef enum
{
    Initialize = 0,      
    CatchingBall = 1,    
    Defend = 2,         
    PreDunk = 3,        
    BackToFold = 4,
		Error = 5
} Overall_States;

typedef struct
{
	float start_pos;
	float end_pos;
	uint32_t start_time_ms;
	uint32_t duration_ms;
	
}Trajectory;


// Function Prototypes
void Loop_Judgement(); 
void Single_Control(); 
int Read_PG();
void Overall_Control(); 

// Helper functions
bool IsAtTargetPositionSettled(float target_pos); 


#endif // CATCHBALL_H
