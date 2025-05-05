#include "catchball.h"
#include "cmsis_os.h"
#include "math.h" 

// --- Global Variables and Parameters ---

HighTorque_t HighTorque[HIGHTORQUE_NUM] = {0}; 
CatchStateFlag catch_sf = {0}; 

// Target Positions
float Init_Pos = 35;
float CatchBall_Pos = 18;
float Defend_Pos = -75;
float PreDunk_Pos = 8;

// Control Parameters for Position HOLDING (Mapped to Overall_States enum indices)
// Indices:          {Initialize, CatchingBall, Defend, PreDunk, BackToFold}
float Kp_Hold[5] =   {0.023,      0.2,          0.28,   0.2,     0.023     }; 
float Kd_Hold[5] =   {0,          0.02,         0.02,   0.06,    0         };      
float Trq_Hold[5]=   {0,          0,            0,      -0.5,    0         };           
float Pos_Target[5]= {35,         18,           -75,    8,       35        };         


// Control Parameters for Velocity MOVEMENT (Based on original SpdUp/SpdDown)
float Spd_Move_Up = -80; 
float Spd_Move_Down = 30; 
float Kd_Move_Up = 3;    
float Kd_Move_Down = 0.2; 
float Trq_Move_Up = 0;  
float Trq_Move_Down = -0.8; 


// Current Control Commands to Motor Driver (Updated by Overall_Control, Used by Single_Control)
float State_Kp = 0;
float State_Kd = 0;
float State_Trq = 0; 
float State_Spd = 0;
float State_Pos = 0; 

const float POS_THRESHOLD_MOVE_DONE = 5.0; 
const float POS_THRESHOLD_PREDUNK = 10.0;

const float POS_THRESHOLD_HOLD_SETTLED = 5.0; 
const float SPD_THRESHOLD_HOLD_SETTLED = 10.0;

static uint32_t catching_entry_time = 0;
const uint32_t CATCH_TIMEOUT_MS = 2000; 


bool IsAtTargetPositionSettled(float target_pos)
{
    if (HIGHTORQUE_NUM == 0) return false; 

    float pos_err = HighTorque[0].fdbk.pos - target_pos;
    float current_spd = HighTorque[0].fdbk.spd; 

		if(catch_sf.stateflag.Overall_State != PreDunk)
		{
			return fabsf(pos_err) < POS_THRESHOLD_HOLD_SETTLED && fabsf(current_spd) < SPD_THRESHOLD_HOLD_SETTLED;
		}
		else
		{
			return fabsf(pos_err) < POS_THRESHOLD_PREDUNK && fabsf(current_spd) < SPD_THRESHOLD_HOLD_SETTLED;
		}
}

bool IsNearTargetPosition(float target_pos)
{
    if (HIGHTORQUE_NUM == 0) return false;

    float pos_err = HighTorque[0].fdbk.pos - target_pos;
    return fabsf(pos_err) < POS_THRESHOLD_MOVE_DONE;
}

int Read_PG()
{
	uint8_t PG_Counter = 0;
    for(int i = 0; i < 10; i++)
	{
		if(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_11) == GPIO_PIN_SET) PG_Counter++;
		else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_SET) PG_Counter++;
		else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_SET) PG_Counter++;
	}

	catch_sf.stateflag.PG_State = (PG_Counter >= 7) ? 1 : 0;
	return catch_sf.stateflag.PG_State;
}


// --- Core Control Logic ---

void Single_Control()
{
    if (HIGHTORQUE_NUM == 0) return;

    HighTorque[0].ctrl = (HighTorque_ctrl_t){
        .pos = State_Pos,
        .spd = State_Spd,
        .trq = State_Trq,
        .Kp = State_Kp,
        .Kd = State_Kd
    };
}


void Overall_Control()
{
    static uint8_t init_step = 0; // For Initialize state sequence
    static uint32_t init_step_start_time = 0; // For tracking time within an init step
    const uint32_t INIT_SWEEP_UP_TIMEOUT = 1000; // ms timeout for sweeping up (example)
    const uint32_t INIT_SWEEP_DOWN_TIMEOUT = 3000; // ms timeout for sweeping down (example)
    const float INIT_SWEEP_UP_POS_CHECK = Init_Pos + 10; // Position check during sweep up (example)


    switch(catch_sf.stateflag.Overall_State)
    {
        case Initialize: { 
            if (catch_sf.stateflag.IfNeedCheck) {
                if (init_step == 0) { 
                    State_Kp = 0; State_Kd = Kd_Move_Up; State_Spd = Spd_Move_Up; State_Pos = 0; State_Trq = Trq_Move_Up;
                    init_step_start_time = HAL_GetTick(); 
                    init_step = 1;
                } else if (init_step == 1) { 
                     State_Kp = 0; State_Kd = Kd_Move_Up; State_Spd = Spd_Move_Up; State_Pos = 0; State_Trq = Trq_Move_Up;
                    if (fabsf(HighTorque[0].fdbk.spd) < SPD_THRESHOLD_HOLD_SETTLED && HighTorque[0].fdbk.pos > INIT_SWEEP_UP_POS_CHECK) { // Check speed is low and moved up significantly
                        init_step = 2; 
                        init_step_start_time = HAL_GetTick(); 
                    } else if (HAL_GetTick() - init_step_start_time > INIT_SWEEP_UP_TIMEOUT) {
                        catch_sf.stateflag.IfNeedCheck = 0;
                        init_step = 0; 
                    }
                } else if (init_step == 2) { 
                     State_Kp = 0; State_Kd = Kd_Move_Down; State_Spd = Spd_Move_Down; State_Pos = 0; State_Trq = Trq_Move_Down;
                    if (HighTorque[0].fdbk.pos < (Init_Pos - 8)) { 
                        init_step = 3; 
                        // init_step_start_time = HAL_GetTick(); // Not strictly needed for the last step
                    } else if (HAL_GetTick() - init_step_start_time > INIT_SWEEP_DOWN_TIMEOUT) {
                        catch_sf.stateflag.IfNeedCheck = 0;
                        init_step = 0; 
                    }
                } else if (init_step == 3) {
                     State_Pos = Pos_Target[BackToFold]; 
                     State_Kp = Kp_Hold[BackToFold]; 
                     State_Kd = Kd_Hold[BackToFold]; 
                     State_Trq = Trq_Hold[BackToFold];
                     State_Spd = 0; 

                    catch_sf.stateflag.IfNeedCheck = 0; 
                    init_step = 0; 
                }
            } else {
                State_Pos = Pos_Target[BackToFold]; 
                State_Kp = Kp_Hold[BackToFold];
                State_Kd = Kd_Hold[BackToFold];
                State_Trq = Trq_Hold[BackToFold];
                State_Spd = 0;
                init_step = 0; 
            }
            break; } 

        case CatchingBall: { 
            float target_pos = Pos_Target[CatchingBall];

            if (catch_sf.stateflag.InMoveToCatchingBall) {
                State_Kp = 0; // Velocity control

                // Determine movement direction and set parameters
                if (HighTorque[0].fdbk.pos < target_pos - POS_THRESHOLD_MOVE_DONE) {
                    State_Spd = Spd_Move_Down;
                    State_Kd = Kd_Move_Down;
                    State_Trq = Trq_Move_Down;
                } else if (HighTorque[0].fdbk.pos > target_pos + POS_THRESHOLD_MOVE_DONE) {
                    State_Spd = Spd_Move_Up;
                    State_Kd = Kd_Move_Up;
                    State_Trq = Trq_Move_Up;
                } else {
                    State_Spd = 0; // Stop velocity command
                    catch_sf.stateflag.InMoveToCatchingBall = 0;
                }

            } else {
                State_Pos = target_pos; 
                State_Kp = Kp_Hold[CatchingBall];
                State_Kd = Kd_Hold[CatchingBall];
                State_Trq = Trq_Hold[CatchingBall];
                State_Spd = 0;

                IsAtTargetPositionSettled(target_pos);
            }
            break; }

        case Defend: { 
            float target_pos = Pos_Target[Defend];

            if (catch_sf.stateflag.InMoveToDefend) {
                State_Kp = 0; // Velocity control

                // Determine movement direction and set parameters
                if (HighTorque[0].fdbk.pos < target_pos - POS_THRESHOLD_MOVE_DONE) {
                    State_Spd = Spd_Move_Down;
                    State_Kd = Kd_Move_Down;
                    State_Trq = Trq_Move_Down;
                } else if (HighTorque[0].fdbk.pos > target_pos + POS_THRESHOLD_MOVE_DONE) {
                    State_Spd = Spd_Move_Up;
                    State_Kd = Kd_Move_Up;
                    State_Trq = Trq_Move_Up;
                } else {
                    State_Spd = 0; // Stop velocity command
                    catch_sf.stateflag.InMoveToDefend = 0;
                }

            } else {
                State_Pos = target_pos;
                State_Kp = Kp_Hold[Defend];
                State_Kd = Kd_Hold[Defend];
                State_Trq = Trq_Hold[Defend];
                State_Spd = 0;

                IsAtTargetPositionSettled(target_pos);
            }
            break; }

        case PreDunk: { 
            float target_pos = Pos_Target[PreDunk]; 

            if (catch_sf.stateflag.InMoveToPreDunk) {
                 State_Kp = 0; // Velocity control

                // Determine movement direction and set parameters
                 if (HighTorque[0].fdbk.pos < target_pos - POS_THRESHOLD_PREDUNK) {
                    State_Spd = Spd_Move_Down;
                    State_Kd = Kd_Move_Down;
                    State_Trq = Trq_Move_Down;
                } else if (HighTorque[0].fdbk.pos > target_pos + POS_THRESHOLD_PREDUNK) {
                    State_Spd = Spd_Move_Up;
                    State_Kd = Kd_Move_Up;
                    State_Trq = Trq_Move_Up;
                } else {
                    State_Spd = 0; // Stop velocity command
                    catch_sf.stateflag.InMoveToPreDunk = 0; 
                }

            } else {
                State_Pos = target_pos;
                State_Kp = Kp_Hold[PreDunk];
                State_Kd = Kd_Hold[PreDunk];
                State_Trq = Trq_Hold[PreDunk];
                State_Spd = 0;

                IsAtTargetPositionSettled(target_pos);
            }
            break; }

        case BackToFold: {
            float target_pos = Pos_Target[BackToFold]; 

            if (catch_sf.stateflag.InMoveToBackToFold) {
                 State_Kp = 0; // Velocity control

                // Determine movement direction and set parameters
                 if (HighTorque[0].fdbk.pos < target_pos - POS_THRESHOLD_MOVE_DONE) {
                    State_Spd = Spd_Move_Down;
                    State_Kd = Kd_Move_Down;
                    State_Trq = Trq_Move_Down;
                } else if (HighTorque[0].fdbk.pos > target_pos + POS_THRESHOLD_MOVE_DONE) {
                    State_Spd = Spd_Move_Up;
                    State_Kd = Kd_Move_Up;
                    State_Trq = Trq_Move_Up;
                } else {
                    State_Spd = 0; // Stop velocity command
                    catch_sf.stateflag.InMoveToBackToFold = 0; 
                }

            } else {
                State_Pos = target_pos;
                State_Kp = Kp_Hold[BackToFold];
                State_Kd = Kd_Hold[BackToFold];
                State_Trq = Trq_Hold[BackToFold];
                State_Spd = 0; // Position control

                IsAtTargetPositionSettled(target_pos);
            }
            break; }
    }
}


void Loop_Judgement()
{
    Overall_States current_state = catch_sf.stateflag.Overall_State; // Read current state
    Overall_States next_state = current_state; // Assume stay in current state by default

    // Read external commands/sensors
    interact.get_ball = Read_PG();
    bool gamepad_pressed = interact.GamePad;
    bool catch_timer_elapsed = (current_state == CatchingBall) && (HAL_GetTick() - catching_entry_time > CATCH_TIMEOUT_MS); // Check timer only when in CatchingBall

    // --- State Transition Logic ---
    switch(interact.defend_status)
    {
        case initial:
            next_state = Initialize;
            break;

        case fold:
            next_state = BackToFold;
            break;

        case catch_ball:
            next_state = CatchingBall;
            break;

        case defend:
            next_state = Defend;
            break;

        case predunk:
//             if (current_state == CatchingBall && (catch_timer_elapsed || gamepad_pressed))
//             {
//                 next_state = PreDunk;
//             }
							next_state = PreDunk;
             break;

        default:
            break;
    }

    // --- Apply the determined next state and Handle State Entry ---
    if (next_state != current_state) {

         // Clear all 'InMoveTo' flags when leaving *any* state
         catch_sf.stateflag.InMoveToCatchingBall = 0;
         catch_sf.stateflag.InMoveToDefend = 0;
         catch_sf.stateflag.InMoveToPreDunk = 0;
         catch_sf.stateflag.InMoveToBackToFold = 0;

         // Apply the new state
        catch_sf.stateflag.Overall_State = next_state;

         // --- State Entry Actions ---
         switch(next_state) {
             case Initialize:
                // catch_sf.stateflag.IfNeedCheck = 1;
                break;
             case CatchingBall:
                catch_sf.stateflag.InMoveToCatchingBall = 1; 
                catching_entry_time = HAL_GetTick();
                break;
             case Defend:
                catch_sf.stateflag.InMoveToDefend = 1; 
                break;
             case PreDunk:
                catch_sf.stateflag.InMoveToPreDunk = 1; 
                break;
             case BackToFold:
                catch_sf.stateflag.InMoveToBackToFold = 1;
                break;
         }
    }

    // --- Clean up flags ---
    interact.GamePad = 0;
}

// --- RTOS Tasks ---

//void HTMotorControl(void const * argument)
//{
//    for(;;)
//    {
//        Single_Control();
//        HighTorque_SendPosParam_f(&hfdcan1, 6);

//        osDelay(4);
//    }
//}

//void ParamsChange(void const * argument)
//{
//    for(;;)
//    {
//        Loop_Judgement();
//        Overall_Control();

//        osDelay(10);
//    }
//}


