#ifndef __INTERACT_H
#define __INTERACT_H
#include "HighTorque.h"
#include "catchball.h"
#include "Interact.h"

enum Interact_Flag{
	initial = 0,
	fold = 1,
  catch_ball = 2,
  defend = 3,
	predunk = 4,
	CMD_ERROR_CLEAR = 5
};
struct Interact{
	enum Interact_Flag defend_status;
	char get_ball;
	char GamePad;
};
extern struct Interact interact;


#endif




