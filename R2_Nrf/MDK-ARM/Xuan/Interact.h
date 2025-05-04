#ifndef __INTERACT_H
#define __INTERACT_H
#include "HighTorque.h"
#include "catchball.h"
#include "Interact.h"

enum Interact_Flag{
	initial = 0,
	fold,
  catch_ball,
  defend,
	predunk
};
struct Interact{
	enum Interact_Flag defend_status;
	char get_ball;
	char GamePad;
};
extern struct Interact interact;


#endif




