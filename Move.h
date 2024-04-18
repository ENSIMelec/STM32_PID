#ifndef MOVE_H
#define MOVE_H

#include "main.h"
#include <cmath>

MovementResult calculateMovement(float targetX, float targetY);
bool goTo(MovementResult mov, float speed);
bool rotate(float angle_need, float speed);
bool updateVmax(int new_Vmax);
#endif
