#ifndef MOVE_H
#define MOVE_H

#include "main.h"
#include <cmath>

MovementResult calculateMovement(float targetX, float targetY);
bool goTo(MovementResult mov);
bool rotate(float angle_need);
bool updateVmax(int new_Vmax);
#endif
