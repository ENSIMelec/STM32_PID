#ifndef MOVE_H
#define MOVE_H

#include "main.h"
#include <cmath>
#include "PID.h"

MovementResult calculateMovement(float targetX, float targetY);
MovementResult calculate_rotation(float angle_need);

bool goTo(MovementResult mov, float speed);
bool rotate(MovementResult mov, float speed);
bool updateVmax(int new_Vmax);

extern PID PID_vitesse_G;
extern PID PID_vitesse_D;
extern PID PID_angle;
extern PID PID_distance;
#endif
