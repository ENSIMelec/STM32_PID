#ifndef MOVE_H
#define MOVE_H

#include "main.h"
#include <cmath>
#include "PID.h"

MovementResult calculateMovement(float targetX, float targetY);
MovementResult calculate_rotation(float angle_need);
MovementResult calculate_moveOf(float distance_);

bool goTo(MovementResult mov, float speed);
bool rotate(MovementResult mov, float speed);
bool updateVmax(int new_Vmax);
bool moveOf(MovementResult mov, float speed);
void change_PID_mode(short mode);

extern PID PID_vitesse_G;
extern PID PID_vitesse_D;
extern PID PID_angle;
extern PID PID_distance;
#endif
