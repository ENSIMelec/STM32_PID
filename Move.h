#ifndef MOVE_H
#define MOVE_H

#include "main.h" 

#include <cmath>

struct MovementResult {
    float angle;
    float distance;
};
MovementResult calculateMovement(float targetX, float targetY);

#endif 
