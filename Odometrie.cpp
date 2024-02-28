#include "Odometrie.h"

bool update_Position(float distance, float angle)
{
    x += cos(angle) * distance;
    y += sin(angle) * distance;
    return true;
}

bool reset_Position(void)
{
    x = 0;
    y = 0;
    return true;
}
