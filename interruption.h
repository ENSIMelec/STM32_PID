#include "../main.h"
#include "Encoder/FastInterruptEncoder.h"
#include "PID/PID.h"

extern Encoder encGauche;
extern Encoder encDroit;
extern PID PID_vitesse_G;
extern PID PID_vitesse_D;
extern PID PID_angle;
extern PID PID_distance;

void Update_IT_callback(void);