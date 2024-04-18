#include "main.h"

bool update_Position(float distance, float angle);

bool reset_Position(void);

bool reset_last_distance(void);

bool reset_angle(float angle_);

bool calculate_distance_time(float distance_, float Vmax_);

bool calculate_angle_time(float angle_, float Vmax_);

float distance_command_ramp(float interrupt_tick);

float angle_command_ramp(float interrupt_tick);
