#pragma once
#include "motor.h"


void motor_indent_register(float angle, float cw, float ccw, float power);
motor_indent_t *motor_indent_find(float angle);
