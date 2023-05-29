#pragma once


typedef struct motor_indent {
    float angle;    //Angle in degrees where indent is to be place
    float cw_range; //Range in degrees where snap starts in clockwise range
    float ccw_range;//Range in degrees where snap starts in counter clockwise range
    float force;    //How much snap force, [0, 1] range 1 is max power
    struct motor_indent *next;
    struct motor_indent *prev;
} motor_indent_t;

void motor_init(void);