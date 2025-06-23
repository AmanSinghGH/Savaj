#ifndef WHEEL_CALCULATION_H
#define WHEEL_CALCULATION_H

// Function to map joystick values
float mapValue(float value, float in_min, float in_max, float out_min, float out_max);

// Function to calculate wheel speeds and convert to PWM motor values (uses pointers in C)
void calculateWheelSpeeds(int x, int y, int *m1, int *m2, int *m3, float *r1,float *degree);

#endif

