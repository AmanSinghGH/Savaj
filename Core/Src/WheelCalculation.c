#include "WheelCalculation.h"
#include <math.h>

float minaxis = 3.4;
float majaxis = 3.9;
float theta = 0.0;

float mapValue(float value, float in_min, float in_max, float out_min, float out_max) {
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void calculateWheelSpeeds(int x, int y, int *m1, int *m2, int *m3, float *r1, float *degree) {
    float x1 = mapValue(x, 2000, 1000, -4, 4);
    float y1 = mapValue(y, 2000, 1000, -4, 4);
    theta = (*degree * 3.1415) / 180.0;

    float wheel1, wheel2, wheel3;

    float ellipse_check =
        (x1 * cos(theta) - y1 * sin(theta)) * (x1 * cos(theta) - y1 * sin(theta)) / (minaxis * minaxis) +
        (x1 * sin(theta) + y1 * cos(theta)) * (x1 * sin(theta) + y1 * cos(theta)) / (majaxis * majaxis);

    if (-0.02f < ellipse_check && ellipse_check < 0.02f) {
        wheel1 = wheel2 = wheel3 = 0;
    } else {
        if (!((-3.4 < x1 && x1 < 3.4) && (-3.4 < y1 && y1 < 3.4))) {
            float scale = 1.0 / ellipse_check;
            x1 *= sqrt(scale);
            y1 *= sqrt(scale);
        }
    }

    wheel1 = (((-6.5789 * cos(theta)) - (11.3951 * sin(theta))) * x1) +
             (((-6.5789 * sin(theta)) + (11.3951 * cos(theta))) * y1) + (4.1092 * *r1);

    wheel2 = (((11.39651 * sin(theta)) - (6.5789 * cos(theta))) * x1) +
             (((-6.5789 * sin(theta)) - (11.3951 * cos(theta))) * y1) + (4.1092 * *r1);

    wheel3 = (13.1579 * cos(theta) * x1) + (13.1579 * sin(theta) * y1) + (4.1092 * *r1);

    *m1 = (int)mapValue(wheel1, -49, 49, -1000, 1000);
    *m2 = (int)mapValue(wheel2, -49, 49, -1000, 1000);
    *m3 = (int)mapValue(wheel3, -49, 49, -1000, 1000);
}
