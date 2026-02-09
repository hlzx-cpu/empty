#include "cmath"
#include "vex.h"
#include <iostream>

extern float PI;
extern float WheelDiameter; // 轮子直径，厘米
extern double IMU_coefficient;

double degNormalize(double deg);
int sign(double x);
int sign(float x);
double rad2deg(double rad);
double deg2rad(double deg);