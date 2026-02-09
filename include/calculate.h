#include "vex.h"
#include <iostream>
#include "cmath"

extern float PI;
extern float WheelDiameter; // 轮子直径，厘米
extern double LF_coeffcient; // 左前轮修正系数
extern double RF_coeffcient; // 右前轮修正系数
extern double LB_coeffcient; // 左后轮修正系数
extern double RB_coeffcient; // 右后轮修正系数

double degNormalize(double deg);
int sign(double x);
int sign(float x);
double rad2deg(double rad);
double deg2rad(double deg);