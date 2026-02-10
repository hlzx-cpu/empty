#include "calculate.h"
#include <cmath>
using namespace std;

const double PI = M_PI;
const float WheelDiameter = 3.25; // 轮子直径英寸

//@brief IMU修正系数（一般用转10圈计算）
double IMU_coefficient = 1.0; 

//@brief 角度归一化到[-180,180)
double degNormalize(double deg) {
    deg = fmod(deg + 180, 360);
    if (deg <= 0) deg += 360;
    return deg - 180;
}

int sign(double x) { return (x > 0) - (x < 0); }

int sign(float x) { return (x > 0) - (x < 0); }

double rad2deg(double rad) { return rad / M_PI * 180.0; }

double deg2rad(double deg) { return deg * M_PI / 180.0; }