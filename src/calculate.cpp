#include "calculate.h"
#include <cmath>
using namespace std;

const double PI = M_PI;
const float WheelDiameter = 4.0; // 轮子直径，厘米
double LF_coeffcient = 1.0; // 左前轮修正系数
double RF_coeffcient = 1.0; // 右前轮修正系数
double LB_coeffcient = 1.0; // 左后轮修正系数
double RB_coeffcient = 1.0; // 右后轮修正系数

double degNormalize(double deg) {
    deg = fmod(deg + 180, 360);
    if (deg <= 0) deg += 360;
    return deg - 180;
}

int sign(double x) { return (x > 0) - (x < 0); }

int sign(float x) { return (x > 0) - (x < 0); }

double rad2deg(double rad) { return rad / M_PI * 180.0; }

double deg2rad(double deg) { return deg * M_PI / 180.0; }