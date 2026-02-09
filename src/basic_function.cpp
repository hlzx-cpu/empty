#include "basic_function.h"
#include "calculate.h"

using namespace vex;

void moveTestMotor(double percent) { Motor_LF.spin(fwd, percent, percentUnits::pct); }

double IMUHeading() {
    double heading_raw = Inertial.heading()* IMU_coefficient;
    double heading = degNormalize(heading_raw);
    return heading;
}

void resetIMU() { Inertial.resetRotation(); }

// 行动的传入的参数部分：
// k_p/k_i/k_d
// tar_angle/tar_distance
// timeout
// speed_limit

// 转弯
void turnTo(double tar_angle) {}

// 直行
void moveTo(double target) {}