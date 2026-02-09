#include "basic_function.h"

using namespace vex;

void moveTestMotor(double percent) {
    Motor_LF.spin(fwd, percent, percentUnits::pct);
}

// 新增
void turnTo(double tar_angle) {

}

void moveTo(double target) {

}