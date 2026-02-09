#include "basic_function.h"
#include "calculate.h"

using namespace vex;
using namespace std;

////////////////
// 运动部分
//@brief 归一化处理得到IMU朝向
double IMUHeading() {
    double heading_raw = Inertial.heading() * IMU_coefficient;
    double heading = degNormalize(heading_raw);
    return heading;
}

//@brief resetIMU角度
void resetIMU() { Inertial.resetRotation(); }

// 行动的传入的参数部分：
// k_p/k_i/k_d
// tar_angle/tar_distance
// timeout
// speed_limit

//@brief 具体距离获取
double getDis() {
    return (Motor_LF.position(rotationUnits::deg) +
            Motor_LB.position(rotationUnits::deg) +
            Motor_RF.position(rotationUnits::deg) +
            Motor_RB.position(rotationUnits::deg)) /
           360 * 3.25 * 25.4 * 3.141 / 2 / 4.0;
}

// 转弯
//@brief 转向目标角度
void turnTo(double tar_angle) {}

// 直行
//@brief 直行到目标距离
void moveTo(double target) {}

///////////////
// 球路部分
//@brief 控制Drum转动

//@brief 控制Intake转动
void moveIntake(float pct) {
    if (pct)
        Motor_Intake.spin(fwd, pct, percentUnits::pct);
    else {
        Motor_Intake.stop(coast); // 目的：自然衰减电机
    }
}

//@brief 控制Shooter转动
void moveShooter(float pct) {
    if (pct)
        Motor_Shooter.spin(fwd, pct, percentUnits::pct);
    else {
        Motor_Shooter.stop(coast); // 目的：自然衰减电机
    }
}

static int motor_event = false;

void setMotorEvent(int _event) { motor_event = _event; }

//@brief 自动模式下电机控制
/*
@param _event 0:所有停止；1:吸球存球；2:高桥吐球；3:中桥吐球；4:下桥吐球
*/
void autonMotor() {
    while (true) {
        switch (motor_event) {
            case 0: // 所有停止
                moveIntake(0);
                moveShooter(0);
                break;
            case 1: // 吸球存球
                moveIntake(80);
                moveShooter(0);
                break;
            case 2: // 高桥吐球
                moveIntake(100);
                moveShooter(100);
                break;
            case 3: // 中桥吐球
                moveIntake(80);
                moveShooter(-80);
                break;
            case 4: // 下桥吐球
                moveIntake(-50);
                moveShooter(60);
                break;
        }
        this_thread::sleep_for(10);
    }
}