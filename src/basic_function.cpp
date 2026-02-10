#include "basic_function.h"
#include "calculate.h"

using namespace vex;
using namespace std;

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

////////////////
// 运动部分
//@brief 获取原始IMU朝向（几千度）
double rawIMUHeading() { return Inertial.heading() * IMU_coefficient; }

//@brief 归一化处理得到IMU朝向
double IMUHeading() {
    double heading = degNormalize(rawIMUHeading());
    return heading;
}

//@brief resetIMU角度
void resetIMU() { Inertial.resetRotation(); }

//@brief 具体距离获取(cm)
// 计算过程：轮子的角度 -> 转过的圈数 -> 轮子周长 -> 机器人前进距离
double getDis() {
    resetMotor();
    return (Motor_LF.position(rotationUnits::deg) +
            Motor_LB.position(rotationUnits::deg) +
            Motor_RF.position(rotationUnits::deg) +
            Motor_RB.position(rotationUnits::deg)) /
           4 /*平均每个轮子*/ / 360 /*圈数*/ * WheelDiameter /*车轮直径*/ *
           2.54 /*英寸转化厘米*/ * PI;
}

//@brief 前进
void moveForwardVoltage(int voltage) {
    Motor_LF.spin(fwd, voltage, voltageUnits::mV);
    Motor_LB.spin(fwd, voltage, voltageUnits::mV);
    Motor_RF.spin(fwd, voltage, voltageUnits::mV);
    Motor_RB.spin(fwd, voltage, voltageUnits::mV);
}

//@brief 右转
void turnRightVoltage(int voltage) {
    Motor_LF.spin(fwd, voltage, voltageUnits::mV);
    Motor_LB.spin(fwd, voltage, voltageUnits::mV);
    Motor_RF.spin(reverse, voltage, voltageUnits::mV);
    Motor_RB.spin(reverse, voltage, voltageUnits::mV);
}

//@brief 不动
void brakeChassic() {
    Motor_LF.stop(brake);
    Motor_LB.stop(brake);
    Motor_RF.stop(brake);
    Motor_RB.stop(brake);
}

//@brief reset电机
void resetMotor() {
    Motor_LF.resetPosition();
    Motor_LB.resetPosition();
    Motor_RF.resetPosition();
    Motor_RB.resetPosition();
}

// Declaration
class PID {
  public:
    PID();

    void setCoefficients(double _kp, double _ki, double _kd);
    void setTarget(double _target);
    void setILimits(double _i_range, double _i_max);
    void setOutputLimits(double _maxPercentage);
    void setSettling(double _errorTol, double _timeout);

    void reset();
    double update(double input);
    bool isSettled();

  private:
    double kp, ki, kd;
    double target;
    double I_Range, I_Max, maxPercentage;

    double errorTol;
    double derivativeTol;
    double timeout;

    double errorPrev;
    double errorInt;
    bool firstRun;
    bool settled;

    timer PIDTimer;
};

// @brief: PID类实现
PID::PID()
    : firstRun(true), settled(false), kp(0), ki(0), kd(0), target(0), I_Range(0),
      I_Max(0), maxPercentage(100), errorTol(0), derivativeTol(0), timeout(0),
      errorPrev(0), errorInt(0) {
    PIDTimer.reset();
}

void PID::setCoefficients(double _kp, double _ki, double _kd) {
    kp = _kp;
    ki = _ki;
    kd = _kd;
}
void PID::setTarget(double _target) { target = _target; }
void PID::setILimits(double _i_range, double _i_max) {
    I_Range = _i_range;
    I_Max = _i_max;
}
void PID::setOutputLimits(double _maxPercentage) { this->maxPercentage = _maxPercentage; }
void PID::setSettling(double _errorTol, double _timeout) {
    errorTol = _errorTol;
    timeout = _timeout;
}

void PID::reset() {
    errorPrev = 0;
    errorInt = 0;
    firstRun = true;
    settled = false;
    PIDTimer.reset();
}

double PID::update(double input) {
    double errorCurt = target - input;

    double P = kp * errorCurt;
    // 防止积分初始饱和
    if (firstRun) {
        errorPrev = errorCurt;
        firstRun = false;
    }
    double errorDev = errorCurt - errorPrev;
    double D = kd * errorDev;
    errorPrev = errorCurt;

    if (fabs(errorCurt) < I_Range && fabs(errorCurt) > errorTol) {
        errorInt += errorCurt;
    } else {
        errorInt = 0;
    }

    double I = ki * errorInt;
    if (fabs(I) > I_Max) I = sign(I) * I_Max; // 积分限幅

    double output = P + I + D;

    double maxVolt = maxPercentage / 100.0 * 12700;
    if (fabs(output) > maxVolt) output = sign(output) * maxVolt;

    if (fabs(errorCurt) <= errorTol) {
        if (PIDTimer.time(msec) >= timeout) {
            settled = true;
        }
    } else {
        PIDTimer.reset();
        settled = false;
    }

    return output;
}
bool PID::isSettled() { return settled; }

/**
 * @brief 机器人直线移动到指定距离(cm)
 * @param targetDis 目标距离
 * @param maxPercentage 最大速度比例
 * @param timeout 时间阈值
 * @param _kp P系数
 * @param _ki I系数
 * @param _kd D系数
 * @param tol_dis 允许距离误差
 * @param tol_time 允许误差持续时间
 */
void moveTo(double targetDis, int timeout, double maxPercentage, int tol_dis,
            int tol_time, double _kp, double _ki, double _kd, double _i_range,
            double _i_max) {

    PID movePID;

    movePID.setCoefficients(_kp, _ki, _kd);
    movePID.setTarget(targetDis);
    movePID.setOutputLimits(maxPercentage);
    movePID.setSettling(tol_dis, tol_time);
    movePID.setILimits(_i_range, _i_max);
    movePID.reset();

    resetMotor();

    timer timeoutTimer;
    timeoutTimer.reset();

    while (!movePID.isSettled() && timeoutTimer.time(msec) < timeout) {
        double currentPos = getDis();

        double outputVolt = movePID.update(currentPos);
        moveForwardVoltage(outputVolt);

        this_thread::sleep_for(10);
    }

    brakeChassic();
}

/**
 * @brief 机器人原地转向到指定角度（deg&-180~180)
 * @param targetAngle 顺时针目标角度
 * @param timeout 时间阈值
 * @param maxPercentage 最大速度比例
 * @param tol_angle 允许角度误差
 * @param _kp P系数
 * @param _ki I系数
 * @param _kd D系数
 * @param tol_time 允许误差持续时间
 */
void turnTo(double targetAngle, int timeout, double maxPercentage, int tol_angle,
            int tol_time, double _kp, double _ki, double _kd, double _i_range,
            double _i_max) {

    PID turnPID;

    turnPID.setCoefficients(_kp, _ki, _kd);
    turnPID.setTarget(degNormalize(targetAngle));
    turnPID.setOutputLimits(maxPercentage);
    turnPID.setILimits(_i_range, _i_max);
    turnPID.setSettling(tol_angle, tol_time);
    turnPID.reset();

    timer turnTimer;
    turnTimer.reset();

    while (!turnPID.isSettled() && turnTimer.time(msec) < timeout) {
        double currentHeading = IMUHeading();

        double outputVolt = turnPID.update(currentHeading);
        turnRightVoltage(outputVolt);

        this_thread::sleep_for(10);
    }

    brakeChassic();
}