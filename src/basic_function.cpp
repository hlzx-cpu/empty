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

// ============================================================================
// 2. PID 类声明 (PID Class Declaration)
// ============================================================================
class PID {
  public:
    PID();

    // 设置参数
    void setCoefficients(double k_p, double k_i, double k_d);
    void setTarget(double target);
    void setILimits(double i_range, double i_max);
    void setOutputLimits(double max_output); // VEX电压最大是12.0
    void setSettlingCriteria(double error_tol, double deriv_tol, double settle_time_ms);

    // 核心功能
    void reset();
    double update(double input); // 返回计算出的电压值
    bool isSettled();

  private:
    double K_p, K_i, K_d;
    double target;
    double IRange, IMax, maxOutput;

    // 稳态判定参数
    double errorTol;     // 允许误差
    double derivTol;     // 允许速度(微分)
    double settleTimeMs; // 需要保持稳定的时间

    // 运行时变量
    double errorPrev;
    double errorInt;
    bool firstRun;
    bool settled;

    timer settleTimer;
};

// ============================================================================
// 3. PID 类实现 (PID Class Implementation)
// ============================================================================

PID::PID()
    : firstRun(true), settled(false), K_p(0), K_i(0), K_d(0), target(0), IRange(0),
      IMax(0), maxOutput(12.0), // VEX最大电压12V
      errorTol(0), derivTol(0), settleTimeMs(0), errorPrev(0), errorInt(0) {
    settleTimer.reset();
}

void PID::setCoefficients(double k_p, double k_i, double k_d) {
    K_p = k_p;
    K_i = k_i;
    K_d = k_d;
}
void PID::setTarget(double t) { target = t; }
void PID::setILimits(double i_range, double i_max) {
    IRange = i_range;
    IMax = i_max;
}
void PID::setOutputLimits(double max_out) { maxOutput = max_out; }
void PID::setSettlingCriteria(double e_tol, double d_tol, double time_tol) {
    errorTol = e_tol;
    derivTol = d_tol;
    settleTimeMs = time_tol;
}

void PID::reset() {
    errorPrev = 0;
    errorInt = 0;
    firstRun = true;
    settled = false;
    settleTimer.reset();
}

double PID::update(double input) {
    double errorCurt = target - input; // 1. 计算误差

    double P = K_p * errorCurt; // 2. P项

    // 3. D项 (带启动保护)
    if (firstRun) {
        errorPrev = errorCurt;
        firstRun = false;
    }
    double errorDev = errorCurt - errorPrev;
    double D = K_d * errorDev;
    errorPrev = errorCurt; // 记录误差供下次使用

    // 4. I项 (积分分离与抗饱和)
    // 只有当误差在 IRange 内，且没有到达目标(errorTol)时才积分
    if (fabs(errorCurt) < IRange && fabs(errorCurt) > errorTol) {
        errorInt += errorCurt;
    } else {
        errorInt = 0;
    }

    double I = K_i * errorInt;
    if (fabs(I) > IMax) I = sign(I) * IMax; // 积分限幅

    // 5. 计算总输出
    double output = P + I + D;

    // 6. 输出限幅 (限制最大电压)
    if (fabs(output) > maxOutput) output = sign(output) * maxOutput;

    // 7. 稳态检测 (Settled Logic)
    // 如果位置误差很小 且 速度(微分)也很小
    if (fabs(errorCurt) <= errorTol && fabs(errorDev) <= derivTol) {
        if (settleTimer.time(msec) >= settleTimeMs) {
            settled = true;
        }
    } else {
        settleTimer.reset(); // 只要抖动，重置计时
        settled = false;
    }

    return output;
}

bool PID::isSettled() { return settled; }

// ============================================================================
// 4. 封装动作函数 (Wrapper Functions) - 这里实例化 PID
// ============================================================================

/**
 * 转向函数 (Turn To Angle)
 * @param targetAngle  目标角度 (度)
 * @param maxSpeedVolt 最大速度 (伏特 0-12)
 * @param timeoutMs    超时时间 (毫秒)
 */
void turn_to(double targetAngle, int timeoutMs, double maxSpeedVolt, double k_p,
             double k_i, double k_d, int tol_angle, int tol_time) {

    // 1. 实例化 PID
    PID turnPID;

    // 2. 配置参数 (需要根据你的机器人重量调试这些 Kp, Ki, Kd)
    // 经验值：Kp=0.2~0.6, Kd=0.1~1.0 (取决于惯性)
    turnPID.setCoefficients(k_p, k_i, k_d);

    turnPID.setTarget(targetAngle);
    turnPID.setOutputLimits(maxSpeedVolt);
    turnPID.setILimits(15.0, 3.0); // 误差15度内开始积分，积分上限3V

    // 3. 设置稳态条件: 误差<1度，速度<0.5，保持150ms
    turnPID.setSettlingCriteria(tol_angle, 0.5, tol_time);
    turnPID.reset();

    timer timeoutTimer;
    timeoutTimer.reset();

    // 4. 循环控制
    while (!turnPID.isSettled() && timeoutTimer.time(msec) < timeoutMs) {
        // A. 读取传感器 (Inertial)
        double currentHeading = IMUHeading();

        // B. 计算 PID 输出
        double outputVolt = turnPID.update(currentHeading);

        // C. 执行动作 (原地旋转：左轮正转，右轮反转)
        // 使用 voltageUnits::volt 非常重要，比 pct 更线性
        Motor_LB.spin(fwd, outputVolt, voltageUnits::volt);
        Motor_RB.spin(reverse, outputVolt, voltageUnits::volt); // 注意反向

        // D. 必须延时，防止 CPU 占用过高
        wait(10, msec);
    }

    // 5. 结束制动
    Motor_LB.stop(brake);
    Motor_RB.stop(brake);
}

/**
 * 直线移动函数 (Drive Distance)
 * @param targetDist   目标距离 (度 degrees, 或自行换算为 cm/inch)
 * @param maxSpeedVolt 最大速度 (伏特 0-12)
 * @param timeoutMs    超时时间 (毫秒)
 */
void drive_distance(double targetDist, double maxSpeedVolt, int timeoutMs, double k_p,
                    double k_i, double k_d, int tol_dist, int tol_time) {

    // 1. 实例化 PID
    PID drivePID;

    // 2. 配置参数 (直线参数和转向通常不同)
    // 直线通常 Kp 较小，Kd 较大防止刹车点头
    drivePID.setCoefficients(k_p, k_i, k_d);

    drivePID.setTarget(targetDist);
    drivePID.setOutputLimits(maxSpeedVolt);
    drivePID.setSettlingCriteria(tol_dist, 1.0, tol_time); // 误差5度以内，保持100ms
    drivePID.reset();

    // 重置电机编码器，从 0 开始走
    Motor_LB.resetPosition();
    Motor_RB.resetPosition();

    timer timeoutTimer;
    timeoutTimer.reset();

    while (!drivePID.isSettled() && timeoutTimer.time(msec) < timeoutMs) {
        // A. 读取传感器 (取两轮平均值)
        double currentPos =
            (Motor_LB.position(degrees) + Motor_RB.position(degrees)) / 2.0;

        // B. 计算
        double outputVolt = drivePID.update(currentPos);

        // C. 执行 (直线行驶：两轮同向)
        // 进阶提示：这里通常还需要叠加一个 "Heading PID" 来保持直线走，
        // 但为了简单，这里只写了纯距离 PID。
        Motor_LB.spin(fwd, outputVolt, voltageUnits::volt);
        Motor_RB.spin(fwd, outputVolt, voltageUnits::volt);

        wait(10, msec);
    }

    Motor_LB.stop(brake);
    Motor_RB.stop(brake);
}
