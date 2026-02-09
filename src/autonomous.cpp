#include "autonomous.h"
#include "my-timer.h"
#include "robot_config.h"
#include "vex.h"
#include <iostream>

using namespace std;
using namespace vex;

double getDis() {
    return (Motor_LF.position(rotationUnits::deg) +
            Motor_LB.position(rotationUnits::deg) +
            Motor_RF.position(rotationUnits::deg) +
            Motor_RB.position(rotationUnits::deg)) /
           360 * 3.25 * 25.4 * 3.141 / 2 / 4.0;
}

void autonomous_1() {}

void autonomous_2() {}

void autonomous() {
    cout << "Entering autonomous" << endl;
    robot_operation_mode = RobotOperationMode::AUTO;
    MyTimer auton_timer;

    switch (auton_type) {
        case Autotype::AUTONOMOUS_1:
            cout << "Starting AUTONOMOUS_1" << endl;
            autonomous_1();
            break;
        case Autotype::AUTONOMOUS_2:
            cout << "Starting AUTONOMOUS_2" << endl;
            autonomous_2();
            break;
    }

    Controller.Screen.setCursor(3, 13);
    Controller.Screen.print("Auton done time: %.2f s", auton_timer.getTimeDouble());
    cout << "Autonomous completed in " << auton_timer.getTimeDouble() << " seconds."
         << endl;

    // 路线结束后不要退出自动函数
    while (true) {
        this_thread::sleep_for(10);
    }
}