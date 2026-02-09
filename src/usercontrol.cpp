#include "usercontrol.h"
// 新增
#include "autonomous.h"
#include <iostream>
using namespace std;
using namespace vex;


// 新增
// 测试函数，按键可以替换，测试完成可将函数注释，不额外占用按键
void TurnTo() {
    if (press_A) {
        press_A = false;
        turnTo(90);
        cout << "TurnTo 90 degrees executed." << endl;
    }
}



void autonomous_1() {
    if (press_X) {
        press_X = false;
        autonomous_1();
    }
}

void autonChooser() {
    if (press_UP) {
        press_UP = false; // 修改按键实现切换不同的自动程序
        int nextVal = (static_cast<int>(auton_type) + 1) % 2;
        auton_type = static_cast<Autotype>(nextVal);
        Controller.Screen.setCursor(5, 1);
        Controller.Screen.print("Auton %s       ", auton_type_name[nextVal].c_str());
    }
}

void allDeviceControl() {
    // 在这里添加对所有设备的控制代码
    // 例如，控制电机、传感器等
    motorTestControl();

    TurnTo();
    autonomous_1();
}

void userControl() {
    // 新增
    // 用户控制代码
    while (true) {
        if (auto_func_reg == COMPETITION) {
            cout << "User Control" << endl;
            robot_operation_mode = RobotOperationMode::USER_CONTROL;
        }
        allDeviceControl();
        autonChooser();
        this_thread::sleep_for(2); // 避免占用过多CPU资源
    }
}