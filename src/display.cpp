#include "display.h"
#include "basic_function.h"
#include "vex.h"
#include <iostream>

using namespace vex;
using namespace std;

void usrctrlDisplay() {
    bool first_time = true;
    while (true) {
        if (first_time == true) {
            Controller.Screen.clearScreen();
            // Show selected auto program
            Controller.Screen.setCursor(5, 1);
            Controller.Screen.print("Auto %s       ",
                                    auton_type_name[static_cast<int>(auton_side)].c_str());
            first_time = false;
        }
        Point pos = Position::getInstance()->getPos();


        my_brain->Screen.setCursor(3, 1);
        my_brain->Screen.print("x: %.2f y: %.2f IMU: %.2f %.2f", pos._x, pos._y,
                               IMUHeading(), Position::getInstance()->sensorHeading);
        my_brain->Screen.setCursor(4, 1);

        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print(
            "%.2f,%.2f,%.2f           ", Position::getInstance()->getGlobalRelPos()._x,
            Position::getInstance()->getGlobalRelPos()._y, rawIMUHeading());

        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("%.2f %.2f %.2f %.2f", pos._x, pos._y, IMUHeading(),
                                Position::getInstance()->sensorHeading);

        this_thread::sleep_for(500);
    }
}