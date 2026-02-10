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
            Controller.Screen.print(
                "Auton %s       ", auton_type_name[static_cast<int>(auton_type)].c_str());
            first_time = false;
        }

        my_brain->Screen.setCursor(3, 1);
        my_brain->Screen.print("dis: %.2f    IMU: %.2f", getDis(), IMUHeading());
        my_brain->Screen.setCursor(4, 1);
        my_brain->Screen.print("raw IMU: %.2f", rawIMUHeading());

        Controller.Screen.setCursor(2, 1);
        Controller.Screen.print("dis: %.2f    IMU: %.2f", getDis(), IMUHeading());

        this_thread::sleep_for(200);
    }
}