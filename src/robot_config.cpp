#include "robot_config.h"
#include "vex.h"

using namespace vex;
MyBrain *my_brain = MyBrain::getInstance();
controller Controller = controller(primary);
inertial Inertial = inertial(PORT1);

motor Motor_LF = motor(PORT1, ratio6_1, false);
motor Motor_RF = motor(PORT2, ratio6_1, false);
motor Motor_LB = motor(PORT3, ratio6_1, false);
motor Motor_RB = motor(PORT4, ratio6_1, false);

