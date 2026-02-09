#include "robot_config.h"
#include "vex.h"

using namespace vex;
MyBrain* my_brain = MyBrain::getInstance();
controller Controller = controller(primary);

//传感器的部分 -> 测试出来俯仰角等三个维度的角度
inertial Inertial = inertial(PORT1);

// 配置底盘电机的部分 -> 通过控制true和false来实现电机正反转
motor Motor_LF = motor(PORT2, ratio6_1, false);
motor Motor_RF = motor(PORT3, ratio6_1, false);
motor Motor_LB = motor(PORT4, ratio6_1, false);
motor Motor_RB = motor(PORT5, ratio6_1, false);

// 配置动力电机的部分 -> 通过控制true和false来实现电机正反转
motor Motor_Drum = motor(PORT6, ratio36_1, false);
motor Motor_Intake = motor(PORT7, ratio36_1, false);
motor Motor_Shooter = motor(PORT8, ratio36_1, false);