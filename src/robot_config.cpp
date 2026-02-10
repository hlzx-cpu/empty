#include "robot_config.h"
#include "vex.h"

using namespace vex;
MyBrain* my_brain = MyBrain::getInstance();
controller Controller = controller(primary);

// 传感器的部分 -> 测试出来俯仰角等三个维度的角度
inertial Inertial = inertial(PORT1);

// ratio部分是电机本身属性配置的，程序需要匹配即可齿轮比，不能随便写
// 红色：36:1/600RPM/低扭矩
// 绿色：18:1/200RPM/中扭矩
// 蓝色：6:1/100RPM/高扭矩
//  配置底盘电机的部分 -> 通过控制true和false来实现电机正反转
motor Motor_LF = motor(PORT2, ratio6_1, false);
motor Motor_RF = motor(PORT3, ratio6_1, false);
motor Motor_LB = motor(PORT4, ratio6_1, false);
motor Motor_RB = motor(PORT5, ratio6_1, false);

// 配置动力电机的部分 -> 通过控制true和false来实现电机正反转
motor Motor_Intake = motor(PORT7, ratio36_1, false);
motor Motor_Shooter = motor(PORT8, ratio36_1, false);