#include "vex.h"

// 模式变量
RobotOperationMode robot_operation_mode = RobotOperationMode::USER_CONTROL;
AutoFuncMode auto_func_reg = AutoFuncMode::COMPETITION;
//选择具体的自动程序
Autotype auton_type = Autotype::AUTONOMOUS_1;//默认
std::string auton_type_name[2] = {"AUTONOMOUS_1", "AUTONOMOUS_2"};//修改自动路线的数量