#ifndef VEX_H_
#define VEX_H_
#include "v5.h"
#include "v5_vcs.h"
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

using namespace std;
using namespace vex;
// 新增

/// @brief Robot Operation Mode
enum RobotOperationMode { AUTO, USER_CONTROL };

enum AutoFuncMode { COMPETITION, EMPTY };

extern RobotOperationMode robot_operation_mode;
extern AutoFuncMode auto_func_reg;

enum Autotype { AUTONOMOUS_1, AUTONOMOUS_2 };
extern Autotype auton_type;
extern std::string auton_type_name[2]; // 修改自动路线的数量

#endif