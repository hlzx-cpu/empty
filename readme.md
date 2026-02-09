#代办
1. 重构校内赛turtle的代码
2. 完成基础版本的配置
3. 

# 培训的基础内容：（目前在搭建初期，可以先不考虑具体内容部分）
1. 在vex_clean这个项目里面的传统代码的含义，不同的程序该摆放在哪里，怎么进入不同的线程
2. 怎么配robot-config，以及怎么调试硬件参数（debug测试角度）
3. 如何让电机转起来，怎么写基本的移动代码（主要面向直轮底盘，怎么旋转和怎么直行）
4. usercontrol的控制顺位和逻辑，怎么避免键位冲突
5. 不同状态的状态机编写
6. PID的基本原理&写法&调试逻辑
7. 怎么写自动路线

# 网站搭建
1. 类似GitHub内容的io网站，已经实现https://sjtu-vex.github.io/，需要增加登录的内容。
2. 可交互式
3. 可以插入具体的交互逻辑（如robot-config）从快到慢的转动效果
4. 有登录系统。若为登录显示是游客，只能看到基础层的内容。登录后可以查阅细节版的内容

# 学姐思路：
1. 看起来就是一套认证系统+一个支持 RBAC 的 wiki 系统
2. 如果想完全无脑的话直接 Django + django-wiki，优点是上面两个系统直接可以拿来用，而且不需要额外的集成。缺点就是认证服务器会锁死在 Django 生态上，如果想用别的框架会引入不必要的麻烦，以及 djagno wiki 的 markdown 方言不是很符合 github 风格（不过应该可以配置），也不是很现代。
3. 如果想两边分开，那认证系统需要成为一个 OAuth2.0 Provider 录入用户数据，wiki 如果找现成的轮子需要支持 OpenID Connect，或者 wiki 静态部署，文章根据路径让支持 OpenID 的 web 服务器做路由限制
4. 但如果目前你们没有认证服务器但是想尽快弄出 wiki，最简单的就是设个密码


# 使用步骤：
0. 具体的函数调用 -> VEX官方API：https://api.vex.com/v5/home/cpp/index.html
1. 给的代码的逻辑（从main等）

2. 配置电机
    ## 选好端口，用true和false来实现电机的正转和反转
    ## 文件怎么连接： 使用头文件的.h，把函数和变量放在.h 文件就是开放了接口。在文件的最开始使用#include ......来实现文件的串联
    ## 外部需要调用的部分前面加上 extern
    ## 调用电机的方式（pct/电压/转速）

3. 球路状态部分（cout具体内容） -> 状态机
   ## 为什么要用状态机： a. 实现非阻遏控制 b. 实现球路和运动单独控制
   ## 状态机怎么写？
   ## 是否需要防堵转的代码？
   // 在 basic_function.h 中定义
enum MotorEvent {
    STOP,
    INTAKE_FORWARD,
    INTAKE_REVERSE,
    SHOOTER_FORWARD,
    SHOOTER_REVERSE,
    BOTH_FORWARD,
    BOTH_REVERSE
};

// 改为
static MotorEvent motor_event = STOP;
static MotorEvent last_motor_event = STOP;

void setMotorEvent(MotorEvent _event) { motor_event = _event; }

void autonMotor(){
    switch(motor_event){
        case STOP:
            moveIntake(0);
            moveShooter(0);
            break;
        case INTAKE_FORWARD:
            moveIntake(100);
            moveShooter(0);
            break;
        case INTAKE_REVERSE:
            moveIntake(-100);
            moveShooter(0);
            break;
        case SHOOTER_FORWARD:
            moveIntake(0);
            moveShooter(100);
            break;
        case SHOOTER_REVERSE:
            moveIntake(0);
            moveShooter(-100);
            break;
        case BOTH_FORWARD:
            moveIntake(100);
            moveShooter(100);
            break;
        case BOTH_REVERSE:
            moveIntake(-100);
            moveShooter(-100);
            break;
    }
}

4. pid移动部分（cout）//需要cout和具体的cout出来具体的内容

5. usercontrol怎么写（怎么判断搜寻的逻辑/是否存在键位冲突）

6. display

7. 自动路线撰写（球路逻辑/巧思/自动停泊）

8. 提问部分：
a. 为什么需要先声明？一般不在头文件实现？单个程序内部为什么声明？
b. 类实现为什么要单独写出来，input具体是什么？
c. 怎么处理第一次的情况？有什么特殊的？
d. 怎么定义issettled？
e. 为什么要单独写出来具体的函数每个部分的void？而不是直接把所有的参数都写进运算过程？
f. 最后的电机输出是怎么实现具体的输出spin的角度的？
g. 最后要写成brake还是coast