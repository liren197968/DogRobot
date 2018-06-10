#ifndef __ROBOTCONTROL_H
#define __ROBOTCONTROL_H

#include <math.h>
#include <stdint.h>
/***************************数学计算常量********************************/
#define ROBOT_PI                            3.141593

#define ROBOT_MOVE_SERVO_NUM                12U

#define FONT_LEG                            1U
#define BACK_LEG                            2U

#define D1_LENH                             40.2
#define A1_LENH                             49.24
#define A2_LENH                             74
#define A3_LENH                             70

#define LEG_LENH                            D1_LENH
#define LEG_HIGH                            180
#define LEG_DOWN                            0

#define SERVO_ANGLE_TO_PWM                  2.27
#define ANGLE_TO_RADIAN                     3.141593 / 180
/**********************************************************************/
#define FONT_X_INIT                         LEG_HIGH
#define FONT_Y_INIT                         0
#define FONT_Z_INIT                         LEG_LENH
#define BACK_X_INIT                         LEG_HIGH
#define BACK_Y_INIT                         0
#define BACK_Z_INIT                         LEG_LENH
/*********************************************************************/
/*********************************************************************/
#define ROBOT_LEG_LIFT                      50                          //机器人抬腿时腿部、脚部抬升增量
#define ROBOT_FET_LIFT                      50

#define STEP_LENH                           5

#define BASIC_FSTEP_ERROR                   0
#define BASIC_BSTEP_ERROR                   4
#define BASIC_LFSTEP_ERROR                  -2
#define BASIC_LBSTEP_ERROR                  0
#define BASIC_RFSTEP_ERROR                  -2
#define BASIC_RBSTEP_ERROR                  0
#define ROBOT_PID_CONST_P                   1
/*********************************************************************/
void RobotInstruct_Control(void);
void MoveCoordinate_Calculate(double FStepLength, double SStepLength);

#endif
