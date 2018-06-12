#include "Servo.h"
#include "Speech.h"
//#include "Mpu6050.h"
#include "PCA9685.h"
#include "Bluetooth.h"
#include "RobotControl.h"

static double                   gDhDegree[3];

static int16_t                  gIServoPwmData[ROBOT_MOVE_SERVO_NUM] = {FOR_RIG_ARM_CENTER - 20, FOR_RIG_LEG_CENTER + 25, FOR_RIG_FET_CENTER + 45,
                                                                            BAK_RIG_ARM_CENTER - 20, BAK_RIG_LEG_CENTER - 60, BAK_RIG_FET_CENTER - 30,
                                                                            FOR_LEF_ARM_CENTER + 20, FOR_LEF_LEG_CENTER - 25, FOR_LEF_FET_CENTER - 45,
                                                                            BAK_LEF_ARM_CENTER + 20, BAK_LEF_LEG_CENTER + 60, BAK_LEF_FET_CENTER + 30};

static int16_t                  gSServoPwmData[ROBOT_MOVE_SERVO_NUM * 2] = {FOR_RIG_ARM_CENTER - 60, FOR_RIG_LEG_CENTER, FOR_RIG_FET_CENTER + 60,
                                                                            BAK_RIG_ARM_CENTER - 60, BAK_RIG_LEG_CENTER - 60, BAK_RIG_FET_CENTER,
                                                                            FOR_LEF_ARM_CENTER + 60, FOR_LEF_LEG_CENTER - 5, FOR_LEF_FET_CENTER - 70,
                                                                            BAK_LEF_ARM_CENTER + 60, BAK_LEF_LEG_CENTER + 60, BAK_LEF_FET_CENTER,

                                                                            FOR_RIG_ARM_CENTER + 20, FOR_RIG_LEG_CENTER + 50, FOR_RIG_FET_CENTER + 40,
                                                                            BAK_RIG_ARM_CENTER + 20, BAK_RIG_LEG_CENTER - 50, BAK_RIG_FET_CENTER - 60,
                                                                            FOR_LEF_ARM_CENTER - 20, FOR_LEF_LEG_CENTER - 50, FOR_LEF_FET_CENTER - 40,
                                                                            BAK_LEF_ARM_CENTER - 20, BAK_LEF_LEG_CENTER + 50, BAK_LEF_FET_CENTER + 60};

static int16_t                  CServoPwmData[ROBOT_MOVE_SERVO_NUM * 2] = {0};
/*************************D-H参数法反解计算函数*************************/
static void DhAlgorithm_Reverse(double x, double y, double z, uint8_t WhichLeg)
{
    double                      l = 0;
    double                      Beta1 = 0, Beta2 = 0;

    l = pow(pow(x - A3_LENH, 2) + pow(y, 2), 0.5);

    gDhDegree[1] = acos( (A1_LENH * A1_LENH + A2_LENH * A2_LENH - l * l) / (2 * A1_LENH * A2_LENH) )* 180 / ROBOT_PI;

    Beta1 = (atan(y / x)) * 180 / 3.141593;

    Beta2 = acos( (A1_LENH * A1_LENH + l * l - A2_LENH * A2_LENH) / (2 * A1_LENH * l) ) * 180 / ROBOT_PI;

    switch(WhichLeg)
    {
        case FONT_LEG:   gDhDegree[0] = Beta2 - Beta1;
                            break;

        case BACK_LEG:   gDhDegree[0] = Beta2 + Beta1;
                            break;

        default:            break;
    }

    gDhDegree[2] = 180 - gDhDegree[0] - gDhDegree[1];
}
/*********************************************************************/
/************************计算补偿修正后的坐标值*************************/
#ifdef CLOSE_CYCLE_CONTROL
static void ReviseCoordinate_Compute(uint8_t LegNum, double DeltaAlpha, double *x, double *y, double *z)
{
    static double           l = 0, r = 0;
    static double           c1 = 0, alfa1 = 0;
    static double           c2 = 0, alfa2 = 0;
    static double           c3 = 0, alfa3 = 0;
    static double           c4 = 0, alfa4 = 0;
    static double           c5 = 0, alfa5 = 0;
    static double           c6 = 0, alfa6 = 0;
    static double           gamma1 = 0, gamma2 = 0, gamma3 = 0;

    if(LegNum == 1)
    {
        r = ROBOT_LENH;
        l = STEP_LENH;

        gamma1 = BETA_INIT_1 - DeltaAlpha;
        gamma2 = BETA_INIT_2 - DeltaAlpha;
        gamma3 = BETA_INIT_3 - DeltaAlpha;

        c1 = sqrt( r * r - 2 * l * r * cos(gamma1 * ANGLE_TO_RADIAN) + l * l);
        c2 = sqrt( r * r - 2 * l * r * cos(gamma2 * ANGLE_TO_RADIAN) + l * l);
        c3 = sqrt( r * r - 2 * l * r * cos(gamma3 * ANGLE_TO_RADIAN) + l * l);
        c4 = sqrt( r * r - 2 * l * r * cos( (180 - gamma1) * ANGLE_TO_RADIAN ) + l * l);
        c5 = sqrt( r * r - 2 * l * r * cos( (180 - gamma2) * ANGLE_TO_RADIAN ) + l * l);
        c6 = sqrt( r * r - 2 * l * r * cos( (180 - gamma3) * ANGLE_TO_RADIAN ) + l * l);

        alfa1 = acos( (c1 * c1 + r * r - l * l) / (2 * c1 * r));
        alfa2 = acos( (c2 * c2 + r * r - l * l) / (2 * c2 * r));
        alfa3 = acos( (c3 * c3 + r * r - l * l) / (2 * c3 * r));
        alfa4 = acos( (c4 * c4 + r * r - l * l) / (2 * c4 * r));
        alfa5 = acos( (c5 * c5 + r * r - l * l) / (2 * c5 * r));
        alfa6 = acos( (c6 * c6 + r * r - l * l) / (2 * c6 * r));
    }

    if(LegNum == 1)
    {
        *x = c1 * cos(THET_INIT_1 * ANGLE_TO_RADIAN + DeltaAlpha * ANGLE_TO_RADIAN + alfa1);
        *y = c1 * sin(THET_INIT_1 * ANGLE_TO_RADIAN + DeltaAlpha * ANGLE_TO_RADIAN + alfa1);
    }
    else if(LegNum == 2)
    {
        *x = c2 * cos(THET_INIT_2 * ANGLE_TO_RADIAN + DeltaAlpha * ANGLE_TO_RADIAN + alfa2);
        *y = c2 * sin(THET_INIT_2 * ANGLE_TO_RADIAN + DeltaAlpha * ANGLE_TO_RADIAN + alfa2);
    }
    else if(LegNum == 3)
    {
        *x = c3 * cos(THET_INIT_3 * ANGLE_TO_RADIAN - DeltaAlpha * ANGLE_TO_RADIAN - alfa3);
        *y = c3 * sin(THET_INIT_3 * ANGLE_TO_RADIAN - DeltaAlpha * ANGLE_TO_RADIAN - alfa3);
    }
    else if(LegNum == 4)
    {
        *x = c4 * cos(THET_INIT_1 * ANGLE_TO_RADIAN + DeltaAlpha * ANGLE_TO_RADIAN - alfa4);
        *y = c4 * sin(THET_INIT_1 * ANGLE_TO_RADIAN + DeltaAlpha * ANGLE_TO_RADIAN - alfa4);
    }
    else if(LegNum == 5)
    {
        *x = c5 * cos(THET_INIT_2 * ANGLE_TO_RADIAN + DeltaAlpha * ANGLE_TO_RADIAN - alfa5);
        *y = c5 * sin(THET_INIT_2 * ANGLE_TO_RADIAN + DeltaAlpha * ANGLE_TO_RADIAN - alfa5);
    }
    else if(LegNum == 6)
    {
        *x = c6 * cos(THET_INIT_3 * ANGLE_TO_RADIAN - DeltaAlpha * ANGLE_TO_RADIAN + alfa6);
        *y = c6 * sin(THET_INIT_3 * ANGLE_TO_RADIAN - DeltaAlpha * ANGLE_TO_RADIAN + alfa6);
    }

    *z = -ROBOT_HIGH;
}
#endif
/**********************************************************************/
/***************************开环运动控制********************************/
void MoveCoordinate_Calculate(double FStepLength, double SStepLength)
{
    double                  x = 0, y = 0, z = 0;

    /***********the coordinate datas of the first group legs ****************/
    x = FONT_X_INIT;
    y = FONT_Y_INIT + FStepLength;
    z = FONT_Z_INIT;

    DhAlgorithm_Reverse(x, y, z, FONT_LEG);

    gSServoPwmData[0] = (uint16_t)(FOR_RIG_ARM_CENTER + gDhDegree[0] * SERVO_ANGLE_TO_PWM);
    gSServoPwmData[1] = (uint16_t)(FOR_RIG_LEG_CENTER + (180 - gDhDegree[1]) * SERVO_ANGLE_TO_PWM);
    gSServoPwmData[2] = (uint16_t)(FOR_RIG_FET_CENTER - gDhDegree[2] * SERVO_ANGLE_TO_PWM);
    gSServoPwmData[12] = (uint16_t)(FOR_RIG_ARM_CENTER + gDhDegree[0] * SERVO_ANGLE_TO_PWM);
    gSServoPwmData[13] = (uint16_t)(FOR_RIG_LEG_CENTER + gDhDegree[1] * SERVO_ANGLE_TO_PWM);
    gSServoPwmData[14] = (uint16_t)(FOR_RIG_FET_CENTER - gDhDegree[2] * SERVO_ANGLE_TO_PWM);


    x = BACK_X_INIT;
    y = BACK_Y_INIT + FStepLength;
    z = BACK_Z_INIT;

    DhAlgorithm_Reverse(x, y, z, BACK_LEG);

    gSServoPwmData[3] = (uint16_t)(BAK_RIG_ARM_CENTER - gDhDegree[0] * SERVO_ANGLE_TO_PWM);
    gSServoPwmData[4] = (uint16_t)(BAK_RIG_LEG_CENTER - (180 - gDhDegree[1]) * SERVO_ANGLE_TO_PWM);
    gSServoPwmData[5] = (uint16_t)(BAK_RIG_FET_CENTER + gDhDegree[2] * SERVO_ANGLE_TO_PWM);
    gSServoPwmData[15] = (uint16_t)(BAK_RIG_ARM_CENTER + gDhDegree[0] * SERVO_ANGLE_TO_PWM);
    gSServoPwmData[16] = (uint16_t)(BAK_RIG_LEG_CENTER + gDhDegree[1] * SERVO_ANGLE_TO_PWM);
    gSServoPwmData[17] = (uint16_t)(BAK_RIG_FET_CENTER - gDhDegree[2] * SERVO_ANGLE_TO_PWM);

    /************************************************************************/

    /***********the coordinate datas of the second group legs ***************/
    x = FONT_X_INIT;
    y = FONT_Y_INIT + SStepLength;
    z = FONT_Z_INIT;

    DhAlgorithm_Reverse(x, y, z, FONT_LEG);

    gSServoPwmData[6] = (uint16_t)(FOR_RIG_ARM_CENTER - gDhDegree[0] * SERVO_ANGLE_TO_PWM);
    gSServoPwmData[7] = (uint16_t)(FOR_RIG_LEG_CENTER + (180 - gDhDegree[1]) * SERVO_ANGLE_TO_PWM);
    gSServoPwmData[8] = (uint16_t)(FOR_RIG_FET_CENTER - gDhDegree[2] * SERVO_ANGLE_TO_PWM);
    gSServoPwmData[18] = (uint16_t)(FOR_RIG_ARM_CENTER + gDhDegree[0] * SERVO_ANGLE_TO_PWM);
    gSServoPwmData[19] = (uint16_t)(FOR_RIG_LEG_CENTER + gDhDegree[1] * SERVO_ANGLE_TO_PWM);
    gSServoPwmData[20] = (uint16_t)(FOR_RIG_FET_CENTER - gDhDegree[2] * SERVO_ANGLE_TO_PWM);

    x = BACK_X_INIT;
    y = BACK_Y_INIT + SStepLength;
    z = BACK_Z_INIT;

    DhAlgorithm_Reverse(x, y, z, BACK_LEG);

    gSServoPwmData[9] = (uint16_t)(BAK_RIG_ARM_CENTER + gDhDegree[0] * SERVO_ANGLE_TO_PWM);
    gSServoPwmData[10] = (uint16_t)(BAK_RIG_LEG_CENTER - (180 - gDhDegree[1]) * SERVO_ANGLE_TO_PWM);
    gSServoPwmData[11] = (uint16_t)(BAK_RIG_FET_CENTER + gDhDegree[2] * SERVO_ANGLE_TO_PWM);
    gSServoPwmData[21] = (uint16_t)(BAK_RIG_ARM_CENTER + gDhDegree[0] * SERVO_ANGLE_TO_PWM);
    gSServoPwmData[22] = (uint16_t)(BAK_RIG_LEG_CENTER + gDhDegree[1] * SERVO_ANGLE_TO_PWM);
    gSServoPwmData[23] = (uint16_t)(BAK_RIG_FET_CENTER - gDhDegree[2] * SERVO_ANGLE_TO_PWM);
    /***********************************************************************/
}
/**********************************************************************/
/***********************转圈运动计算函数********************************/
#ifdef CLOSE_CYCLE_CONTROL
static void CircleCoordinate_Calculate(uint8_t DeltaAngle)
{
    double          x = 0, y = 0, z = 0;

    /***********the shifting datas of the first group legs ******************/
    x = ROBOT_LENH * cos( (FONT_ARM_INIT_ANGLE + DeltaAngle) * ANGLE_TO_RADIAN);
    y = ROBOT_LENH * sin( (FONT_ARM_INIT_ANGLE + DeltaAngle) * ANGLE_TO_RADIAN);
    z = FONT_Z_INIT;

    DhAlgorithm_Reverse(x, y, z);

    CServoPwmData[0] = (uint16_t)(SERVO_PWM_CENTER + (FONT_ARM_INIT_ANGLE - gDhDegree[0]) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[1] = (uint16_t)(SERVO_PWM_CENTER_1 + (FONT_LEG_INIT_ANGLE + gDhDegree[1]) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[2] = (uint16_t)(SERVO_PWM_CENTER_2 + (FONT_FET_INIT_ANGLE - gDhDegree[2]) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[24] = (uint16_t)(SERVO_PWM_CENTER + (gDhDegree[0] - BACK_ARM_INIT_ANGLE) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[25] = (uint16_t)(SERVO_PWM_CENTER_1 + (BACK_LEG_INIT_ANGLE + gDhDegree[1]) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[26] = (uint16_t)(SERVO_PWM_CENTER_2 + (BACK_FET_INIT_ANGLE - gDhDegree[2]) * SERVO_ANGLE_TO_PWM);


    x = ROBOT_LENH * cos( (MIDE_ARM_INIT_ANGLE - DeltaAngle) * ANGLE_TO_RADIAN);
    y = ROBOT_LENH * sin( (MIDE_ARM_INIT_ANGLE - DeltaAngle) * ANGLE_TO_RADIAN);
    z = MIDE_Z_INIT;

    DhAlgorithm_Reverse(x, y, z);

    CServoPwmData[12] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_ARM_INIT_ANGLE - gDhDegree[0]) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[13] = (uint16_t)(SERVO_PWM_CENTER_1 + (MIDE_LEG_INIT_ANGLE + gDhDegree[1]) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[14] = (uint16_t)(SERVO_PWM_CENTER_2 + (MIDE_FET_INIT_ANGLE - gDhDegree[2]) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[30] = (uint16_t)(SERVO_PWM_CENTER + (gDhDegree[0] - MIDE_ARM_INIT_ANGLE) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[31] = (uint16_t)(SERVO_PWM_CENTER_1 + (MIDE_LEG_INIT_ANGLE + gDhDegree[1]) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[32] = (uint16_t)(SERVO_PWM_CENTER_2 + (MIDE_FET_INIT_ANGLE - gDhDegree[2]) * SERVO_ANGLE_TO_PWM);


    x = ROBOT_LENH * cos( (BACK_ARM_INIT_ANGLE - DeltaAngle) * ANGLE_TO_RADIAN);
    y = ROBOT_LENH * sin( (BACK_ARM_INIT_ANGLE - DeltaAngle) * ANGLE_TO_RADIAN);
    z = BACK_Z_INIT;

    DhAlgorithm_Reverse(x, y, z);

    CServoPwmData[6] = (uint16_t)(SERVO_PWM_CENTER + (gDhDegree[0] - BACK_ARM_INIT_ANGLE) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[7] = (uint16_t)(SERVO_PWM_CENTER_1 + (BACK_LEG_INIT_ANGLE + gDhDegree[1]) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[8] = (uint16_t)(SERVO_PWM_CENTER_2 + (BACK_FET_INIT_ANGLE - gDhDegree[2]) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[18] = (uint16_t)(SERVO_PWM_CENTER + (FONT_ARM_INIT_ANGLE - gDhDegree[0]) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[19] = (uint16_t)(SERVO_PWM_CENTER_1 + (FONT_LEG_INIT_ANGLE + gDhDegree[1]) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[20] = (uint16_t)(SERVO_PWM_CENTER_2 + (FONT_FET_INIT_ANGLE - gDhDegree[2]) * SERVO_ANGLE_TO_PWM);
    /************************************************************************/

    /***********the shifting datas of the second group legs *****************/
    x = ROBOT_LENH * cos( (FONT_ARM_INIT_ANGLE - DeltaAngle) * ANGLE_TO_RADIAN);
    y = ROBOT_LENH * sin( (FONT_ARM_INIT_ANGLE - DeltaAngle) * ANGLE_TO_RADIAN);
    z = FONT_Z_INIT;

    DhAlgorithm_Reverse(x, y, z);

    CServoPwmData[9] = (uint16_t)(SERVO_PWM_CENTER + (FONT_ARM_INIT_ANGLE - gDhDegree[0]) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[10] = (uint16_t)(SERVO_PWM_CENTER_1 + (FONT_LEG_INIT_ANGLE + gDhDegree[1]) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[11] = (uint16_t)(SERVO_PWM_CENTER_2 + (FONT_FET_INIT_ANGLE - gDhDegree[2]) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[33] = (uint16_t)(SERVO_PWM_CENTER + (gDhDegree[0] - BACK_ARM_INIT_ANGLE) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[34] = (uint16_t)(SERVO_PWM_CENTER_1 + (BACK_LEG_INIT_ANGLE + gDhDegree[1]) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[35] = (uint16_t)(SERVO_PWM_CENTER_2 + (BACK_FET_INIT_ANGLE - gDhDegree[2]) * SERVO_ANGLE_TO_PWM);


    x = ROBOT_LENH * cos( (MIDE_ARM_INIT_ANGLE + DeltaAngle) * ANGLE_TO_RADIAN);
    y = ROBOT_LENH * sin( (MIDE_ARM_INIT_ANGLE + DeltaAngle) * ANGLE_TO_RADIAN);
    z = MIDE_Z_INIT;

    DhAlgorithm_Reverse(x, y, z);

    CServoPwmData[3] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_ARM_INIT_ANGLE - gDhDegree[0]) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[4] = (uint16_t)(SERVO_PWM_CENTER_1 + (MIDE_LEG_INIT_ANGLE + gDhDegree[1]) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[5] = (uint16_t)(SERVO_PWM_CENTER_2 + (MIDE_FET_INIT_ANGLE - gDhDegree[2]) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[21] = (uint16_t)(SERVO_PWM_CENTER + (gDhDegree[0] - MIDE_ARM_INIT_ANGLE) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[22] = (uint16_t)(SERVO_PWM_CENTER_1 + (MIDE_LEG_INIT_ANGLE + gDhDegree[1]) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[23] = (uint16_t)(SERVO_PWM_CENTER_2 + (MIDE_FET_INIT_ANGLE - gDhDegree[2]) * SERVO_ANGLE_TO_PWM);

    x = ROBOT_LENH * cos( (BACK_ARM_INIT_ANGLE + DeltaAngle) * ANGLE_TO_RADIAN);
    y = ROBOT_LENH * sin( (BACK_ARM_INIT_ANGLE + DeltaAngle) * ANGLE_TO_RADIAN);
    z = BACK_Z_INIT;

    DhAlgorithm_Reverse(x, y, z);

    CServoPwmData[15] = (uint16_t)(SERVO_PWM_CENTER + (gDhDegree[0] - BACK_ARM_INIT_ANGLE) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[16] = (uint16_t)(SERVO_PWM_CENTER_1 + (BACK_LEG_INIT_ANGLE + gDhDegree[1]) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[17] = (uint16_t)(SERVO_PWM_CENTER_2 + (BACK_FET_INIT_ANGLE - gDhDegree[2]) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[27] = (uint16_t)(SERVO_PWM_CENTER + (FONT_ARM_INIT_ANGLE - gDhDegree[0]) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[28] = (uint16_t)(SERVO_PWM_CENTER_1 + (FONT_LEG_INIT_ANGLE + gDhDegree[1]) * SERVO_ANGLE_TO_PWM);
    CServoPwmData[29] = (uint16_t)(SERVO_PWM_CENTER_2 + (FONT_FET_INIT_ANGLE - gDhDegree[2]) * SERVO_ANGLE_TO_PWM);
}
#endif
/**********************************************************************/
/*************************步长修正计算函数******************************/
#ifdef CLOSE_CYCLE_CONTROL
static void AdjustStepLen_Calculate(double ExpectedAngle, uint8_t direction)
{
    double      CurrentAngle = 0;
    double      x = 0, y = 0, z = 0;

    CurrentAngle = yaw;
/********************************前进功能***************************************************************/
    if(direction == 0)
    {
        FAngleError = (CurrentAngle - ExpectedAngle) * ROBOT_PID_CONST_P;
        SAngleError = (ExpectedAngle - CurrentAngle) * ROBOT_PID_CONST_P + BASIC_FSTEP_ERROR;

        if(FAngleError >= 15)
        {
            FAngleError = 15;
        }
        else
        {
            if(FAngleError < -15)
            {
                FAngleError = -15;
            }
        }

        if(SAngleError >= 15)
        {
            SAngleError = 15;
        }
        else
        {
            if(SAngleError < -15)
            {
                SAngleError = -15;
            }
        }
        /*****************************************第一组腿***********************************************/
        ReviseCoordinate_Compute(1, FAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        FServoPwmData[0] = (uint16_t)(SERVO_PWM_CENTER + (FONT_ARM_INIT_ANGLE - gDhDegree[0]) * SERVO_ANGLE_TO_PWM);
        FServoPwmData[1] = (uint16_t)(SERVO_PWM_CENTER_1 + (FONT_LEG_INIT_ANGLE + gDhDegree[1]) * SERVO_ANGLE_TO_PWM);
        FServoPwmData[2] = (uint16_t)(SERVO_PWM_CENTER_2 + (FONT_FET_INIT_ANGLE - gDhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(2, FAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        FServoPwmData[12] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_ARM_INIT_ANGLE - gDhDegree[0]) * SERVO_ANGLE_TO_PWM);
        FServoPwmData[13] = (uint16_t)(SERVO_PWM_CENTER_1 + (MIDE_LEG_INIT_ANGLE + gDhDegree[1]) * SERVO_ANGLE_TO_PWM);
        FServoPwmData[14] = (uint16_t)(SERVO_PWM_CENTER_2 + (MIDE_FET_INIT_ANGLE - gDhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(3, FAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        FServoPwmData[6] = (uint16_t)(SERVO_PWM_CENTER + (gDhDegree[0] - BACK_ARM_INIT_ANGLE) * SERVO_ANGLE_TO_PWM);
        FServoPwmData[7] = (uint16_t)(SERVO_PWM_CENTER_1 + (BACK_LEG_INIT_ANGLE + gDhDegree[1]) * SERVO_ANGLE_TO_PWM);
        FServoPwmData[8] = (uint16_t)(SERVO_PWM_CENTER_2 + (BACK_FET_INIT_ANGLE - gDhDegree[2]) * SERVO_ANGLE_TO_PWM);
        /*****************************************第二组腿***********************************************/
        ReviseCoordinate_Compute(1, SAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        FServoPwmData[9] = (uint16_t)(SERVO_PWM_CENTER + (FONT_ARM_INIT_ANGLE - gDhDegree[0]) * SERVO_ANGLE_TO_PWM);
        FServoPwmData[10] = (uint16_t)(SERVO_PWM_CENTER_1 + (FONT_LEG_INIT_ANGLE + gDhDegree[1]) * SERVO_ANGLE_TO_PWM);
        FServoPwmData[11] = (uint16_t)(SERVO_PWM_CENTER_2 + (FONT_FET_INIT_ANGLE - gDhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(2, SAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        FServoPwmData[3] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_ARM_INIT_ANGLE - gDhDegree[0]) * SERVO_ANGLE_TO_PWM);
        FServoPwmData[4] = (uint16_t)(SERVO_PWM_CENTER_1 + (MIDE_LEG_INIT_ANGLE + gDhDegree[1]) * SERVO_ANGLE_TO_PWM);
        FServoPwmData[5] = (uint16_t)(SERVO_PWM_CENTER_2 + (MIDE_FET_INIT_ANGLE - gDhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(3, SAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        FServoPwmData[15] = (uint16_t)(SERVO_PWM_CENTER + (gDhDegree[0] - BACK_ARM_INIT_ANGLE) * SERVO_ANGLE_TO_PWM);
        FServoPwmData[16] = (uint16_t)(SERVO_PWM_CENTER_1 + (BACK_LEG_INIT_ANGLE + gDhDegree[1]) * SERVO_ANGLE_TO_PWM);
        FServoPwmData[17] = (uint16_t)(SERVO_PWM_CENTER_2 + (BACK_FET_INIT_ANGLE - gDhDegree[2]) * SERVO_ANGLE_TO_PWM);

        /*****************************************第一组腿************操作与前一组操作相同***************/
        ReviseCoordinate_Compute(4, FAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        FServoPwmData[18] = (uint16_t)(SERVO_PWM_CENTER + (FONT_ARM_INIT_ANGLE - gDhDegree[0]) * SERVO_ANGLE_TO_PWM);
        FServoPwmData[19] = (uint16_t)(SERVO_PWM_CENTER_1 + (FONT_LEG_INIT_ANGLE + gDhDegree[1]) * SERVO_ANGLE_TO_PWM);
        FServoPwmData[20] = (uint16_t)(SERVO_PWM_CENTER_2 + (FONT_FET_INIT_ANGLE - gDhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(5, FAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        FServoPwmData[30] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_ARM_INIT_ANGLE - gDhDegree[0]) * SERVO_ANGLE_TO_PWM);
        FServoPwmData[31] = (uint16_t)(SERVO_PWM_CENTER_1 + (MIDE_LEG_INIT_ANGLE + gDhDegree[1]) * SERVO_ANGLE_TO_PWM);
        FServoPwmData[32] = (uint16_t)(SERVO_PWM_CENTER_2 + (MIDE_FET_INIT_ANGLE - gDhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(6, FAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        FServoPwmData[24] = (uint16_t)(SERVO_PWM_CENTER + (gDhDegree[0] - BACK_ARM_INIT_ANGLE) * SERVO_ANGLE_TO_PWM);
        FServoPwmData[25] = (uint16_t)(SERVO_PWM_CENTER_1 + (BACK_LEG_INIT_ANGLE + gDhDegree[1]) * SERVO_ANGLE_TO_PWM);
        FServoPwmData[26] = (uint16_t)(SERVO_PWM_CENTER_2 + (BACK_FET_INIT_ANGLE - gDhDegree[2]) * SERVO_ANGLE_TO_PWM);
        /*****************************************第二组腿***********************************************/
        ReviseCoordinate_Compute(4, SAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        FServoPwmData[27] = (uint16_t)(SERVO_PWM_CENTER + (FONT_ARM_INIT_ANGLE - gDhDegree[0]) * SERVO_ANGLE_TO_PWM);
        FServoPwmData[28] = (uint16_t)(SERVO_PWM_CENTER_1 + (FONT_LEG_INIT_ANGLE + gDhDegree[1]) * SERVO_ANGLE_TO_PWM);
        FServoPwmData[29] = (uint16_t)(SERVO_PWM_CENTER_2 + (FONT_FET_INIT_ANGLE - gDhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(5, SAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        FServoPwmData[21] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_ARM_INIT_ANGLE - gDhDegree[0]) * SERVO_ANGLE_TO_PWM);
        FServoPwmData[22] = (uint16_t)(SERVO_PWM_CENTER_1 + (MIDE_LEG_INIT_ANGLE + gDhDegree[1]) * SERVO_ANGLE_TO_PWM);
        FServoPwmData[23] = (uint16_t)(SERVO_PWM_CENTER_2 + (MIDE_FET_INIT_ANGLE - gDhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(6, SAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        FServoPwmData[33] = (uint16_t)(SERVO_PWM_CENTER + (gDhDegree[0] - BACK_ARM_INIT_ANGLE) * SERVO_ANGLE_TO_PWM);
        FServoPwmData[34] = (uint16_t)(SERVO_PWM_CENTER_1 + (BACK_LEG_INIT_ANGLE + gDhDegree[1]) * SERVO_ANGLE_TO_PWM);
        FServoPwmData[35] = (uint16_t)(SERVO_PWM_CENTER_2 + (BACK_FET_INIT_ANGLE - gDhDegree[2]) * SERVO_ANGLE_TO_PWM);
    }
/********************************后退功能*****************后退操作与前进操作相同************************/
    if(direction == 1)
    {
        FAngleError = (CurrentAngle - ExpectedAngle) * ROBOT_PID_CONST_P;
        SAngleError = (ExpectedAngle - CurrentAngle) * ROBOT_PID_CONST_P + BASIC_FSTEP_ERROR;

        if(FAngleError >= 15)
        {
            FAngleError = 15;
        }
        else
        {
            if(FAngleError < -15)
            {
                FAngleError = -15;
            }
        }
        
        if(SAngleError >= 15)
        {
            SAngleError = 15;
        }
        else
        {
            if(SAngleError < -15)
            {
                SAngleError = -15;
            }
        }
        /*****************************************第一组腿***********************************************/
        ReviseCoordinate_Compute(3, SAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        BServoPwmData[0] = (uint16_t)(SERVO_PWM_CENTER + (FONT_ARM_INIT_ANGLE - gDhDegree[0]) * SERVO_ANGLE_TO_PWM);
        BServoPwmData[1] = (uint16_t)(SERVO_PWM_CENTER_1 + (FONT_LEG_INIT_ANGLE + gDhDegree[1]) * SERVO_ANGLE_TO_PWM);
        BServoPwmData[2] = (uint16_t)(SERVO_PWM_CENTER_2 + (FONT_FET_INIT_ANGLE - gDhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(5, SAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        BServoPwmData[12] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_ARM_INIT_ANGLE - gDhDegree[0]) * SERVO_ANGLE_TO_PWM);
        BServoPwmData[13] = (uint16_t)(SERVO_PWM_CENTER_1 + (MIDE_LEG_INIT_ANGLE + gDhDegree[1]) * SERVO_ANGLE_TO_PWM);
        BServoPwmData[14] = (uint16_t)(SERVO_PWM_CENTER_2 + (MIDE_FET_INIT_ANGLE - gDhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(1, SAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        BServoPwmData[6] = (uint16_t)(SERVO_PWM_CENTER + (gDhDegree[0] - BACK_ARM_INIT_ANGLE) * SERVO_ANGLE_TO_PWM);
        BServoPwmData[7] = (uint16_t)(SERVO_PWM_CENTER_1 + (BACK_LEG_INIT_ANGLE + gDhDegree[1]) * SERVO_ANGLE_TO_PWM);
        BServoPwmData[8] = (uint16_t)(SERVO_PWM_CENTER_2 + (BACK_FET_INIT_ANGLE - gDhDegree[2]) * SERVO_ANGLE_TO_PWM);
        /*****************************************第二组腿***********************************************/
        ReviseCoordinate_Compute(3, FAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        BServoPwmData[9] = (uint16_t)(SERVO_PWM_CENTER + (FONT_ARM_INIT_ANGLE - gDhDegree[0]) * SERVO_ANGLE_TO_PWM);
        BServoPwmData[10] = (uint16_t)(SERVO_PWM_CENTER_1 + (FONT_LEG_INIT_ANGLE + gDhDegree[1]) * SERVO_ANGLE_TO_PWM);
        BServoPwmData[11] = (uint16_t)(SERVO_PWM_CENTER_2 + (FONT_FET_INIT_ANGLE - gDhDegree[2]) * SERVO_ANGLE_TO_PWM);

        ReviseCoordinate_Compute(5, FAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        BServoPwmData[3] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_ARM_INIT_ANGLE - gDhDegree[0]) * SERVO_ANGLE_TO_PWM);
        BServoPwmData[4] = (uint16_t)(SERVO_PWM_CENTER_1 + (MIDE_LEG_INIT_ANGLE + gDhDegree[1]) * SERVO_ANGLE_TO_PWM);
        BServoPwmData[5] = (uint16_t)(SERVO_PWM_CENTER_2 + (MIDE_FET_INIT_ANGLE - gDhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(1, FAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        BServoPwmData[15] = (uint16_t)(SERVO_PWM_CENTER + (gDhDegree[0] - BACK_ARM_INIT_ANGLE) * SERVO_ANGLE_TO_PWM);
        BServoPwmData[16] = (uint16_t)(SERVO_PWM_CENTER_1 + (BACK_LEG_INIT_ANGLE + gDhDegree[1]) * SERVO_ANGLE_TO_PWM);
        BServoPwmData[17] = (uint16_t)(SERVO_PWM_CENTER_2 + (BACK_FET_INIT_ANGLE - gDhDegree[2]) * SERVO_ANGLE_TO_PWM);

        /*****************************************第一组腿************操作与前一组操作相同***************/
        ReviseCoordinate_Compute(6, SAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        BServoPwmData[18] = (uint16_t)(SERVO_PWM_CENTER + (FONT_ARM_INIT_ANGLE - gDhDegree[0]) * SERVO_ANGLE_TO_PWM);
        BServoPwmData[19] = (uint16_t)(SERVO_PWM_CENTER_1 + (FONT_LEG_INIT_ANGLE + gDhDegree[1]) * SERVO_ANGLE_TO_PWM);
        BServoPwmData[20] = (uint16_t)(SERVO_PWM_CENTER_2 + (FONT_FET_INIT_ANGLE - gDhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(2, SAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        BServoPwmData[30] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_ARM_INIT_ANGLE - gDhDegree[0]) * SERVO_ANGLE_TO_PWM);
        BServoPwmData[31] = (uint16_t)(SERVO_PWM_CENTER_1 + (MIDE_LEG_INIT_ANGLE + gDhDegree[1]) * SERVO_ANGLE_TO_PWM);
        BServoPwmData[32] = (uint16_t)(SERVO_PWM_CENTER_2 + (MIDE_FET_INIT_ANGLE - gDhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(4, SAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        BServoPwmData[24] = (uint16_t)(SERVO_PWM_CENTER + (gDhDegree[0] - BACK_ARM_INIT_ANGLE) * SERVO_ANGLE_TO_PWM);
        BServoPwmData[25] = (uint16_t)(SERVO_PWM_CENTER_1 + (BACK_LEG_INIT_ANGLE + gDhDegree[1]) * SERVO_ANGLE_TO_PWM);
        BServoPwmData[26] = (uint16_t)(SERVO_PWM_CENTER_2 + (BACK_FET_INIT_ANGLE - gDhDegree[2]) * SERVO_ANGLE_TO_PWM);
        /*****************************************第二组腿***********************************************/
        ReviseCoordinate_Compute(6, FAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        BServoPwmData[27] = (uint16_t)(SERVO_PWM_CENTER + (FONT_ARM_INIT_ANGLE - gDhDegree[0]) * SERVO_ANGLE_TO_PWM);
        BServoPwmData[28] = (uint16_t)(SERVO_PWM_CENTER_1 + (FONT_LEG_INIT_ANGLE + gDhDegree[1]) * SERVO_ANGLE_TO_PWM);
        BServoPwmData[29] = (uint16_t)(SERVO_PWM_CENTER_2 + (FONT_FET_INIT_ANGLE - gDhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(2, FAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        BServoPwmData[21] = (uint16_t)(SERVO_PWM_CENTER + (MIDE_ARM_INIT_ANGLE - gDhDegree[0]) * SERVO_ANGLE_TO_PWM);
        BServoPwmData[22] = (uint16_t)(SERVO_PWM_CENTER_1 + (MIDE_LEG_INIT_ANGLE + gDhDegree[1]) * SERVO_ANGLE_TO_PWM);
        BServoPwmData[23] = (uint16_t)(SERVO_PWM_CENTER_2 + (MIDE_FET_INIT_ANGLE - gDhDegree[2]) * SERVO_ANGLE_TO_PWM);


        ReviseCoordinate_Compute(4, FAngleError, &x, &y, &z);
        DhAlgorithm_Reverse(x, y, z);

        BServoPwmData[33] = (uint16_t)(SERVO_PWM_CENTER + (gDhDegree[0] - BACK_ARM_INIT_ANGLE) * SERVO_ANGLE_TO_PWM);
        BServoPwmData[34] = (uint16_t)(SERVO_PWM_CENTER_1 + (BACK_LEG_INIT_ANGLE + gDhDegree[1]) * SERVO_ANGLE_TO_PWM);
        BServoPwmData[35] = (uint16_t)(SERVO_PWM_CENTER_2 + (BACK_FET_INIT_ANGLE - gDhDegree[2]) * SERVO_ANGLE_TO_PWM);
    }
}
#endif
/**********************************************************************/
/************************机器人第一条腿控制函数**************************/
static void FontRigtLeg_Control(uint16_t FirstParm, uint16_t SecndParm, uint16_t ThirdParm)
{
    gPwmExpetVal[0] = FirstParm;
    gPwmExpetVal[1] = SecndParm;
    gPwmExpetVal[2] = ThirdParm;
}
/**********************************************************************/
/************************机器人第二条腿控制函数**************************/
static void BackRigtLeg_Control(uint16_t FirstParm, uint16_t SecndParm, uint16_t ThirdParm)
{
    gPwmExpetVal[3] = FirstParm;
    gPwmExpetVal[4] = SecndParm;
    gPwmExpetVal[5] = ThirdParm;
}
/**********************************************************************/
/************************机器人第三条腿控制函数**************************/
static void FontLeftLeg_Control(uint16_t FirstParm, uint16_t SecndParm, uint16_t ThirdParm)
{
    gPwmExpetVal[9] = FirstParm;
    gPwmExpetVal[10] = SecndParm;
    gPwmExpetVal[11] = ThirdParm;
}
/**********************************************************************/
/************************机器人第四条腿控制函数**************************/
static void BackLeftLeg_Control(uint16_t FirstParm, uint16_t SecndParm, uint16_t ThirdParm)
{
    gPwmExpetVal[12] = FirstParm;
    gPwmExpetVal[13] = SecndParm;
    gPwmExpetVal[14] = ThirdParm;
}
/**********************************************************************/
/***********************机器人站立函数*********************************/
void DogRobot_Stand(void)
{
    uint8_t             i = 0;
    static uint8_t      CalcuFlag = 0;

    if(!CalcuFlag)
    {
        CalcuFlag = 1;
        MoveCoordinate_Calculate(0, 0);
    }

    for(i = 0; i < ROBOT_MOVE_SERVO_NUM; i++)
    {
        gIServoPwmData[i] = gSServoPwmData[i];
    }

    FontRigtLeg_Control(gIServoPwmData[0], gIServoPwmData[1], gIServoPwmData[2]);                                                        //将所有舵机赋值为初始值，机器人站立待命
    BackRigtLeg_Control(gIServoPwmData[3], gIServoPwmData[4], gIServoPwmData[5]);
    FontLeftLeg_Control(gIServoPwmData[6], gIServoPwmData[7], gIServoPwmData[8]);
    BackLeftLeg_Control(gIServoPwmData[9], gIServoPwmData[10], gIServoPwmData[11]);
}
/*********************************************************************/
/************************机器人前进函数********************************/
static void DogRobot_Forward(void)
{
    static uint8_t      Flag = 0;                                                                                                   //Flag标记用于控制机器人动作组的执行顺序

    Flag++;

    if(Flag > 4)
    {
        Flag = 1;
    }

#ifdef CLOSE_CYCLE_CONTROL
    if(!StartMoveFlag)
    {
        StartMoveFlag = 1;

        ExpectedAngle = yaw;
    }
#endif

    switch(Flag)
    {
        case 1: //AdjustStepLen_Calculate(ExpectedAngle, 0);
                //MoveCoordinate_Calculate(STEP_LENH, STEP_LENH);

                FontRigtLeg_Control(gSServoPwmData[0], gIServoPwmData[1] + ROBOT_LEG_LIFT, gIServoPwmData[2] + ROBOT_FET_LIFT);          //第一步：第一组腿抬脚前移
                BackLeftLeg_Control(gSServoPwmData[9], gIServoPwmData[10] - ROBOT_LEG_LIFT, gIServoPwmData[11] - ROBOT_FET_LIFT);

                FontLeftLeg_Control(gIServoPwmData[6], gIServoPwmData[7], gIServoPwmData[8]);
                BackRigtLeg_Control(gIServoPwmData[3], gIServoPwmData[4], gIServoPwmData[5]);                                            //同时，第二组腿向后滑动
                break;

        case 2: FontRigtLeg_Control(gSServoPwmData[0], gSServoPwmData[1], gSServoPwmData[2]);                                             //第二步：第一组腿落地
                BackLeftLeg_Control(gSServoPwmData[9], gSServoPwmData[10], gSServoPwmData[11]);

                FontLeftLeg_Control(gSServoPwmData[18], gSServoPwmData[19], gSServoPwmData[20]);
                BackRigtLeg_Control(gSServoPwmData[15], gSServoPwmData[16], gSServoPwmData[17]);                                           //同时，第二组腿继续向后滑动
                break;

        case 3: FontRigtLeg_Control(gIServoPwmData[0], gIServoPwmData[1], gIServoPwmData[2]);                                             //第三步：第一组腿向后滑动
                BackLeftLeg_Control(gIServoPwmData[9], gIServoPwmData[10], gIServoPwmData[11]);

                FontLeftLeg_Control(gSServoPwmData[6], gIServoPwmData[7] - ROBOT_LEG_LIFT, gIServoPwmData[8] - ROBOT_FET_LIFT);
                BackRigtLeg_Control(gSServoPwmData[3], gIServoPwmData[4] + ROBOT_LEG_LIFT, gIServoPwmData[5] + ROBOT_FET_LIFT);           //同时，第二组腿抬腿
                break; 

        case 4: FontRigtLeg_Control(gSServoPwmData[12], gSServoPwmData[13], gSServoPwmData[14]);                                          //第四步：第一组腿继续向后滑动
                BackLeftLeg_Control(gSServoPwmData[21], gSServoPwmData[22], gSServoPwmData[23]);

                FontLeftLeg_Control(gSServoPwmData[6], gSServoPwmData[7], gSServoPwmData[8]);
                BackRigtLeg_Control(gSServoPwmData[3], gSServoPwmData[4], gSServoPwmData[5]);                                            //同时，第二组腿落地
                break;

        default:break;
    }
}
/*********************************************************************/
/*************************机器人后退函数*******************************/
static void DogRobot_Backwrd(void)
{
//    static uint8_t          Flag = 0;                                                                                                        //Flag标记用于控制机器人动作组的执行顺序

//    Flag++;

//    if(Flag > 4)
//    {
//        Flag = 1;
//    }

//    if(! StartMoveFlag)
//    {
//        StartMoveFlag = 1;
//        //ExpectedAngle = yaw;
//    }

//    switch(Flag)
//    {
//        case 1: //AdjustStepLen_Calculate(ExpectedAngle, 1);
//        
//                FontRigtLeg_Control(BServoPwmData[0], gSServoPwmData[1] + ROBOT_LEG_LIFT, gSServoPwmData[2] + ROBOT_FET_LIFT);           //第一步：第一组腿抬脚前移
//                FifthLeg_Control(BServoPwmData[12], gSServoPwmData[13] + ROBOT_LEG_LIFT, gSServoPwmData[14] + ROBOT_FET_LIFT);
//                FontLeftLeg_Control(BServoPwmData[6], gSServoPwmData[7] + ROBOT_LEG_LIFT, gSServoPwmData[8] + ROBOT_FET_LIFT);

//                BackLeftLeg_Control(gSServoPwmData[9], gSServoPwmData[10], gSServoPwmData[11]);
//                BackRigtLeg_Control(gSServoPwmData[3], gSServoPwmData[4], gSServoPwmData[5]);                                             //同时，第二组腿向后滑动
//                SixthLeg_Control(gSServoPwmData[15], gSServoPwmData[16], gSServoPwmData[17]);
//                break;

//        case 2: FontRigtLeg_Control(BServoPwmData[0], BServoPwmData[1], BServoPwmData[2]);                                             //第二步：第一组腿落地
//                FifthLeg_Control(BServoPwmData[12], BServoPwmData[13], BServoPwmData[14]);
//                FontLeftLeg_Control(BServoPwmData[6], BServoPwmData[7], BServoPwmData[8]);

//                BackLeftLeg_Control(BServoPwmData[27], BServoPwmData[28], BServoPwmData[29]);
//                BackRigtLeg_Control(BServoPwmData[21], BServoPwmData[22], BServoPwmData[23]);                                          //同时，第二组腿继续向后滑动
//                SixthLeg_Control(BServoPwmData[33], BServoPwmData[34], BServoPwmData[35]);
//                break;

//        case 3: FontRigtLeg_Control(gSServoPwmData[0], gSServoPwmData[1], gSServoPwmData[2]);                                             //第三步：第一组腿向后滑动
//                FifthLeg_Control(gSServoPwmData[12], gSServoPwmData[13], gSServoPwmData[14]);
//                FontLeftLeg_Control(gSServoPwmData[6], gSServoPwmData[7], gSServoPwmData[8]);
//        
//                BackLeftLeg_Control(BServoPwmData[9], gSServoPwmData[10] + ROBOT_LEG_LIFT, gSServoPwmData[11] + ROBOT_FET_LIFT);
//                BackRigtLeg_Control(BServoPwmData[3], gSServoPwmData[4] + ROBOT_LEG_LIFT, gSServoPwmData[5] + ROBOT_FET_LIFT);           //同时，第二组腿抬腿
//                SixthLeg_Control(BServoPwmData[15], gSServoPwmData[16] + ROBOT_LEG_LIFT, gSServoPwmData[17] + ROBOT_FET_LIFT);
//                break; 

//        case 4: FontRigtLeg_Control(BServoPwmData[18], BServoPwmData[19], BServoPwmData[20]);                                          //第四步：第一组腿继续向后滑动
//                FifthLeg_Control(BServoPwmData[30], BServoPwmData[31], BServoPwmData[32]);
//                FontLeftLeg_Control(BServoPwmData[24], BServoPwmData[25], BServoPwmData[26]);
//                
//                BackLeftLeg_Control(BServoPwmData[9], BServoPwmData[10], BServoPwmData[11]);                                           //同时，第二组腿落地
//                BackRigtLeg_Control(BServoPwmData[3], BServoPwmData[4], BServoPwmData[5]);
//                SixthLeg_Control(BServoPwmData[15], BServoPwmData[16], BServoPwmData[17]);
//                break;

//        default:break;
//    }
}
/*********************************************************************/
/************************机器人逆时针转圈函数**************************/
static void DogRobot_AtiClck(void)
{
//    static uint8_t          Flag = 0;

//    Flag++;

//    StartMoveFlag = 0;

//    if(Flag > 4)
//    {
//        Flag = 1;
//    }

//    switch(Flag)
//    {
//        case 1: //CircleCoordinate_Calculate(10);

//                FontRigtLeg_Control(CServoPwmData[18], gSServoPwmData[1] + ROBOT_LEG_LIFT, gSServoPwmData[2] + ROBOT_FET_LIFT);         //第一步：第一组腿抬脚前移
//                FifthLeg_Control(CServoPwmData[30], gSServoPwmData[13] + ROBOT_LEG_LIFT, gSServoPwmData[14] + ROBOT_FET_LIFT);
//                FontLeftLeg_Control(CServoPwmData[24], gSServoPwmData[7] + ROBOT_LEG_LIFT, gSServoPwmData[8] + ROBOT_FET_LIFT);

//                BackLeftLeg_Control(gSServoPwmData[9], gSServoPwmData[10], gSServoPwmData[11]);
//                BackRigtLeg_Control(gSServoPwmData[3], gSServoPwmData[4], gSServoPwmData[5]);                                            //同时，第二组腿向后滑动
//                SixthLeg_Control(gSServoPwmData[15], gSServoPwmData[16], gSServoPwmData[17]);
//                break;

//        case 2: FontRigtLeg_Control(CServoPwmData[18], CServoPwmData[19], CServoPwmData[20]);                                          //第二步：第一组腿落地
//                FifthLeg_Control(CServoPwmData[30], CServoPwmData[31], CServoPwmData[32]);
//                FontLeftLeg_Control(CServoPwmData[24], CServoPwmData[25], CServoPwmData[26]);

//                BackLeftLeg_Control(CServoPwmData[9], CServoPwmData[10], CServoPwmData[11]);
//                BackRigtLeg_Control(CServoPwmData[3], CServoPwmData[4], CServoPwmData[5]);                                            //同时，第二组腿落地
//                SixthLeg_Control(CServoPwmData[15], CServoPwmData[16], CServoPwmData[17]);
//                break;

//        case 3: FontRigtLeg_Control(gSServoPwmData[0], gSServoPwmData[1], gSServoPwmData[2]);                                             //第三步：第一组腿向后滑动
//                FifthLeg_Control(gSServoPwmData[12], gSServoPwmData[13], gSServoPwmData[14]);
//                FontLeftLeg_Control(gSServoPwmData[6], gSServoPwmData[7], gSServoPwmData[8]);

//                BackLeftLeg_Control(CServoPwmData[27], gSServoPwmData[10] + ROBOT_LEG_LIFT, gSServoPwmData[11] + ROBOT_FET_LIFT);
//                BackRigtLeg_Control(CServoPwmData[21], gSServoPwmData[4] + ROBOT_LEG_LIFT, gSServoPwmData[5] + ROBOT_FET_LIFT);        //同时，第二组腿抬腿
//                SixthLeg_Control(CServoPwmData[33], gSServoPwmData[16] + ROBOT_LEG_LIFT, gSServoPwmData[17] + ROBOT_FET_LIFT);
//                break; 

//        case 4: FontRigtLeg_Control(CServoPwmData[0], CServoPwmData[1], CServoPwmData[2]);                                             //第四步：第一组腿继续向后滑动
//                FifthLeg_Control(CServoPwmData[12], CServoPwmData[13], CServoPwmData[14]);
//                FontLeftLeg_Control(CServoPwmData[6], CServoPwmData[7], CServoPwmData[8]);

//                BackLeftLeg_Control(CServoPwmData[27], CServoPwmData[28], CServoPwmData[29]);
//                BackRigtLeg_Control(CServoPwmData[21], CServoPwmData[22], CServoPwmData[23]);                                         //同时，第二组腿继续向后滑动
//                SixthLeg_Control(CServoPwmData[33], CServoPwmData[34], CServoPwmData[35]);
//                break;

//        default:break;
//    }
}
/*********************************************************************/
/************************机器人顺时针转圈函数**************************/
static void DogRobot_ClckWis(void)
{
//    static uint8_t          Flag = 0;                                                                                                        //Flag标记用于控制机器人动作组的执行顺序

//    Flag++;

//    StartMoveFlag = 0;

//    if(Flag > 4)
//    {
//        Flag = 1;
//    }

//    switch(Flag)
//    {
//        case 1: //CircleCoordinate_Calculate(10);

//                FontRigtLeg_Control(CServoPwmData[0], gSServoPwmData[1] + ROBOT_LEG_LIFT, gSServoPwmData[2] + ROBOT_FET_LIFT);           //第一步：第一组腿抬脚前移
//                FifthLeg_Control(CServoPwmData[12], gSServoPwmData[13] + ROBOT_LEG_LIFT, gSServoPwmData[14] + ROBOT_FET_LIFT);
//                FontLeftLeg_Control(CServoPwmData[6], gSServoPwmData[7] + ROBOT_LEG_LIFT, gSServoPwmData[8] + ROBOT_FET_LIFT);

//                BackLeftLeg_Control(gSServoPwmData[9], gSServoPwmData[10], gSServoPwmData[11]);
//                BackRigtLeg_Control(gSServoPwmData[3], gSServoPwmData[4], gSServoPwmData[5]);                                             //同时，第二组腿向后滑动
//                SixthLeg_Control(gSServoPwmData[15], gSServoPwmData[16], gSServoPwmData[17]);
//                break;

//        case 2: FontRigtLeg_Control(CServoPwmData[0], CServoPwmData[1], CServoPwmData[2]);                                             //第二步：第一组腿落地
//                FifthLeg_Control(CServoPwmData[12], CServoPwmData[13], CServoPwmData[14]);
//                FontLeftLeg_Control(CServoPwmData[6], CServoPwmData[7], CServoPwmData[8]);

//                BackLeftLeg_Control(CServoPwmData[27], CServoPwmData[28], CServoPwmData[29]);
//                BackRigtLeg_Control(CServoPwmData[21], CServoPwmData[22], CServoPwmData[23]);                                          //同时，第二组腿继续向后滑动
//                SixthLeg_Control(CServoPwmData[33], CServoPwmData[34], CServoPwmData[35]);
//                break;

//        case 3: FontRigtLeg_Control(gSServoPwmData[0], gSServoPwmData[1], gSServoPwmData[2]);                                             //第三步：第一组腿向后滑动
//                FifthLeg_Control(gSServoPwmData[12], gSServoPwmData[13], gSServoPwmData[14]);
//                FontLeftLeg_Control(gSServoPwmData[6], gSServoPwmData[7], gSServoPwmData[8]);

//                BackLeftLeg_Control(CServoPwmData[9], gSServoPwmData[10] + ROBOT_LEG_LIFT, gSServoPwmData[11] + ROBOT_FET_LIFT);
//                BackRigtLeg_Control(CServoPwmData[3], gSServoPwmData[4] + ROBOT_LEG_LIFT, gSServoPwmData[5] + ROBOT_FET_LIFT);           //同时，第二组腿抬腿
//                SixthLeg_Control(CServoPwmData[15], gSServoPwmData[16] + ROBOT_LEG_LIFT, gSServoPwmData[17] + ROBOT_FET_LIFT);
//                break; 

//        case 4: FontRigtLeg_Control(CServoPwmData[18], CServoPwmData[19], CServoPwmData[20]);                                          //第四步：第一组腿继续向后滑动
//                FifthLeg_Control(CServoPwmData[30], CServoPwmData[31], CServoPwmData[32]);
//                FontLeftLeg_Control(CServoPwmData[24], CServoPwmData[25], CServoPwmData[26]);

//                BackLeftLeg_Control(CServoPwmData[9], CServoPwmData[10], CServoPwmData[11]);
//                BackRigtLeg_Control(CServoPwmData[3], CServoPwmData[4], CServoPwmData[5]);                                             //同时，第二组腿落地
//                SixthLeg_Control(CServoPwmData[15], CServoPwmData[16], CServoPwmData[17]);
//                break;

//        default:break;
//    }
}
/********************************************************************/
/************************机器人遥控函数*******************************/
void RobotInstruct_Control(void)
{
    static uint8_t          TransfrmFlag = 1;

#ifdef SPEEH_CONTROL
    static uint8_t          RollLockFlag = 0;
    static uint8_t          TransformHexFinsh = 1;
    static uint8_t          TransformBoxFinsh = 0;
    static uint8_t          TransformBoxOrder = 0;
    static uint8_t          TransformHexOrder = 0;
    static uint8_t          ForwardSpeechFlag = 0;
    static uint8_t          BackwrdSpeechFlag = 0;
    static uint8_t          ClckWisSpeechFlag = 0;
    static uint8_t          AticlckSpeechFlag = 0;
    static uint8_t          TrasfrmSpeechFlag = 0;
    static uint8_t          FrdRollSpeechFlag = 0;
    static uint8_t          BakRollSpeechFlag = 0;

    switch(SpeechReturnData)
    {
        case 2:     if(TransfrmFlag)
                    {
                        DogRobot_Forward();
                    }
                    break;

        case 3:     if(TransfrmFlag)
                    {
                        DogRobot_Backwrd();
                    }
                    break;

        case 5:     if(TransfrmFlag)
                    {
                        DogRobot_AtiClck();
                    }
                    break;

        case 6:     if(TransfrmFlag)
                    {
                        DogRobot_ClckWis();
                    }
                    break;

        default:    break;
    }
#endif

    switch(RxBuffer[0])
    {
        case 'W':   if(TransfrmFlag)
                    {
                        #ifdef SPEEH_CONTROL
                        if(!ForwardSpeechFlag)
                        {
                            ForwardSpeechFlag = 1;
                            ForwardSpeech_Play();

                            ServoRunTime_Set(RobotCrepSpeed);
                        }
                        #endif

                        DogRobot_Forward();
                    }
                    break;

        case 'S':   if(TransfrmFlag)
                    {
                        #ifdef SPEEH_CONTROL
                        if(!BackwrdSpeechFlag)
                        {
                            BackwrdSpeechFlag = 1;
                            BackwrdSpeech_Play();
                            ServoRunTime_Set(RobotCrepSpeed);
                        }
                        #endif

                        DogRobot_Backwrd();
                    }
                    break;

        case 'A':   if(TransfrmFlag)
                    {
                        #ifdef SPEEH_CONTROL
                        if(!AticlckSpeechFlag)
                        {
                            AticlckSpeechFlag = 1;
                            AtiClckSpeech_Play();

                            ServoRunTime_Set(RobotCrepSpeed);
                        }
                        #endif

                        DogRobot_AtiClck();
                    }
                    break;

        case 'D':   if(TransfrmFlag)
                    {
                        #ifdef SPEEH_CONTROL
                        if(!ClckWisSpeechFlag)
                        {
                            ClckWisSpeechFlag = 1;
                            ClckWisSpeech_Play();

                            ServoRunTime_Set(RobotCrepSpeed);
                        }
                        #endif

                        DogRobot_ClckWis();
                    }
                    break;


        default:    break;
    }
}
/********************************************************************/

