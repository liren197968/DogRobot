#include "fsl_ctimer.h"

#include "app_led.h"

#include "Servo.h"
#include "PCA9685.h"
#include "RobotControl.h"

//int16_t             gPwmExpetVal[ROBOT_SERVO_MAX_NUM] = {FOR_RIG_ARM_CENTER - 60, FOR_RIG_LEG_CENTER, FOR_RIG_FET_CENTER + 60,
//                                                         BAK_RIG_ARM_CENTER - 60, BAK_RIG_LEG_CENTER - 60, BAK_RIG_FET_CENTER,
//                                                         TAL_MID_SWG_CENTER, ROBOT_SERVO_CENTER, ROBOT_SERVO_CENTER,
//                                                         FOR_LEF_ARM_CENTER + 60, FOR_LEF_LEG_CENTER - 5, FOR_LEF_FET_CENTER - 70,
//                                                         BAK_LEF_ARM_CENTER + 60, BAK_LEF_LEG_CENTER + 60, BAK_LEF_FET_CENTER,
//                                                         HED_MID_NEK_CENTER, HED_RIG_EAR_CENTER, HED_LEF_EAR_CENTER};

int16_t             gPwmExpetVal[ROBOT_SERVO_MAX_NUM] = {FOR_RIG_ARM_CENTER - 20, FOR_RIG_LEG_CENTER + 25, FOR_RIG_FET_CENTER + 45,
                                                         BAK_RIG_ARM_CENTER - 20, BAK_RIG_LEG_CENTER - 60, BAK_RIG_FET_CENTER - 30,
                                                         TAL_MID_SWG_CENTER, ROBOT_SERVO_CENTER, ROBOT_SERVO_CENTER,
                                                         FOR_LEF_ARM_CENTER + 20, FOR_LEF_LEG_CENTER - 25, FOR_LEF_FET_CENTER - 45,
                                                         BAK_LEF_ARM_CENTER + 20, BAK_LEF_LEG_CENTER + 60, BAK_LEF_FET_CENTER + 30,
                                                         HED_MID_NEK_CENTER, HED_RIG_EAR_CENTER, HED_LEF_EAR_CENTER};

static int16_t      gPwmIncreVal[ROBOT_SERVO_MAX_NUM] = {0};
static int16_t      gPwmPreseVal[ROBOT_SERVO_MAX_NUM] = {0};
static int16_t      gPwmLastdVal[ROBOT_SERVO_MAX_NUM] = {0};

static ctimer_match_config_t gMatchConfig;
/***************************舵机转动增量计算函数************************************/
static void ServoCalcuIncre(void)
{
    uint8_t         i = 0U;

    for(i = 0U; i < ROBOT_SERVO_MAX_NUM; i++)
    {
        gPwmIncreVal[i] = (gPwmExpetVal[i] - gPwmLastdVal[i]) / SERVO_SPEED_DIV_STP;
    }
}
/**********************************************************************************/
/************************更新当前gPwmExpetVal数据函数*******************************/
static void ServoRefrhValue(void)
{
    uint8_t         i = 0U;
    static uint8_t  CountTimes = 0U;

    CountTimes++;

    if(CountTimes < SERVO_SPEED_DIV_STP)
    {
        for(i = 0U; i < ROBOT_SERVO_MAX_NUM; i++)
        {
            gPwmPreseVal[i] = gPwmPreseVal[i] + gPwmIncreVal[i];
        }
    }
    else
    {
        CountTimes = 0U;

        for(i = 0U; i < ROBOT_SERVO_MAX_NUM; i++)
        {
            gPwmPreseVal[i] = gPwmPreseVal[i] + (gPwmExpetVal[i]- gPwmLastdVal[i]
                                    - (SERVO_SPEED_DIV_STP - 1U) * gPwmIncreVal[i]);
        }
    }
}
/**********************************************************************************/
/***************************记录上次PWM数据****************************************/
static void ServoRecodValue(void)
{
    uint8_t         i = 0U;

    for(i = 0U; i < ROBOT_SERVO_MAX_NUM; i++)
    {
        gPwmLastdVal[i] = gPwmPreseVal[i];
    }
}
/**********************************************************************************/
/*******************************执行PWM数据****************************************/
void ServoExcutValue(void)
{
    uint8_t         i = 0U;

    for(i = 0U; i < 15; i++)
    {
        Pca9685OutPwm(PWM_ADDRESS_L, i, 0U, gPwmPreseVal[i]);

//        if(i > 15U)
//        {
//            Pca9685OutPwm(PWM_ADDRESS_H, i - 16U, 0U, gPwmPreseVal[i]);
//        }
    }
}
/**********************************************************************************/
/*******************************舵机控制函数***************************************/
static void ServoContrlExcut(void)
{
    static uint8_t  RefreshTimes = 0U;                      //记录PWM数据更新次数

    RefreshTimes ++;

    ServoExcutValue();

    if(RefreshTimes <= SERVO_SPEED_DIV_STP)
    {
        ServoRefrhValue();
    }
    else                                                    //数据更新次数达到SERVO_SPEED_DIV_STP次，据更新完毕
    {
        RefreshTimes = 0;

        ServoRecodValue();                                  //记录当前PWM数据

        RobotInstruct_Control();

        ServoCalcuIncre();                                  //加载将要执行的PWM数据计算增量数据
    }
}
/**********************************************************************************/
/***************************舵机定时器中断函数**************************************/
void CTimer0IrqHandler(uint32_t flags)
{
    ServoContrlExcut();

    led_toggle(5);

    CTIMER_SetupMatch(CTIMER0, kCTIMER_Match_0, &gMatchConfig);
}
/**********************************************************************************/
/***************************舵机定时器初始化函数************************************/
void ServoTimerInit(void)
{
    ctimer_config_t     CTimerConfig;
    ctimer_callback_t   CTimerCallback = CTimer0IrqHandler;

    SYSCON->ASYNCAPBCTRL = 1;

    CLOCK_AttachClk(kFRO12M_to_ASYNC_APB);

    CTIMER_GetDefaultConfig(&CTimerConfig);
    CTIMER_Init(CTIMER0, &CTimerConfig);

    gMatchConfig.enableCounterReset = true;
    gMatchConfig.enableCounterStop = false;

    gMatchConfig.matchValue = 480000U;
    gMatchConfig.outControl = kCTIMER_Output_NoAction;
    gMatchConfig.outPinInitState = false;
    gMatchConfig.enableInterrupt = true;

    CTIMER_RegisterCallBack(CTIMER0, &CTimerCallback, kCTIMER_MultipleCallback);
    CTIMER_SetupMatch(CTIMER0, kCTIMER_Match_0, &gMatchConfig);
    CTIMER_StartTimer(CTIMER0);
}
/**********************************************************************************/
/***************************舵机转速设置函数****************************************/
void ServoSpeedSet(uint16_t ServoRunTimeMs)
{
    uint16_t        CTimerInterval = 0;

    CTimerInterval = ServoRunTimeMs / SERVO_SPEED_DIV_STP;

    gMatchConfig.matchValue = CTimerInterval *96000U;
}
/**********************************************************************************/
