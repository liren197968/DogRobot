#include "fsl_ctimer.h"

#include "servo.h"
#include "PCA9685.h"


#define BUS_CLK_FREQ        CLOCK_GetFreq(kCLOCK_BusClk)


int32_t             gPwmExpetVal[ROBOT_SERVO_MAX_NUM] = {0};
static int32_t      gPwmIncreVal[ROBOT_SERVO_MAX_NUM] = {0};
static int32_t      gPwmPreseVal[ROBOT_SERVO_MAX_NUM] = {0};
static int32_t      gPwmLastdVal[ROBOT_SERVO_MAX_NUM] = {0};

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

    for(i = 0U; i < ROBOT_SERVO_MAX_NUM; i++)
    {
        Pca9685OutPwm(PWM_ADDRESS_L, i, 0U, gPwmPreseVal[i]);

        if(i > 15U)
        {
            Pca9685OutPwm(PWM_ADDRESS_H, i - 16U, 0U, gPwmPreseVal[i]);
        }
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
        RefreshTimes = 0U;

        ServoRecodValue();                                  //记录当前PWM数据
        ServoCalcuIncre();                                  //加载将要执行的PWM数据计算增量数据
    }
}
/**********************************************************************************/
/***************************舵机定时器中断函数**************************************/
static void CTimer0IrqHandler(uint32_t flags)
{
    ServoContrlExcut();
}
/**********************************************************************************/
/***************************舵机定时器初始化函数************************************/
void ServoTimerInit(void)
{
    ctimer_config_t     CTimerConfig;
    ctimer_callback_t   CTimerCallbackTable = CTimer0IrqHandler;

    SYSCON->ASYNCAPBCTRL = 1;

    CLOCK_AttachClk(kFRO12M_to_ASYNC_APB);

    CTIMER_GetDefaultConfig(&CTimerConfig);
    CTIMER_Init(CTIMER0, &CTimerConfig);

    gMatchConfig.enableCounterReset = true;
    gMatchConfig.enableCounterStop = false;
    gMatchConfig.matchValue = BUS_CLK_FREQ / 2;
    gMatchConfig.outControl = kCTIMER_Output_NoAction;
    gMatchConfig.outPinInitState = false;
    gMatchConfig.enableInterrupt = true;

    CTIMER_RegisterCallBack(CTIMER0, &CTimerCallbackTable, kCTIMER_MultipleCallback);
    CTIMER_SetupMatch(CTIMER0, kCTIMER_Match_0, &gMatchConfig);
    CTIMER_StartTimer(CTIMER0);
}
/**********************************************************************************/
/***************************舵机转速设置函数****************************************/
void ServoSpeedSet(uint16_t ServoRunTimeMs)
{
    uint16_t        CTimerInterval = 0;

    CTimerInterval = ServoRunTimeMs / SERVO_SPEED_DIV_STP;

    gMatchConfig.matchValue = CTimerInterval * 1000;

    CTIMER_SetupMatch(CTIMER0, kCTIMER_Match_0, &gMatchConfig);
}
/**********************************************************************************/
