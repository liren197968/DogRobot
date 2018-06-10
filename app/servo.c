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
/***************************���ת���������㺯��************************************/
static void ServoCalcuIncre(void)
{
    uint8_t         i = 0U;

    for(i = 0U; i < ROBOT_SERVO_MAX_NUM; i++)
    {
        gPwmIncreVal[i] = (gPwmExpetVal[i] - gPwmLastdVal[i]) / SERVO_SPEED_DIV_STP;
    }
}
/**********************************************************************************/
/************************���µ�ǰgPwmExpetVal���ݺ���*******************************/
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
/***************************��¼�ϴ�PWM����****************************************/
static void ServoRecodValue(void)
{
    uint8_t         i = 0U;

    for(i = 0U; i < ROBOT_SERVO_MAX_NUM; i++)
    {
        gPwmLastdVal[i] = gPwmPreseVal[i];
    }
}
/**********************************************************************************/
/*******************************ִ��PWM����****************************************/
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
/*******************************������ƺ���***************************************/
static void ServoContrlExcut(void)
{
    static uint8_t  RefreshTimes = 0U;                      //��¼PWM���ݸ��´���

    RefreshTimes ++;

    ServoExcutValue();

    if(RefreshTimes <= SERVO_SPEED_DIV_STP)
    {
        ServoRefrhValue();
    }
    else                                                    //���ݸ��´����ﵽSERVO_SPEED_DIV_STP�Σ��ݸ������
    {
        RefreshTimes = 0;

        ServoRecodValue();                                  //��¼��ǰPWM����

        RobotInstruct_Control();

        ServoCalcuIncre();                                  //���ؽ�Ҫִ�е�PWM���ݼ�����������
    }
}
/**********************************************************************************/
/***************************�����ʱ���жϺ���**************************************/
void CTimer0IrqHandler(uint32_t flags)
{
    ServoContrlExcut();

    led_toggle(5);

    CTIMER_SetupMatch(CTIMER0, kCTIMER_Match_0, &gMatchConfig);
}
/**********************************************************************************/
/***************************�����ʱ����ʼ������************************************/
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
/***************************���ת�����ú���****************************************/
void ServoSpeedSet(uint16_t ServoRunTimeMs)
{
    uint16_t        CTimerInterval = 0;

    CTimerInterval = ServoRunTimeMs / SERVO_SPEED_DIV_STP;

    gMatchConfig.matchValue = CTimerInterval *96000U;
}
/**********************************************************************************/
