#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"

#include "pin_mux.h"

#include "fsl_common.h"
#include "fsl_iocon.h"

#include <stdbool.h>

#include "app_interrupt.h"
#include "app_led.h"
#include "app_adc.h"
#include "app_key.h"
#include "app_dmic.h"
#include "app_spiflash.h"
#include "app_pct2075.h"
#include "app_wm8904.h"
#include "app_usbdmsc.h"
#include "ff.h"
#include "diskio.h"
#include "app_spisd.h"

#include "PCA9685.h"
#include "Servo.h"
#include "Bluetooth.h"
#include "RobotControl.h"

volatile float      fPCTValue;
//static FATFS        g_fileSystem; /* File system object */
const TCHAR         driverNumberBuffer[3U] = {SDSPIDISK + '0', ':', '/'};


int main(void)
{
    uint16_t        wADCValue = 0;
    uint8_t         ret = 0;

    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);

    BOARD_InitPins();
    BOARD_BootClockHSRUN();
    BOARD_InitDebugConsole();

    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 2000);

    PRINTF("\r\n-------------------------------\r\n\r\n");
    PRINTF("hello world.\r\n");
    PRINTF("LPC54110 Sys Clock is %dMhz.\r\n", SystemCoreClock/1000000);
    PRINTF("\r\n-------------------------------\r\n");

    CLOCK_EnableClock(kCLOCK_InputMux);
    CLOCK_EnableClock(kCLOCK_Iocon);
    CLOCK_EnableClock(kCLOCK_Gpio0);
    CLOCK_EnableClock(kCLOCK_Gpio1);

    led_init();
//    key_init();
//    adc_init();
//    dmic_init();

    ret = spiflash_init();
    if(ret == 1)
    {
        led_on(5);
    }
    else
    {
        led_off(5);
    }

//    pct2075_i2c_init();
//    wm8904_i2s_init();
//    usbdmsc_init();

    Pca9685Init();
    ServoTimerInit();
    Bluetooth_Init();

    HalDelayMs(1000);
    ServoSpeedSet(400);

//    RxBuffer[0] = 'W';

    while (1)
    {
//    gPwmExpetVal[0] = 482;

//        wADCValue = adc_read(ADC_CHANNEL_NUM);

//        if(wADCValue != 0xFFFF)
//        {
//            dwLedTime = (wADCValue*2000)/0xFFFF; // ajust the led blinky freqency
//        }

//        if( (key_value(0) == 0) || (key_value(1) == 0) || (key_value(2) == 0) || (key_value(3) == 0) )
//        {
//            led_on(2);
//        }
//        else
//        {
//            led_off(2);
//        }

//        ret = pct2075_i2c_read((float *)&fPCTValue);
//        if(ret == 1)
//        {
//            led_on(4);
//        }
//        else
//        {
//            led_off(4);
//        }

//        if(fPCTValue>=0)
//        {
//            PRINTF("Temperature Value is +%d.%d oC.\r", (int)(fPCTValue*100)/100, (int)(fPCTValue*100)%100);
//        }
//        else
//        {
//            PRINTF("Temperature Value is -%d.%d oC.\r", (int)((0-fPCTValue)*100)/100, (int)((0-fPCTValue)*100)%100);
//        }

//        usbdmsc_proc();
//        __WFI();
    }
}
