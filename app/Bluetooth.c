#include "pin_mux.h"

#include "fsl_common.h"
#include "fsl_iocon.h"
#include "fsl_usart.h"

#include "Bluetooth.h"

uint8_t TxBuffer[TX_LENH] = {0};
uint8_t RxBuffer[RX_LENH] = {0};

/****************************************************************************
* 名    称: USART_GPIOInit
* 功    能：UART3初始化
* 返回参数：无
* 说    明： 
****************************************************************************/
static void BluetoothGpio_Init(void)
{
     usart_config_t         Config;
     uint8_t                TestBuf[4] = "123";

    /* enable clock for IOCON */
    CLOCK_EnableClock(kCLOCK_Iocon);

    /* attach 12 MHz clock to FLEXCOMM3  */
    CLOCK_AttachClk(BLUETOOTH_USART_CLK_ATTACH);

    /* USART3 RX/TX pin */
    IOCON_PinMuxSet(IOCON, PORT1_IDX, PIN4_IDX, IOCON_MODE_INACT | IOCON_FUNC3 | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF);    //RX
    IOCON_PinMuxSet(IOCON, PORT1_IDX, PIN9_IDX, IOCON_MODE_INACT | IOCON_FUNC3 | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF);    //TX

    /* reset FLEXCOMM for USART3 */
    RESET_PeripheralReset(BLUETOOTH_USART_RST);

    USART_GetDefaultConfig(&Config);
    Config.baudRate_Bps = BLUETOOTH_USART_BAUDRATE;
    Config.enableTx = true;
    Config.enableRx = true;

    USART_Init(BLUETOOTH_USART, &Config, BLUETOOTH_USART_CLK_FREQ);

    USART_WriteBlocking(BLUETOOTH_USART, TestBuf, 4);

    /* Enable RX interrupt. */
    USART_EnableInterrupts(BLUETOOTH_USART, kUSART_RxLevelInterruptEnable | kUSART_RxErrorInterruptEnable);

    EnableIRQ(BLUETOOTH_USART_IRQn);
}

void BLUETOOTH_USART_IRQHandler(void)
{
    /* If new data arrived. */
    if ((kUSART_RxFifoNotEmptyFlag | kUSART_RxError) & USART_GetStatusFlags(BLUETOOTH_USART))
    {
        RxBuffer[0] = USART_ReadByte(BLUETOOTH_USART);
    }

}

void BluetoothData_Send(uint8_t *Buff, uint16_t Len)
{

}


void Bluetooth_Init(void)
{
    BluetoothGpio_Init();
}
