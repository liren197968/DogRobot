#include "pin_mux.h"

#include "fsl_common.h"
#include "fsl_iocon.h"
#include "fsl_usart.h"

#include "Bluetooth.h"

uint8_t TxBuffer[TX_LENH] = "12345";
uint8_t RxBuffer[RX_LENH] = {0};

/****************************************************************************
* 名    称: USART_GPIOInit
* 功    能：UART3初始化
* 返回参数：无
* 说    明： 
****************************************************************************/
const usart_config_t Usart3Config = {
    .baudRate_Bps = BLUETOOTH_USART_BAUDRATE,
    .parityMode = kUSART_ParityDisabled,
    .stopBitCount = kUSART_OneStopBit,
    .bitCountPerChar = kUSART_8BitsPerChar,
    .loopback = false,
    .txWatermark = kUSART_TxFifo0,
    .rxWatermark = kUSART_RxFifo1,
    .enableRx = true,
    .enableTx = true
};

static void BluetoothUsart_Init(void)
{
    /* attach 12 MHz clock to FLEXCOMM3  */
    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM3);

    /* Reset FLEXCOMM device */
    RESET_PeripheralReset(kFC3_RST_SHIFT_RSTn);
    USART_Init(BLUETOOTH_PERIPHERAL, &Usart3Config, BLUETOOTH_CLOCK_SOURCE);
    USART_EnableInterrupts(BLUETOOTH_PERIPHERAL, kUSART_RxLevelInterruptEnable);
    /* Enable interrupt FLEXCOMM3_IRQn request in the NVIC */
    EnableIRQ(BLUETOOTH_FLEXCOMM_IRQN);
}

void BLUETOOTH_FLEXCOMM_IRQHANDLER(void)
{
    /* If new data arrived. */
    if ((kUSART_RxFifoNotEmptyFlag) & USART_GetStatusFlags(BLUETOOTH_PERIPHERAL))
    {
        RxBuffer[0] = USART_ReadByte(BLUETOOTH_PERIPHERAL);
    }
}

void BluetoothData_Send(uint8_t *Buff)
{
    USART_WriteBlocking(BLUETOOTH_PERIPHERAL, Buff, sizeof(Buff) / sizeof(Buff[0]));
}

void Bluetooth_Init(void)
{
    BluetoothUsart_Init();

    BluetoothData_Send(TxBuffer);
}
