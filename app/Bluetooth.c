#include "Bluetooth.h"

uint8_t TxBuffer[TX_LENH] = {0};
uint8_t RxBuffer[RX_LENH] = {0};

/****************************************************************************
* 名    称: USART_GPIOInit
* 功    能：UART5初始化
* 返回参数：无
* 说    明： 
****************************************************************************/
static void UART5_GPIOInit(void)
{   //GPIO端口设置
}

static void UART5_DMAInit(void)
{

}

void BluetoothData_Send(uint8_t *Buff, uint16_t Len)
{

}

void DMA1_Stream7_IRQHandler(void)
{

}

void DMA1_Stream0_IRQHandler(void)
{

}

void Bluetooth_Init(void)
{
    UART5_GPIOInit();
    UART5_DMAInit();
}
