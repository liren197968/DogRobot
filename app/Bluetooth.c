#include "Bluetooth.h"

uint8_t TxBuffer[TX_LENH] = {0};
uint8_t RxBuffer[RX_LENH] = {0};

/****************************************************************************
* ��    ��: USART_GPIOInit
* ��    �ܣ�UART5��ʼ��
* ���ز�������
* ˵    ���� 
****************************************************************************/
static void UART5_GPIOInit(void)
{   //GPIO�˿�����
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
