#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

#include <stdio.h>
#include <stdint.h>

/*******************************************************************************/
#define BLUETOOTH_PERIPHERAL                            ((USART_Type *)FLEXCOMM3)
/* Definition of the clock source frequency */
#define BLUETOOTH_CLOCK_SOURCE                          CLOCK_GetFreq(kCLOCK_Flexcomm3)
/* USART3 interrupt vector ID (number). */
#define BLUETOOTH_FLEXCOMM_IRQN                         FLEXCOMM3_IRQn
/* USART3 interrupt handler identifier. */
#define BLUETOOTH_FLEXCOMM_IRQHANDLER                   FLEXCOMM3_IRQHandler

#define BLUETOOTH_USART_BAUDRATE                        115200

#define TX_LENH                                         10
#define RX_LENH                                         1

void Bluetooth_Init(void);
void BluetoothData_Send(uint8_t *Buff);

extern uint8_t TxBuffer[TX_LENH];
extern uint8_t RxBuffer[RX_LENH];

#endif


