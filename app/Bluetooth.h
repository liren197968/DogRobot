#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

#include <stdio.h>
#include <stdint.h>

#define TX_LENH                           10
#define RX_LENH                           1


void Bluetooth_Init(void);
void BluetoothData_Send(uint8_t *buff, uint16_t len);

extern uint8_t TxBuffer[TX_LENH];
extern uint8_t RxBuffer[RX_LENH];

#endif


