#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

#include <stdio.h>
#include <stdint.h>

/*******************************************************************************/
#define BLUETOOTH_USART                               USART3
#define BLUETOOTH_USART_CLK_SRC                       kCLOCK_Flexcomm3
#define BLUETOOTH_USART_CLK_FREQ                      CLOCK_GetFreq(kCLOCK_Flexcomm3)
#define BLUETOOTH_USART_IRQHandler                    FLEXCOMM3_IRQHandler
#define BLUETOOTH_USART_IRQn                          FLEXCOMM3_IRQn

#define BLUETOOTH_USART_CLK_ATTACH                    kFRO12M_to_FLEXCOMM3
#define BLUETOOTH_USART_RST                           kFC3_RST_SHIFT_RSTn

#define BLUETOOTH_USART_BAUDRATE                      115200

#define PIN4_IDX                                      4U   /*!< Pin number for pin 4 in a port 1 */
#define PIN9_IDX                                      9U   /*!< Pin number for pin 9 in a port 1 */
#define PORT1_IDX                                     1U   /*!< Port index */

#define TX_LENH                                       10
#define RX_LENH                                       1


void Bluetooth_Init(void);
void BluetoothData_Send(uint8_t *buff, uint16_t len);

extern uint8_t TxBuffer[TX_LENH];
extern uint8_t RxBuffer[RX_LENH];

#endif


