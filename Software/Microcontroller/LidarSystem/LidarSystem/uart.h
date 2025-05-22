/*
 * uart.h
 *
 * Created: 2023-11-07 22:22:36
 *  Author: Filip Andersson
 */ 


#ifndef UART_H_
#define UART_H_

#include "main.h"

void uart1_init(uint8_t baud);
int uart1_send_char(char ch, FILE *stream);

void UART_RX_interrupt(void);
void UART_TX_interrupt(void);
unsigned char ReceiveByte(void);
int TransmitByte(char ch, FILE *stream);
unsigned char DataInReceiveBuffer(void);



#endif /* UART_H_ */