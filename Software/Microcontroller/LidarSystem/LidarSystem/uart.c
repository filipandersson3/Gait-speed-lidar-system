/*
 * uart.c
 *
 * Created: 2023-11-07 22:22:21
 *  Author: Filip Andersson
 */ 
#include "uart.h"

/* UART Buffer Defines */
#define UART_RX_BUFFER_SIZE 128 /* 1,2,4,8,16,32,64,128 or 256 bytes */
#define UART_RX_BUFFER_MASK ( UART_RX_BUFFER_SIZE - 1 )
#if ( UART_RX_BUFFER_SIZE & UART_RX_BUFFER_MASK )
#error RX buffer is not a power of 2
#endif

#define UART_TX_BUFFER_SIZE 128 /* 1,2,4,8,16,32,64,128 or 256 bytes */
#define UART_TX_BUFFER_MASK ( UART_TX_BUFFER_SIZE - 1 )
#if ( UART_TX_BUFFER_SIZE & UART_TX_BUFFER_MASK )
#error TX buffer is not a power of 2
#endif

/* Static Variables */
static unsigned char UART_RxBuf[UART_RX_BUFFER_SIZE];
static volatile unsigned char UART_RxHead;
static volatile unsigned char UART_RxTail;
static unsigned char UART_TxBuf[UART_TX_BUFFER_SIZE];
static volatile unsigned char UART_TxHead;
static volatile unsigned char UART_TxTail;

void uart1_init(uint8_t baud) {
	SETBIT(DDRD, 3);
	UCSR0A = 0x00;
	UCSR0B = (1<<RXCIE0) | (1<<UDRIE0) | (1<<RXEN0) | (1<<TXEN0); //enable TX&RX. RX and data register empty interrupt
	UCSR0C = (1<<UCSZ01) | (1<<UCSZ00); //char size 8-bit
	UBRR0 = baud;
	return;
}

int uart1_send_char(char ch, FILE *stream) {
	uint16_t counter = 0;
	while ((UCSR0A & (1<<UDRE0)) == 0) {
		if(counter++ > 1000) {
			return 1;
		}
	};
	UDR0 = ch;
	_delay_us(300);
	return 0;
}

/* interrupt handlers */
void UART_RX_interrupt(void) {
	unsigned char data;
	unsigned char tmphead;
	data = UDR0; /* read the received data */
	/* calculate buffer index */
	tmphead = ( UART_RxHead + 1 ) & UART_RX_BUFFER_MASK;
	UART_RxHead = tmphead; /* store new index */
	if ( tmphead == UART_RxTail )
	{
		/* ERROR! Receive buffer overflow */
	}
	UART_RxBuf[tmphead] = data; /* store received data in buffer */
}
void UART_TX_interrupt(void) {
	unsigned char tmptail;
	/* check if all data is transmitted */
	if (UART_TxHead != UART_TxTail) {
		/* calculate buffer index */
		tmptail = (UART_TxTail + 1) & UART_TX_BUFFER_MASK;
		UART_TxTail = tmptail; /* store new index */
		UDR0 = UART_TxBuf[tmptail]; /* start transmission */
	}
	else {
		UCSR0B &= ~(1<<UDRIE0); /* disable UDRE interrupt */
	}
}

/* Read and write functions */

unsigned char ReceiveByte(void) {
	unsigned char tmptail;
	while (UART_RxHead == UART_RxTail) /* wait for incoming data */
	;
	tmptail = (UART_RxTail + 1) & UART_RX_BUFFER_MASK;/* calculate buffer index */
	UART_RxTail = tmptail; /* store new index */
	return UART_RxBuf[tmptail]; /* return data */
}
int TransmitByte(char ch, FILE *stream) {
	unsigned char tmphead;
	uint16_t counter = 0;
	/* calculate buffer index */
	tmphead = (UART_TxHead + 1) & UART_TX_BUFFER_MASK; /* wait for free space in buffer */
	while (tmphead == UART_TxTail) {
		if(counter++ > 1000) {
			return 1;
		}
	}
	UART_TxBuf[tmphead] = (unsigned char)ch; /* store data in buffer */
	UART_TxHead = tmphead; /* store new index */
	UCSR0B |= (1<<UDRIE0); /* enable UDRE interrupt */
	return 0;
}
unsigned char DataInReceiveBuffer(void) {
	return (UART_RxHead != UART_RxTail); /* return 0 (FALSE) if the receive buffer is empty */
}
