#ifndef LCDUTIL_H
#define LCDUTIL_H

#include "main.h"

/* Data pins */
#define lcd_DB7_bit			PORTC3
#define lcd_DB6_bit			PORTC2
#define lcd_DB5_bit			PORTC1
#define lcd_DB4_bit			PORTC0
#define lcd_Data_DDR		DDRC
#define lcd_Data_Port		PORTC

/* Control pins */
#define lcd_E_bit			PORTB0 //Starts data read/write
#define lcd_RS_bit			PORTB1 //Register Select
#define lcd_Ctrl_DDR		DDRB
#define lcd_Ctrl_Port		PORTB

//void SPI_MasterInit(void);
void LCD_init(void);
int LCD_send_char(char ch, FILE *stream);

//uint8_t SPI_MasterTransmit(char cData);
int LCD_send_byte(char ch);
int LCD_send_nibble(char nb);
int LCD_clear(void);
int LCD_cursor_pos(uint8_t pos);

#endif
