#include "lcdutil.h"

/*
void SPI_MasterInit(void) {
	lcd_Data_DDR |= (1<<lcd_DB4_bit)|(1<<lcd_DB5_bit)|(1<<lcd_DB6_bit)|(1<<lcd_DB7_bit);
	lcd_Ctrl_DDR |= (1<<lcd_E_bit)|(1<<lcd_RS_bit);
}*/

void LCD_init(void) {
	lcd_Data_DDR |= (1<<lcd_DB4_bit)|(1<<lcd_DB5_bit)|(1<<lcd_DB6_bit)|(1<<lcd_DB7_bit);
	lcd_Ctrl_DDR |= (1<<lcd_E_bit)|(1<<lcd_RS_bit);
	CLEARBIT(lcd_Ctrl_Port,lcd_RS_bit);
	_delay_ms(100);
	LCD_send_nibble((0x38>>4));
	_delay_us(4200);
	LCD_send_nibble((0x38>>4));
	_delay_us(200);
	LCD_send_nibble((0x38>>4));
	LCD_send_nibble((0x28>>4));
	LCD_send_byte(0x28);
	LCD_send_byte(0x0C);
	LCD_send_byte(0x01);
	_delay_ms(2);
	LCD_send_byte(0x06);
	_delay_ms(2);
	/*
	char happychar[] = {0x00, 0x00, 0x0A, 0x00, 0x11, 0x0E, 0x00, 0x00};
	char angrychar[] = {0x11, 0x0A, 0x04, 0x0A, 0x00, 0x0E, 0x11, 0x00};
	char hatchar[] = {0x0E, 0x1F, 0x00, 0x0A, 0x00, 0x0E, 0x1F, 0x11};
	char demonchar[] = {0x11, 0x1B, 0x00, 0x0A, 0x00, 0x0E, 0x00, 0x0E};
	char batmanchar[] = {0x11, 0x1B, 0x1F, 0x15, 0x1F, 0x11, 0x0E, 0x1F};
	DDRD |= (1<<PORTD0);
	DDRB |= (1<<PORTB4);
	CLEARBIT(PORTB,4);
	LCD_send_byte(0x39);
	LCD_send_byte(0x1C);
	LCD_send_byte(0x52);
	LCD_send_byte(0x69);
	LCD_send_byte(0x74);
	LCD_send_byte(0x38);
	LCD_send_byte(0x0C);
	LCD_send_byte(0x01);
	LCD_send_byte(0x06);
	
	LCD_send_byte(0x40);
	SETBIT(PORTD, 0);
	for (int i=0; i<8; i++) {
		LCD_send_byte(happychar[i]);
	}
	for (int i=0; i<8; i++) {
		LCD_send_byte(angrychar[i]);
	}
	for (int i=0; i<8; i++) {
		LCD_send_byte(hatchar[i]);
	}
	for (int i=0; i<8; i++) {
		LCD_send_byte(demonchar[i]);
	}
	for (int i=0; i<8; i++) {
		LCD_send_byte(batmanchar[i]);
	}
	CLEARBIT(PORTD, 0);*/
	
}

int LCD_send_char(char ch, FILE *stream) {
	SETBIT(lcd_Ctrl_Port, lcd_RS_bit);
	LCD_send_byte(ch);
	CLEARBIT(lcd_Ctrl_Port, lcd_RS_bit);
	return 0;
}

int LCD_send_byte(char ch) {
	uint8_t high_nibble = (ch>>4);
	uint8_t low_nibble = ch & 0x0F;
	
	LCD_send_nibble(high_nibble);
	LCD_send_nibble(low_nibble);
	return 0;
}

int LCD_send_nibble(char nb) {
	lcd_Data_Port &= 0xF0;
	SETBIT(lcd_Ctrl_Port,lcd_E_bit);
	lcd_Data_Port |= nb;
	CLEARBIT(lcd_Ctrl_Port,lcd_E_bit);
	_delay_us(50);
	lcd_Data_Port &= 0xF0;
	return 0;
}

int LCD_clear(void) { 
	LCD_send_byte(0x01);
	_delay_ms(2);
	return 0;
}

int LCD_cursor_pos(uint8_t pos) {
	LCD_send_byte((pos | 0x80));
	return 0;
}
/*
uint8_t SPI_MasterTransmit(char cData) {
	uint16_t counter = 0;
	// Start transmission
	SPDR = cData;
	// Wait for transmission complete
	while(!(SPSR & (1<<SPIF))) {
		if(counter++ > 1000) {
			printf("SPI Timeout\r\n");
			return 1;
		}
	};
	return 0;
}

uint8_t SPI_MasterTransmit(char cData) {
	uint16_t counter = 0;
	// Start transmission
	SPDR = cData;
	// Wait for transmission complete
	while(!(SPSR & (1<<SPIF))) {
		if(counter++ > 1000) {
			printf("SPI Timeout\r\n");
			return 1;
		}
	};
	return 0;
}
*/