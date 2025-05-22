/*
 * fram.c
 *
 * Created: 2025-05-09 15:18:40
 *  Author: Filip Andersson
 */ 

#include "fram.h"

void FRAM_SPI_Init(void) {
	// Set MOSI and SCK output
	SPI_DDR |= (1<<SPI_SCK_bit)|(1<<SPI_MOSI_bit)|(1<<SPI_CS_bit);
	//SPI_PORT |= (1<<SPI_MISO_bit); // pull-up MISO
	// Chip select is active low, set high
	SETBIT(SPI_PORT,SPI_CS_bit);
	// Enable SPI, Master, Mode 3, set clock rate fck/64
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<CPOL)|(1<<CPHA);
}

uint8_t SPI_Transmit(char cData) {
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
	return SPDR;
}

void FRAM_Write8(uint32_t addr, uint8_t data) {
	CLEARBIT(PORTB,PORTB2); //CS
	SPI_Transmit(0x06); //WREN
	SETBIT(PORTB,PORTB2);
	CLEARBIT(PORTB,PORTB2);
	SPI_Transmit(0x02); //WRITE
	SPI_Transmit((addr >> 16) & 0xFF);
	SPI_Transmit((addr >> 8) & 0xFF);
	SPI_Transmit(addr & 0xFF);
	SPI_Transmit(data);
	SETBIT(PORTB,PORTB2);
}

void FRAM_Write16(uint32_t addr, uint16_t data) {
	CLEARBIT(PORTB,PORTB2); //CS
	SPI_Transmit(0x06); //WREN
	SETBIT(PORTB,PORTB2);
	CLEARBIT(PORTB,PORTB2);
	SPI_Transmit(0x02); //WRITE
	SPI_Transmit((addr >> 16) & 0xFF);
	SPI_Transmit((addr >> 8) & 0xFF);
	SPI_Transmit(addr & 0xFF);
	SPI_Transmit((data>>8) & 0xFF);
	SPI_Transmit(data & 0xFF);
	SETBIT(PORTB,PORTB2);
}

void FRAM_Write32(uint32_t addr, uint32_t data) {
	CLEARBIT(PORTB,PORTB2); //CS
	SPI_Transmit(0x06); //WREN
	SETBIT(PORTB,PORTB2);
	CLEARBIT(PORTB,PORTB2);
	SPI_Transmit(0x02); //WRITE
	SPI_Transmit((addr >> 16) & 0xFF);
	SPI_Transmit((addr >> 8) & 0xFF);
	SPI_Transmit(addr & 0xFF);
	SPI_Transmit((data>>24) & 0xFF);
	SPI_Transmit((data>>16) & 0xFF);
	SPI_Transmit((data>>8) & 0xFF);
	SPI_Transmit(data & 0xFF);
	SETBIT(PORTB,PORTB2);
}

uint8_t FRAM_Read8(uint32_t addr) {
	CLEARBIT(PORTB,PORTB2);
	SPI_Transmit(0x03); //READ
	SPI_Transmit((addr >> 16) & 0xFF);
	SPI_Transmit((addr >> 8) & 0xFF);
	SPI_Transmit(addr & 0xFF);
	uint8_t recv = SPI_Transmit(0x00);
	SETBIT(PORTB,PORTB2);
	return recv;
}

uint16_t FRAM_Read16(uint32_t addr) {
	CLEARBIT(PORTB,PORTB2);
	SPI_Transmit(0x03); //READ
	SPI_Transmit((addr >> 16) & 0xFF);
	SPI_Transmit((addr >> 8) & 0xFF);
	SPI_Transmit(addr & 0xFF);
	uint8_t msb = SPI_Transmit(0x00);
	uint8_t lsb = SPI_Transmit(0x00);
	SETBIT(PORTB,PORTB2);
	return (msb<<8)|lsb;
}

uint32_t FRAM_Read32(uint32_t addr) {
	CLEARBIT(PORTB,PORTB2);
	SPI_Transmit(0x03); //READ
	SPI_Transmit((addr >> 16) & 0xFF);
	SPI_Transmit((addr >> 8) & 0xFF);
	SPI_Transmit(addr & 0xFF);
	uint32_t msb = SPI_Transmit(0x00);
	uint32_t byte1 = SPI_Transmit(0x00);
	uint32_t byte2 = SPI_Transmit(0x00);
	uint32_t lsb = SPI_Transmit(0x00);
	SETBIT(PORTB,PORTB2);
	return (msb<<24)|(byte1<<16)|(byte2<<8)|lsb;
}