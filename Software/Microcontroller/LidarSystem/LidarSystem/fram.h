/*
 * fram.h
 *
 * Created: 2025-05-09 15:18:56
 *  Author: Filip Andersson
 */ 

#ifndef FRAM_H_
#define FRAM_H_

#include "avr/io.h"
#include <stdio.h>

#define SETBIT(ADDR,BIT) (ADDR |= (1<<BIT))
#define CLEARBIT(ADDR,BIT) (ADDR &= ~(1<<BIT))
#define FLIPBIT(ADDR,BIT) (ADDR ^= (1<<BIT))
#define IS_BIT_SET(ADDR,BIT) (ADDR & (1<<BIT))
#define IS_BIT_CLEAR(ADDR,BIT) !(IS_BIT_SET(ADDR,BIT))

#define SPI_MOSI_bit        PORTB3
#define SPI_MISO_bit        PORTB4
#define SPI_SCK_bit         PORTB5
#define SPI_CS_bit          PORTB2

#define SPI_DDR             DDRB
#define SPI_PORT            PORTB
#define SPI_PIN             PINB

void FRAM_SPI_Init(void);
void FRAM_Write8(uint32_t addr, uint8_t data);
void FRAM_Write16(uint32_t addr, uint16_t data);
void FRAM_Write32(uint32_t addr, uint32_t data);
uint8_t FRAM_Read8(uint32_t addr);
uint16_t FRAM_Read16(uint32_t addr);
uint32_t FRAM_Read32(uint32_t addr);

#endif /* FRAM_H_ */