#ifndef MAIN_H
#define MAIN_H

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <string.h>
#include "uart.h"
#include "i2cmaster.h"
#include "lcdutil.h"
#include "ds1307.h"
#include "fram.h"

#define SETBIT(ADDR,BIT) (ADDR |= (1<<BIT))
#define CLEARBIT(ADDR,BIT) (ADDR &= ~(1<<BIT))
#define FLIPBIT(ADDR,BIT) (ADDR ^= (1<<BIT))
#define IS_BIT_SET(ADDR,BIT) (ADDR & (1<<BIT))
#define IS_BIT_CLEAR(ADDR,BIT) !(IS_BIT_SET(ADDR,BIT))

uint16_t lidar_read_dist(void);
void print_values(void);
void store_in_mem(void);
void cancel_measurement(void);
void start_beep(uint16_t frequency, uint16_t length);
void timer1_init(void);
void timer2_init(void);
uint8_t try_read_from_pc(DateTime *calibr_time, int *keep_data);
void transmit_stored_data(void);

#endif