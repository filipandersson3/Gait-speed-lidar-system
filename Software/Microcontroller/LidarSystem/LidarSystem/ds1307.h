/*
 * ds1307.h
 *
 * Created: 2025-05-09 10:52:06
 *  Author: Filip Andersson
 */ 
#ifndef DS1307_H_
#define DS1307_H_

#include <avr/io.h>
#include "i2cmaster.h"

#define DS1307_ADDR		0b11010000
#define DS1307_SECONDS  0x00
#define DS1307_MINUTES  0x01
#define DS1307_HOURS    0x02
#define DS1307_DAY      0x03
#define DS1307_DATE     0x04
#define DS1307_MONTH    0x05
#define DS1307_YEAR     0x06
#define DS1307_CONTROL  0x07

typedef struct DateTime {
	uint16_t year;
	uint16_t month;
	uint16_t day;
	uint16_t hour;
	uint16_t minute;
	uint16_t second;
} DateTime;

DateTime DS1307_now(void);
void DS1307_set(DateTime dt);

#endif /* DS1307_H_ */