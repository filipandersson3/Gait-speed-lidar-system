/*
 * ds1307.c
 *
 * Created: 2025-05-09 10:51:18
 *  Author: Filip Andersson
 */ 

#include "ds1307.h"

uint8_t bin2bcd(uint8_t val) {
	return ((val / 10) << 4) | (val % 10);
}

uint8_t bcd2bin(uint8_t val) {
	return ((val >> 4) * 10) + (val & 0x0F);
}

DateTime DS1307_now(void) {
	DateTime dt;
	i2c_start_wait(DS1307_ADDR+I2C_WRITE);
	i2c_write(DS1307_SECONDS);                 // set address
	i2c_rep_start(DS1307_ADDR+I2C_READ);
	dt.second = bcd2bin(i2c_readAck());
	dt.minute = bcd2bin(i2c_readAck());
	dt.hour = bcd2bin(i2c_readAck());
	i2c_readAck(); //ignore day of the week (1-7)
	dt.day = bcd2bin(i2c_readAck());
	dt.month = bcd2bin(i2c_readAck());
	dt.year = bcd2bin(i2c_readNak())+2000U;
	i2c_stop();
	
	return dt;
}

void DS1307_set(DateTime dt) {
	i2c_start_wait(DS1307_ADDR+I2C_WRITE);
	i2c_write(DS1307_SECONDS);                 // set address
	i2c_rep_start(DS1307_ADDR+I2C_WRITE);
	i2c_write(0);
	i2c_write(bin2bcd(dt.second));
	i2c_write(bin2bcd(dt.minute));
	i2c_write(bin2bcd(dt.hour));
	i2c_write(0); //ignore day of the week (1-7)
	i2c_write(bin2bcd(dt.day));
	i2c_write(bin2bcd(dt.month));
	i2c_write(bin2bcd(dt.year-2000));
	i2c_stop();
}