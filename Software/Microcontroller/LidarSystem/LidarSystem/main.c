/*
* LidarSystem.c
*
* Created: 2025-04-02 11:13:17
* Author : Filip Andersson
*/
#include "main.h"

volatile uint16_t lcd_counter = 0;
FILE lcdstream = FDEV_SETUP_STREAM(LCD_send_char, NULL, _FDEV_SETUP_WRITE);
char strbuf[33];
uint16_t distance;
uint32_t counter;
volatile uint16_t time_since_start;
uint8_t strupdated;
volatile uint8_t lcdcooldown;
uint16_t meas_time;
uint16_t meas_cplt;
uint8_t timeseries_start;
uint16_t timebuf[21];
uint16_t distbuf[21];
uint16_t meas_index;
uint8_t do_measurement;
uint8_t meas_ongoing;
uint16_t saved_distance;
uint32_t curr_addr;
uint32_t curr_startbyte_addr;
uint8_t startbyte_stored = 0;
uint8_t endbyte_stored = 1;
volatile uint16_t lasercounter;
uint32_t buzzer_beep_counter;
volatile uint16_t buzzer_beep_length;
volatile uint8_t buzzer_beep_start;
uint16_t counter2;
uint8_t counterremainder;
#define RECEIVEBUF_SIZE  60
char receivebuf[RECEIVEBUF_SIZE];
uint8_t menu_state = 0;
volatile uint32_t millis;

#define LIDAR_ADDR  0xC4      // I2C device address of lidar, see datasheet

#define MIN_DISTANCE	100
#define MAX_DISTANCE	350

#define BUZZER_PORT     PORTD
#define BUZZER_DDR      DDRD
#define BUZZER_PIN      PIND
#define BUZZER_BIT      PORTD7

#define LASER_PORT		PORTD
#define LASER_DDR		DDRD
#define LASER_PIN		PIND
#define LASER_BIT		PORTD5

#define BTN1_PORT		PORTD
#define BTN1_DDR		DDRD
#define BTN1_PIN		PIND
#define BTN1_BIT		PORTD6

// Timer constants (in milliseconds)
#define MEASUREMENT_TIMEOUT 60000 // maximum measurement length
#define SAMPLE_PERIOD 20 // time between samples
#define BUZZER_METRONOME 500 // how long between short buzzer beeps when walking
#define BUZZER_FREQ_DEFAULT 1000 // Hz

int main(void) {
	FILE uartstream = FDEV_SETUP_STREAM(TransmitByte, NULL, _FDEV_SETUP_WRITE);
	
	DDRD = 0b10111100;
	SETBIT(PORTD,PORTD2);
	SETBIT(PORTD,PORTD3);
	SETBIT(PORTD,PORTD4);
	//PORTD = 0x04;
	SETBIT(BTN1_PORT, BTN1_BIT); //pull-up, input
	
	LASER_DDR |= (1<<LASER_BIT); // set as output
	BUZZER_DDR |= (1<<BUZZER_BIT);
	
	FRAM_SPI_Init();
	LCD_init();
	i2c_init();
	uart1_init(3); //uart init 250000 bps
	stdout = &uartstream;
	printf("UART started...\r\n");
	timer1_init();
	timer2_init();
	_delay_ms(50);
	
	fprintf(&lcdstream, "Starting...");
	
	//FRAM_Write32(0x0, 0x04);
	curr_addr = FRAM_Read32(0x0);
	curr_startbyte_addr = curr_addr;
	
	// try to read a line from serial
	DateTime calibr_time;
	int valid_msg_received = 0;
	int keep_data;
	// get datetime from computer to calibrate
	// read lines from buffer until scanf success or timeout
	int startmillis = millis;
	while(!valid_msg_received || millis-startmillis < 500) {
		if (try_read_from_pc(&calibr_time, &keep_data) > 6) { //valid message received from pc?
			valid_msg_received = 1;
			DS1307_set(calibr_time);
			transmit_stored_data();
			if (keep_data == 0) {
				// set last address, "removing" all data after the address
				FRAM_Write32(0x0, 0x04);
			}
		}
	}
	
	FRAM_Write16(255992, (uint16_t)0xDEADBEEF);
	if (FRAM_Read16(255992) != (uint16_t)0xDEADBEEF) {
		LCD_clear();
		fprintf(&lcdstream, "memory doesn't");
		LCD_cursor_pos(40);
		fprintf(&lcdstream, "work :(");
		while(FRAM_Read16(255992) != (uint16_t)0xDEADBEEF){
			printf("memory doesn't work\n");
			FRAM_Write16(255992, (uint16_t)0xDEADBEEF);
			_delay_ms(100);
		};
	}
	FRAM_Write16(255992, 0x0);
	
	DateTime testtime = DS1307_now(); //get time
	start_beep(523, 180); // play some beeps until 1 second passed
	_delay_ms(300);
	start_beep(784, 180);
	_delay_ms(300);
	_delay_ms(450);
	if (DS1307_now().year < 2025U || DS1307_now().second-testtime.second < 1) { // check clock
		LCD_clear();
		fprintf(&lcdstream, "RTC doesn't");
		LCD_cursor_pos(40);
		fprintf(&lcdstream, "work :(");
		while(1){
			printf("RTC doesn't work\n");
			_delay_ms(100);
		};
	}
	
	/* Replace with your application code */
	while (1)
	{
		int prev_dist = distance;
		if (do_measurement) {
			distance = lidar_read_dist();
			if (timeseries_start) {
				timebuf[meas_index] = time_since_start;
				distbuf[meas_index++] = distance;
			}
			do_measurement = 0;
		}
		if(meas_index >= 20) {
			store_in_mem();
		}
		if (lcdcooldown == 0) {
			if (distance != prev_dist) {
				LCD_clear();
				 if (menu_state == 0) {
					LCD_cursor_pos(0);
					fprintf(&lcdstream, "Start measuring?");
					LCD_cursor_pos(40);
					fprintf(&lcdstream, "Distance: %.2lf m", distance/100.0);
				} else if (menu_state == 1) {
					LCD_cursor_pos(0);
					fprintf(&lcdstream, "Distance: %.2lf m", distance/100.0);
					LCD_cursor_pos(40);
					fprintf(&lcdstream, "Time: %.2lf s", time_since_start/1000.0);
				} else if (menu_state == 2) {
					LCD_cursor_pos(0);
					fprintf(&lcdstream, "Data saved.");
					LCD_cursor_pos(40);
					fprintf(&lcdstream, "Speed: %.2lf m/s", ((MAX_DISTANCE-MIN_DISTANCE)*10.0)/(meas_time));
					_delay_ms(100);
				} else if (menu_state == 3) {
					LCD_cursor_pos(0);
					fprintf(&lcdstream, "Wall distance low");
					LCD_cursor_pos(40);
					fprintf(&lcdstream, "%.2lfm < %.2lfm", distance/100.0, (MAX_DISTANCE+50)/100.0);
					_delay_ms(1000);
					menu_state = 0;
				}
				
				if ((menu_state == 0 || menu_state == 2) && IS_BIT_CLEAR(BTN1_PIN,BTN1_BIT)) {
					menu_state = 1;
				}
				if (distance < MAX_DISTANCE+50 && IS_BIT_CLEAR(BTN1_PIN,BTN1_BIT)) {
					menu_state = 3;
				}
				
				
				lcdcooldown = 1;
			}
		}
		// in measuring mode
		if (menu_state == 1) {
			if (distance < MIN_DISTANCE+3 && distance > MIN_DISTANCE) {
				start_beep(BUZZER_FREQ_DEFAULT, 100);
				SETBIT(PORTD,PORTD2);
				time_since_start = 0;
				timeseries_start = 1;
				} else {
				CLEARBIT(PORTD,PORTD2);
			}
			if (distance < MAX_DISTANCE+20 && distance > MIN_DISTANCE-20) {
				SETBIT(PORTD,PORTD3);
				} else {
				if (timeseries_start) {
					cancel_measurement();
				}
				CLEARBIT(PORTD,PORTD3);
			}
			if (distance < MAX_DISTANCE+5 && distance > MAX_DISTANCE) {
				CLEARBIT(PORTD,PORTD3);
				SETBIT(PORTD,PORTD4);
				if (timeseries_start) {
					meas_time = time_since_start;
					endbyte_stored = 0;
					store_in_mem();
					curr_startbyte_addr = curr_addr;
					startbyte_stored = 0;
					time_since_start = 0;
					timeseries_start = 0;
					menu_state = 2;
					start_beep(523, 150);
					_delay_ms(150);
					start_beep(784, 150);
					_delay_ms(150);
					start_beep(1046, 150);
					_delay_ms(150);
				}
				} else {
				CLEARBIT(PORTD,PORTD4);
			}
		}
	}
}

uint8_t try_read_from_pc(DateTime *calibr_time, int *keep_data) {
	uint8_t receive_count = 0;
	while (DataInReceiveBuffer() && receive_count < RECEIVEBUF_SIZE-1) {
		receivebuf[receive_count] = ReceiveByte();
		receivebuf[receive_count+1] = 0;
		if (receivebuf[receive_count] == '\n') {
			receivebuf[receive_count] = 0;
			break;
		}
		receive_count++;
	}
	
	int res = sscanf(receivebuf,
	"%4d-%d-%d_%d-%d-%d %d",
	&calibr_time->year,
	&calibr_time->month,
	&calibr_time->day,
	&calibr_time->hour,
	&calibr_time->minute,
	&calibr_time->second,
	keep_data);
	return res;
}

void transmit_stored_data(void) {
	printf("CONTACT\n");
	uint8_t found_startbyte = 0;
	for (int i = 4; i < curr_addr; i+=2) {
		uint16_t value = FRAM_Read16(i);
		if (value == 0xFFFF) {
			printf("END\n");
			found_startbyte = 0;
		}
		if (found_startbyte != 0) {
			if ((i-found_startbyte)%4 == 0) {
				printf("%.2lf\n", value/1000.0);
				} else {
				printf("%.2lf, ", value/100.0);
			}
		}
		if (value == 0xFFFE) {
			found_startbyte = i;
			DateTime timestamp;
			timestamp.year = FRAM_Read16(i+2);
			timestamp.month = FRAM_Read16(i+4);
			timestamp.day = FRAM_Read16(i+6);
			timestamp.hour = FRAM_Read16(i+8);
			timestamp.minute = FRAM_Read16(i+10);
			timestamp.second = FRAM_Read16(i+12);
			i += 12;
			printf("START %d-%d-%d_%d-%d-%d\n", timestamp.year,
			timestamp.month,
			timestamp.day,
			timestamp.hour,
			timestamp.minute,
			timestamp.second);
		}
	}
	printf("TRANSFER COMPLETE\n");
}

uint16_t lidar_read_dist(void) {
	uint8_t ret = 1;
	i2c_start_wait(LIDAR_ADDR+I2C_WRITE);     // set device address and write mode
	i2c_write(0x00);                        // write address
	i2c_write(0x04);                        // write value
	i2c_stop();
	
	while (IS_BIT_SET(ret,0)) {
		i2c_start_wait(LIDAR_ADDR+I2C_WRITE);     // set device address and write mode
		i2c_write(0x01);                        // write address
		i2c_rep_start(LIDAR_ADDR+I2C_READ);       // read mode
		ret = i2c_readNak();                    // read one byte
		i2c_stop();
	}
	
	i2c_start_wait(LIDAR_ADDR+I2C_WRITE);     // set device address and write mode
	i2c_write(0x8f);                        // write address
	i2c_rep_start(LIDAR_ADDR+I2C_READ);       // read mode
	uint8_t highByte = i2c_readNak();         // read one byte
	i2c_stop();
	
	i2c_start_wait(LIDAR_ADDR+I2C_WRITE);     // set device address and write mode
	i2c_write(0x90);                        // write address
	i2c_rep_start(LIDAR_ADDR+I2C_READ);       // read mode
	uint8_t lowByte = i2c_readNak();         // read one byte
	i2c_stop();
	
	return ((highByte << 8) | lowByte);
}

void print_values(void) {
	for (int i = 0; i < meas_index; i++) {
		printf("%.2lf, ", distbuf[i]/100.0);
		printf("%.2lf\n", timebuf[i]/1000.0);
	}
	meas_index = 0;
}

void store_in_mem(void) {
	if (!startbyte_stored) {
		curr_startbyte_addr = curr_addr;
		curr_addr += 14;
		startbyte_stored = 1;
	}
	for (int i = 0; i < meas_index; i++) {
		if (curr_addr > 255990) {
			cancel_measurement();
			LCD_clear();
			LCD_cursor_pos(0);
			fprintf(&lcdstream, "Memory full.");
			while (1);
		}
		FRAM_Write16(curr_addr,distbuf[i]);
		curr_addr += 2;
		FRAM_Write16(curr_addr,timebuf[i]);
		curr_addr += 2;
	}
	if (!endbyte_stored) {
		DateTime dt = DS1307_now();
		FRAM_Write16(curr_startbyte_addr,0xFFFE);
		FRAM_Write16(curr_startbyte_addr+2,dt.year);
		FRAM_Write16(curr_startbyte_addr+4,dt.month);
		FRAM_Write16(curr_startbyte_addr+6,dt.day);
		FRAM_Write16(curr_startbyte_addr+8,dt.hour);
		FRAM_Write16(curr_startbyte_addr+10,dt.minute);
		FRAM_Write16(curr_startbyte_addr+12,dt.second);
		FRAM_Write16(curr_addr,0xFFFF);
		curr_addr += 2;
		FRAM_Write32(0x0,curr_addr);
		endbyte_stored = 1;
	}
	meas_index = 0;
}

void cancel_measurement(void) {
	FRAM_Write32(0x0,curr_startbyte_addr); //"erase" data from last startbyte
	curr_addr = curr_startbyte_addr;
	startbyte_stored = 0;
	time_since_start = 0;
	timeseries_start = 0;
	meas_index = 0;
}

void start_beep(uint16_t frequency, uint16_t length) {
	uint8_t ocr = (uint8_t)((F_CPU / (8UL * 64UL * frequency)) - 1);
	if (ocr > 255) ocr = 255;
	OCR2A = ocr; // change timer compare register for frequency
	buzzer_beep_start = 1;
	buzzer_beep_length = length;
	TIMSK2 |= (1 << OCIE2A); // start timer interrupts
}

void timer1_init() {
	// set timer1 to CTC mode with OCR1A as the compare value
	TCCR1A = 0; // normal operation
	TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10); // CTC mode, prescaler 64
	OCR1A = 249; // for 1ms interrupt
	TIMSK1 = (1 << OCIE1A); // timer1 compare A interrupt
	sei(); // enable interrupts
}

void timer2_init() {
	// CTC Mode
	TCCR2A = (1 << WGM21);

	// Prescaler = 64
	TCCR2B = (1 << CS22) | (1 << CS21);

	// Output Compare Match A Interrupt
	TIMSK2 = (1 << OCIE2A);

	TCNT2 = 0;
}


ISR(USART_RX_vect) {
	UART_RX_interrupt();
}

ISR(USART_UDRE_vect) {
	UART_TX_interrupt();
}

ISR(TIMER1_COMPA_vect) { //triggered every millisecond
	millis++;
	if (millis%SAMPLE_PERIOD == 0) {
		do_measurement = 1;
	}
	if (timeseries_start) {
		time_since_start++;
		if (time_since_start == MEASUREMENT_TIMEOUT) {
			//cancel_measurement();
			time_since_start = 0;
		}
		if ((time_since_start%BUZZER_METRONOME==0) && distance < MAX_DISTANCE+3 && distance > MIN_DISTANCE) {
			start_beep(1000, 60);
		}
	}
	
	////////////// LASER //////////////
	lasercounter++;
	if (lasercounter == 200) {
		CLEARBIT(PORTD, PORTD5);
	}
	if (lasercounter == 1000) {
		// if button pressed, activate
		if (IS_BIT_CLEAR(BTN1_PIN,BTN1_BIT)) {
			SETBIT(PORTD, PORTD5);
		}
		lasercounter = 0;
	}
	
	////////////// BUZZER ///////////
	if (buzzer_beep_start == 1) {
		buzzer_beep_length--;
		if (buzzer_beep_length <= 0) {
			buzzer_beep_length = 0;
			buzzer_beep_start = 0;
			TIMSK2 &= ~(1 << OCIE2A); // stop timer interrupts
		}
	}
	
	////////////// LCD //////////////
	// lcd refresh (lcd clear takes 2ms)
	lcd_counter++;
	if (lcd_counter == 3) {
		lcdcooldown = 0;
	}
	if (lcd_counter == 40) {
		lcd_counter = 0;
	}
}

ISR(TIMER2_COMPA_vect) {
	////////////// BUZZER //////////////
	if (buzzer_beep_start == 1) {
		FLIPBIT(BUZZER_PORT,BUZZER_BIT);
	}
}