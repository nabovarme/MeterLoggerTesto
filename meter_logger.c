#include <pic18fregs.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <usart.h>
#include "config.h"
#include "meter_logger.h"

#define DEBUG
//#define OUTPUT_ON_SERIAL
#define DEBUG_LED_ON_FSK_RX
#define DEBUG_LED_ON_FSK_TX

#define QUEUE_SIZE 256
#define QUEUE_SIZE_COMBINED (4 * QUEUE_SIZE)

// Global variables
unsigned int timer_0;
unsigned int last_timer_0;
unsigned long timer_1_ms;
volatile unsigned int timer0_reload;

unsigned char debug_buffer[DEBUG_BUFFER_MAX];
unsigned char c;	// used in interrupt as buffer for fifo stuff

// command queue
unsigned int fifo_head, fifo_tail;
unsigned char fifo_buffer_0[QUEUE_SIZE];
unsigned char fifo_buffer_1[QUEUE_SIZE];
unsigned char fifo_buffer_2[QUEUE_SIZE];
unsigned char fifo_buffer_3[QUEUE_SIZE];

enum codec_type_t {
	NONE,
	TESTO,
	RS232_RX,
	RS232_TX,
	FSK_RX,
	FSK_TX
};
enum codec_type_t codec_type;

enum state_t {
	INIT_STATE,
	IDLE,
	START_BIT_WAIT,
	START_BIT_SENT,
	DATA_WAIT,
	DATA_SENT,
	PARITY_WAIT,
	STOP_BIT_WAIT,
	STOP_BIT2_WAIT,
	STOP_BIT_SENT,
	STOP_BIT2_SENT
};

typedef struct {
	enum state_t state;
	unsigned char start_bit;
	unsigned char start_bit_len;
	unsigned int data;
	unsigned char data_len;
} testo_ir_proto_t;

typedef struct {
	enum state_t state;
	unsigned char start_bit;
	unsigned char data;
	unsigned char data_len;
	unsigned char parity;
	unsigned char stop_bit;
} rs232_proto_t;

typedef struct {
	enum state_t state;
	unsigned int diff;
	unsigned int last_diff;
	unsigned int low_count;
	unsigned int high_count;
	unsigned char start_bit;
	unsigned int start_bit_time;
	unsigned char data;
	unsigned char data_len;
	unsigned char parity;
	unsigned char stop_bit;
} fsk_proto_t;

// for led flasher
enum led_flash_state_t {
	LED_FLASH_RUN,
	LED_FLASH_RUNNING,
	LED_FLASH_STOPPED
};
typedef struct {
	enum led_flash_state_t state;
	unsigned char timer;
} led_flash_t;
volatile led_flash_t led_flash;

volatile testo_ir_proto_t testo_ir_proto;
volatile rs232_proto_t rs232_proto;
volatile fsk_proto_t fsk_proto;

void main(void) {
	unsigned int i;
	unsigned char cmd, sub_cmd;
	unsigned int fifo_size, last_fifo_size;
    OSCCONbits.SCS = 0x10;
//    OSCCONbits.SCS = 0x00;	// external osc
    OSCCONbits.IRCF = 0x7;	// 8 MHz
	

	timer_1_ms = 0;

	fifo_head = 0;
	fifo_tail = 0;
	
	init_system();

#ifdef DEBUG
	usart_puts("\n\rMeterLogger... serial working\n\r");
	sleep_ms(100);
#endif

	fsk_rx_enable();
	while (1) {
		if (fifo_get(&cmd)) {
			switch (cmd) {
				case PROTO_TESTO:
					fsk_rx_disable();
					usart_puts("\n\rpress print on testo\n\r");
					testo_ir_enable();
					// wait for testo 310 stop sending data
					last_fifo_size = 0;
					sleep_ms(10000);						// 10 seconds to start printing
					fifo_size = fifo_in_use();
					while (fifo_size > last_fifo_size) {	// and wait while we are still receiving data
						last_fifo_size = fifo_size;
						sleep_ms(200);						// return data when no data for 200 ms
						fifo_size = fifo_in_use();
					}			
					testo_ir_disable();
					usart_puts("\n\rdone receiving - sending via serial/fsk\n\r");
#ifndef OUTPUT_ON_SERIAL
					fsk_tx_enable();
#endif
					while (fifo_get(&cmd)) {	// and print them to serial
#ifdef OUTPUT_ON_SERIAL
						sprintf(debug_buffer, "%c", cmd);
						usart_puts(debug_buffer);
#else               	
						fsk_tx_byte(cmd);
						sleep_ms(FSK_TX_SLEEP_AFTER);
#endif
					}
#ifndef OUTPUT_ON_SERIAL
					fsk_tx_disable();
#endif
					usart_puts("\n\rwaiting for new command\n\r");
					fsk_rx_enable();
					break;

				case PROTO_TESTO_DEMO:
					fsk_rx_disable();
					usart_puts("\n\recho test - send some data\n\r");
					fsk_rx_enable();
					// wait for iOS stop sending data
					last_fifo_size = 0;
					sleep_ms(1000);							// 1 second
					fifo_size = fifo_in_use();
					while (fifo_size > last_fifo_size) {	// and wait while we are still receiving data
						last_fifo_size = fifo_size;
						sleep_ms(500);						// return data when no data for 500 ms
						fifo_size = fifo_in_use();
					}			
					fsk_rx_disable();
#ifndef OUTPUT_ON_SERIAL
					fsk_tx_enable();
#endif
					while (fifo_get(&sub_cmd)) {
#ifdef OUTPUT_ON_SERIAL
						sprintf(debug_buffer, "%d ", sub_cmd);
						usart_puts(debug_buffer);
#else               	
						fsk_tx_byte(sub_cmd);
						sleep_ms(FSK_TX_SLEEP_AFTER);
#endif
					}
#ifndef OUTPUT_ON_SERIAL
					fsk_tx_disable();
#endif
					usart_puts("\n\rwaiting for new command\n\r");
					fsk_rx_enable();
					break;

				case PROTO_KAMSTRUP:
					fsk_rx_disable();
					usart_puts("\n\rkamstrup - send kmp frame data\n\r");
					fsk_rx_enable();
					
					// wait for iOS stop sending kmp command
					last_fifo_size = 0;
					sleep_ms(100);							// 100 ms
					fifo_size = fifo_in_use();
					while (fifo_size > last_fifo_size) {	// and wait while we are still receiving data
						last_fifo_size = fifo_size;
						sleep_ms(100);						// return data when no data for 100 ms
						fifo_size = fifo_in_use();
					}			
					fsk_rx_disable();
					
					usart_puts("\n\rkamstrup - kmp frame:\n\r");
					rs232_tx_enable();
					sleep_ms(RS232_TX_SLEEP_AFTER);		// DEBUG: should be its own #define
					while (fifo_get(&sub_cmd)) {
						sprintf(debug_buffer, "%d ", sub_cmd);
						usart_puts(debug_buffer);
						rs232_tx_byte(sub_cmd);
						sleep_ms(RS232_TX_SLEEP_AFTER);
					}
					sleep_ms(RS232_TX_SLEEP_AFTER);
					rs232_tx_disable();
		
					
					// Wait for kmp reply
					rs232_rx_enable();
					last_fifo_size = 0;
					sleep_ms(100);							// 100 ms
					fifo_size = fifo_in_use();
					while (fifo_size > last_fifo_size) {	// and wait while we are still receiving data
						last_fifo_size = fifo_size;
						sleep_ms(100);						// return data when no data for 100 ms
						fifo_size = fifo_in_use();
					}			
					
					rs232_rx_disable();
			
					// Send reply back to iOS
					usart_puts("\n\rkamstrup - kmp reply:\n\r");
#ifndef OUTPUT_ON_SERIAL
					fsk_tx_enable();
#endif
					while (fifo_get(&sub_cmd)) {
#ifdef OUTPUT_ON_SERIAL
						sprintf(debug_buffer, "%d ", sub_cmd);
						usart_puts(debug_buffer);
#else
						fsk_tx_byte(sub_cmd);
						sleep_ms(FSK_TX_SLEEP_AFTER);
#endif
					}
#ifndef OUTPUT_ON_SERIAL
					fsk_tx_disable();
#endif
					usart_puts("\n\rwaiting for new command\n\r");
					fsk_rx_enable();
					break;
			}
		}
	}
}

static void isr_high_prio(void) __interrupt 1 {
	// external interrupt handler
	if (INTCONbits.INT0IF && INTCONbits.INT0IE) {
		timer_0 = (unsigned int)(TMR0L) | ((unsigned int)(TMR0H) << 8);
		TMR0H = (unsigned char)(timer0_reload >> 8);
		TMR0L = (unsigned char)timer0_reload;

		switch (codec_type) {
			case TESTO:
				flash_led(100);
				switch (testo_ir_proto.state) {
					case INIT_STATE:
						T0CONbits.TMR0ON = 1;		// Start TMR0
						testo_ir_proto.start_bit_len = 1;
						testo_ir_proto.state = START_BIT_WAIT;
						break;
					case START_BIT_WAIT:
						if ((TICK + timer0_reload - TICK_ADJ < timer_0) && (timer_0 < TICK + timer0_reload + TICK_ADJ)) {
							if (testo_ir_proto.start_bit_len < 2) {
								testo_ir_proto.start_bit_len++;
							}
							else {
								// last start bits received, set state to DATA_WAIT
								testo_ir_proto.data = 0;
								testo_ir_proto.data_len = 0;
								testo_ir_proto.state = DATA_WAIT;
							}
						}
						else {
							// error in bit time framing
							testo_ir_proto.start_bit_len = 1;
							testo_ir_proto.state = START_BIT_WAIT;
						}
						break;
					case DATA_WAIT:
						if (testo_ir_proto.data_len <= 12) {
							if (((TICK + timer0_reload - TICK_ADJ < timer_0) && (timer_0 < TICK + timer0_reload + TICK_ADJ)) || ((3 * TICK + timer0_reload - TICK_ADJ < timer_0) && (timer_0 < 3 * TICK + timer0_reload + TICK_ADJ))) {
								// phase shift
								if ((testo_ir_proto.data & 1) != 0) {
									// previous bit is set
									testo_ir_proto.data <<= 1;		// bitshift once to left
									//testo_ir_proto.data &= 0b111111111110;	// and clear bit 0
								}
								else {
									// previous bit is zero
									testo_ir_proto.data <<= 1;		// bitshift once to left
									testo_ir_proto.data |= 1;	// and set bit 0
								}
								testo_ir_proto.data_len++;
							}
							else if ((2 * TICK + timer0_reload - TICK_ADJ < timer_0) && (timer_0 < 2 * TICK + timer0_reload + TICK_ADJ)) {
								// in phase
								if ((testo_ir_proto.data & 1) != 0) {
									// previous bit is set
									testo_ir_proto.data <<= 1;		// bitshift once to left
									testo_ir_proto.data |= 1;	// and set bit 0
								}
								else {
									// previous bit is zero
									testo_ir_proto.data <<= 1;		// bitshift once to left
									//testo_ir_proto.data &= 0b111111111110;	// and clear bit 0
								}
								testo_ir_proto.data_len++;
							}
							else {
								// error in bit time framing
						
								testo_ir_proto.start_bit_len = 1;
								testo_ir_proto.state = START_BIT_WAIT;
							}
							if (testo_ir_proto.data_len == 12) {
								// frame received!
								// calculate error correction and send via serial port
								if (testo_valid_err_corr(testo_ir_proto.data & 0xffff)) {
									//usart_putc(testo_ir_proto.data & 0xff);
									fifo_put(testo_ir_proto.data & 0xff);
									LED_PIN = 1;
								}
								testo_ir_proto.state = INIT_STATE;
							}
						}
						break;
				}
				break;
			case RS232_RX:
				switch (rs232_proto.state) {
					case START_BIT_WAIT:
						DEBUG3_PIN = 1;
						__asm
							nop
							nop
						__endasm;
						DEBUG3_PIN = 0;
						__asm
							nop
							nop
						__endasm;
						DEBUG3_PIN = 1;
						__asm
							nop
							nop
						__endasm;
						DEBUG3_PIN = 0;
						// sample data half bit time after...
						TMR0H = (unsigned char)(TIMER0_RS232_1200_START >> 8);
						TMR0L = (unsigned char)TIMER0_RS232_1200_START;
						INTCONbits.INT0IE = 0;		// disable ext int while we are using timer to receive data bits
						T0CONbits.TMR0ON = 1;		// Start TMR0
						rs232_proto.state = DATA_WAIT;
						break;
				}
				break;
		}
		INTCONbits.INT0IF = 0;	/* Clear Interrupt Flag */
	}
	
	// timer interrupt handler
	if (INTCONbits.TMR0IF && INTCONbits.TMR0IE) {
		// if timer overflow occurs - reset state
		TMR0H = (unsigned char)(timer0_reload >> 8);
		TMR0L = (unsigned char)timer0_reload;

		switch (codec_type) {
			case TESTO:							// rx timeout
				T0CONbits.TMR0ON = 0;			// Stop TMR0
				testo_ir_proto.state = INIT_STATE;
				sleep();						// sleep until we receive next bit via interrupt on INT0
				break;
			case RS232_TX:
				switch (rs232_proto.state) {
					case INIT_STATE:
						if (rs232_proto.data_len == 8) {
							IR_LED_PIN = 1;		// inverted rs232 output on ir, start bit = ir light
							rs232_proto.state = START_BIT_SENT;
						}
						break;
					case START_BIT_SENT:
						if (rs232_proto.data_len >= 1) {
							IR_LED_PIN = (rs232_proto.data & 1) == 0;	// inverted rs232 output on ir
							rs232_proto.data = rs232_proto.data >> 1;
							rs232_proto.data_len--;
						}
						else {
							IR_LED_PIN = 0;								// inverted rs232 output on ir					
							rs232_proto.state = STOP_BIT_SENT;
						}
						break;
					case STOP_BIT_SENT:
						IR_LED_PIN = 0;									// inverted rs232 output on ir
						rs232_proto.state = STOP_BIT2_SENT;
						break;
 	 				case STOP_BIT2_SENT:
						IR_LED_PIN = 0;									// inverted rs232 output on ir
						rs232_proto.state = INIT_STATE;
						break;
				}
				break;
			case RS232_RX:
				switch (rs232_proto.state) {
					case DATA_WAIT:
						rs232_proto.data_len++;
						if (rs232_proto.data_len <= 8) {
							if (IR_PIN) {		
								// logical 0, ir input inverted
								rs232_proto.data >>= 1;

								DEBUG3_PIN = 1;
								__asm
									nop
									nop
								__endasm;
								DEBUG3_PIN = 0;
							}
							else {				
								// logical 1, ir input inverted
								rs232_proto.data >>= 1;
								rs232_proto.data |= 0x80;
								
								DEBUG3_PIN = 1;
								__asm
									nop
									nop
								__endasm;
								DEBUG3_PIN = 0;
								__asm
									nop
									nop
								__endasm;
								DEBUG3_PIN = 1;
								__asm
									nop
									nop
								__endasm;
								DEBUG3_PIN = 0;
							}
						}
						else {
							rs232_proto.state = STOP_BIT_WAIT;
						}
						break;
					case STOP_BIT_WAIT:
					//	rs232_proto.state = STOP_BIT2_WAIT;
					//	break;
					// kamstrup meter does not realy send 2 stop bits... 
					//case STOP_BIT2_WAIT:
						fifo_put(rs232_proto.data);
						rs232_proto.data = 0;
						rs232_proto.data_len = 0;
						rs232_proto.state = START_BIT_WAIT;
						T0CONbits.TMR0ON = 0;
						INTCONbits.INT0IF = 0;		// dont enter ext int now
						INTCONbits.INT0IE = 1;		// enable ext int again
						break;
				}
				break;
			case FSK_RX:
				switch (fsk_proto.state) {
					case DATA_WAIT:
						fsk_proto.data_len++;						
						if (fsk_proto.data_len <= 8) {
							if ((fsk_proto.diff > 340) && (fsk_proto.diff < 476)) {
							//if (high_count > low_count) {
								// zero
								fsk_proto.data >>= 1;
								//fsk_proto.data <<= 1;
							}
							else {
								// one
								fsk_proto.data >>= 1;
								fsk_proto.data |= 0x80;
								//fsk_proto.data <<= 1;
								//fsk_proto.data |= 1;
							}

						}
						else {	// stop bit
							// check it...
							fsk_proto.state = STOP_BIT_WAIT;
						}					
						break;
						
					case STOP_BIT_WAIT:
						fifo_put(fsk_proto.data);
						fsk_proto.data = 0;
						fsk_proto.state = START_BIT_WAIT;
						//T0CONbits.TMR0ON = 0;
						INTCONbits.TMR0IE = 0;						
						break;
				}
				break;
			case FSK_TX:
				switch (fsk_proto.state) {
					case INIT_STATE:
						send_fsk_high();
						if (fsk_proto.data_len == 8) {
							fsk_proto.state = IDLE;
						}
#ifdef DEBUG_LED_ON_FSK_TX
						DEBUG_PIN = 0;
#endif
						break;
					case IDLE:
						send_fsk_low();
						fsk_proto.state = START_BIT_SENT;
#ifdef DEBUG_LED_ON_FSK_TX
						DEBUG_PIN = 1;
#endif
						break;
					case START_BIT_SENT:
						if (fsk_proto.data_len--) {
							if (fsk_proto.data & (0x80 >> fsk_proto.data_len)) {
								send_fsk_high();
#ifdef DEBUG_LED_ON_FSK_TX
								DEBUG_PIN = 0;
#endif
							}
							else {
								send_fsk_low();
#ifdef DEBUG_LED_ON_FSK_TX
								DEBUG_PIN = 1;
#endif
							}
						}
						if (fsk_proto.data_len == 0) {
							fsk_proto.state = DATA_SENT;
						}
						break;
					case DATA_SENT:
						send_fsk_high();
						fsk_proto.state = STOP_BIT_SENT;
#ifdef DEBUG_LED_ON_FSK_TX
						DEBUG_PIN = 0;
#endif
						break;
					case STOP_BIT_SENT:
						send_fsk_high();
						fsk_proto.state = INIT_STATE;
#ifdef DEBUG_LED_ON_FSK_TX
						DEBUG_PIN = 0;
#endif
						break;
				}
				break;
		}
		
		INTCONbits.TMR0IF = 0;
	}

	if (PIR2bits.CMIF && PIE2bits.CMIE) {
		// fsk demodulate
		if (CMCONbits.C1OUT) {		// rising edge
			timer_0 = (unsigned int)(TMR0L) | ((unsigned int)(TMR0H) << 8);
			//TMR0H = (unsigned char)(timer0_reload >> 8);
			//TMR0L = (unsigned char)timer0_reload;
			//fsk_proto.start_bit_time += timer_0;
#ifdef DEBUG_LED_ON_FSK_RX
			DEBUG_PIN = 1;
#endif
			fsk_proto.diff = timer_0 - last_timer_0;
			last_timer_0 = timer_0;

			if ((fsk_proto.diff > 340) && (fsk_proto.diff < 476)) {
				fsk_proto.low_count += fsk_proto.diff;
				if (fsk_proto.state == START_BIT_WAIT) {
					if (fsk_proto.low_count >= 800) {								// start bit received
						// start bits received, set state to DATA_WAIT
						TMR0H = (unsigned char)(timer0_reload >> 8);
						TMR0L = (unsigned char)timer0_reload;
						fsk_proto.low_count = 0;
						fsk_proto.high_count = 0;
//						fsk_proto.start_bit_time = 0;
						fsk_proto.data_len = 0;
						fsk_proto.data = 0;
						fsk_proto.state = DATA_WAIT;
						INTCONbits.TMR0IF = 0;		// clear flag so it dont enter isr now
						INTCONbits.TMR0IE = 1;		// Enable TMR0 Interrupt
					}
				}

			}
			else {
				if (fsk_proto.state == START_BIT_WAIT) {
					fsk_proto.low_count = 0;
					fsk_proto.high_count = 0;
				}
				else {
					fsk_proto.high_count += fsk_proto.diff;
				}
			}
		}
#ifdef DEBUG_LED_ON_FSK_RX
		else {					// faling edge
			DEBUG_PIN = 0;
		}
#endif

		PIR2bits.CMIF = 0;
	}
//	if (PIR1bits.TMR2IF) {
//		PIR1bits.TMR2IF = 0;
//		_debug();
//	}
}

static void isr_low_prio(void) __interrupt 2 {
	unsigned char c;

	if (PIR1bits.TMR1IF) {
		TMR1H = (unsigned char)(TIMER1_RELOAD >> 8);    // 262,158ms @ 8MHz
		TMR1L = (unsigned char)TIMER1_RELOAD;
		// blink led if set to LED_FLASH_RUN
		switch (led_flash.state) {
			case LED_FLASH_RUN:
				LED_PIN = 1;
				led_flash.state = LED_FLASH_RUNNING;
				break;
			case LED_FLASH_RUNNING:
				if (led_flash.timer-- == 0) {
					LED_PIN = 0;
					led_flash.state = LED_FLASH_STOPPED;
				}
				break;
		}
		timer_1_ms++;
		PIR1bits.TMR1IF = 0;    /* Clear the Timer Flag  */
	}

	// serial rx interrupt
	if (usart_drdy()) {
		// retransmit it
		c = usart_getc();
		usart_putc(c);
	}

}

void sleep_ms(unsigned long ms) {
	unsigned long start_timer_1_ms;
	start_timer_1_ms = timer_1_ms;	

// while the absolute value of the time diff < ms
	while ( (((signed long)(timer_1_ms - start_timer_1_ms) < 0) ? (-1 * (timer_1_ms - start_timer_1_ms)) : (timer_1_ms - start_timer_1_ms)) < ms) {
		// do nothing
	}
}

void init_system() {
	// PIN CONFIGURATION
	TRIS_IR_PIN = INPUT_STATE;		// as input
	TRIS_LED_PIN = OUTPUT_STATE;	// as output
	LED_PIN = 0;					// and clear
	
	TRIS_DEBUG_PIN = OUTPUT_STATE;	// as output
	DEBUG_PIN = 0;					// and clear
	TRIS_IR_LED_PIN = OUTPUT_STATE;	// as output
	IR_LED_PIN = 0;					// and clear
	TRIS_DEBUG3_PIN = OUTPUT_STATE;	// as output
	DEBUG3_PIN = 0;					// and clear
	
	TRIS_COMP1 = INPUT_STATE;		// as input
	TRIS_COMP2 = INPUT_STATE;		// as input

	// pwm
//	TRIS_PWM_PIN = INPUT_STATE;		// disable output from pwm module at init
	TRIS_PWM_PIN = OUTPUT_STATE;	// enable output from pwm module at init
	PWM_PIN = 0;					// and clear

	// serial
	TRIS_RX_PIN = INPUT_STATE;		// as input
	TRIS_TX_PIN = OUTPUT_STATE;		// as input


	// TIMERS
	// timer 1
	T1CONbits.TMR1ON = 1;
	T1CONbits.RD16 = 1;
	T1CONbits.TMR1CS = 0;   // internal clock source
	T1CONbits.T1OSCEN = 0;  // dont put t1 on pin
	T1CONbits.T1CKPS0 = 0;
	T1CONbits.T1CKPS1 = 0;
	IPR1bits.TMR1IP = 0;	// low priority
	PIE1bits.TMR1IE = 1;	// Ensure that TMR1 Interrupt is enabled
	PIR1bits.TMR1IF = 1;	// Force Instant entry to Timer 1 Interrupt

	/*
    // timer 2
    T2CONbits.TMR2ON = 1;
    T2CONbits.T2CKPS0 = 1;
    T2CONbits.T2CKPS1 = 1;
    T2CONbits.T2OUTPS0 = 1;
    T2CONbits.T2OUTPS1 = 1;
    T2CONbits.T2OUTPS2 = 1;
    T2CONbits.T2OUTPS3 = 1;
    IPR1bits.TMR2IP = 0;            // low priority
    PIE1bits.TMR2IE = 1;
    PIR1bits.TMR2IF = 1;
	*/

	/*
    // timer 3
    T3CONbits.RD16 = 1;
    T3CONbits.TMR3CS = 0;   // internal clock source
    T3CONbits.T3CKPS0 = 1;
    T3CONbits.T3CKPS0 = 1;
    IPR2bits.TMR3IP = 0;            // low priority
    T3CONbits.TMR3ON = 1;
    PIE2bits.TMR3IE = 1;
    PIR2bits.TMR3IF = 1;
	*/



    // set up interrupt
    RCONbits.IPEN = 1;
	
	INTCONbits.INT0IE = 0;		// disable ext int, enabled when ir demodulator is started
	INTCON2bits.INTEDG0 = 1;	// rising edge

	INTCONbits.PEIE = 1;
	INTCONbits.GIE = 1;	/* Enable Global interrupts   */	



	// USART interrupt low priority
	IPR1bits.RCIP = 0;
	IPR1bits.TXIP = 0;
	/*
	usart_open(	USART_TX_INT_OFF &
				USART_RX_INT_ON & 
				USART_BRGH_HIGH & 
				USART_ASYNCH_MODE & 
				USART_EIGHT_BIT &
				USART_CONT_RX,
				12     // 19200 kbps @ 8 MHz
	);
	*/
	my_usart_open();
}

void my_usart_open() {
	SPBRG = 103;					// 8MHz => 19230 baud
	TXSTAbits.BRGH = 1;	// (0 = low speed)
	TXSTAbits.SYNC = 0;	// (0 = asynchronous)
	BAUDCONbits.BRG16 = 1;
	
	// SPEN - Serial Port Enable Bit 
	RCSTAbits.SPEN = 1; // (1 = serial port enabled)

	// TXIE - USART Transmit Interupt Enable Bit
	PIE1bits.TXIE = 0; // (1 = enabled)
	IPR1bits.TXIP = 0; // USART Tx on low priority interrupt

	// RCIE - USART Receive Interupt Enable Bit
	PIE1bits.RCIE = 1; // (1 = enabled)
	IPR1bits.RCIP = 0; // USART Rx on low priority interrupt
	
	// TX9 - 9-bit Transmit Enable Bit
	TXSTAbits.TX9 = 0; // (0 = 8-bit transmit)
	
	// RX9 - 9-bit Receive Enable Bit
	RCSTAbits.RX9 = 0; // (0 = 8-bit reception)
	
	// CREN - Continuous Receive Enable Bit
	RCSTAbits.CREN = 1; // (1 = Enables receiver)
	
	// TXEN - Trasmit Enable Bit
	TXSTAbits.TXEN = 1; // (1 = transmit enabled)
}

unsigned char reverse(unsigned char b) {
	unsigned char c;
	c  = ((b >>  1) & 0x55) | ((b <<  1) & 0xaa);
	c |= ((b >>  2) & 0x33) | ((b <<  2) & 0xcc);
	c |= ((b >>  4) & 0x0f) | ((b <<  4) & 0xf0);
	return(c);
}

unsigned char testo_valid_err_corr(unsigned int c) {
    unsigned char calculated_err_corr, calculated_err_corr_bit;
    unsigned char i;
    
    calculated_err_corr = 0;
    
    // bit 3
    calculated_err_corr_bit = 0;
    for (i = 0; i < 8; i++) {
        calculated_err_corr_bit ^= (((c & 0x78) & (1 << i)) != 0);   // 0b01111000
    }
    calculated_err_corr |= calculated_err_corr_bit;
    calculated_err_corr = calculated_err_corr << 1;
    
    // bit 2
    calculated_err_corr_bit = 0;
    for (i = 0; i < 8; i++) {
        calculated_err_corr_bit ^= (((c & 0xe6) & (1 << i)) != 0);   // 0b11100110
    }
    calculated_err_corr |= calculated_err_corr_bit;
    calculated_err_corr = calculated_err_corr << 1;
    
    // bit 1
    calculated_err_corr_bit = 0;
    for (i = 0; i < 8; i++) {
        calculated_err_corr_bit ^= (((c & 0xd5) & (1 << i)) != 0);   // 0b11010101
    }
    calculated_err_corr |= calculated_err_corr_bit;
    calculated_err_corr = calculated_err_corr << 1;
    
    // bit 0
    calculated_err_corr_bit = 0;
    for (i = 0; i < 8; i++) {
        calculated_err_corr_bit ^= (((c & 0x8b) & (1 << i)) != 0);   // 0b10001011
    }
    calculated_err_corr |= calculated_err_corr_bit;

#ifdef DEBUG_SERIAL_ERROR_CORRECTION
	sprintf(debug_buffer, "\nbyte:\t"BYTETOBINARYPATTERN, BYTETOBINARY(c));
	usart_puts(debug_buffer);
	sprintf(debug_buffer, "\nerr_corr:\t"BYTETOBINARYPATTERN, BYTETOBINARY((c >> 8)));
	usart_puts(debug_buffer);
	sprintf(debug_buffer, "\ncalc_err_corr:\t"BYTETOBINARYPATTERN, BYTETOBINARY(calculated_err_corr));
	usart_puts(debug_buffer);
	return 0;
#else
    if ((c >> 8) == calculated_err_corr) {
        return 1;
    }
    else {
        return 0;
    }
#endif
}

void testo_ir_enable() {
	testo_ir_proto.state = INIT_STATE;
	testo_ir_proto.start_bit_len = 0;

	timer0_reload = TIMER0_TESTO;
	
	codec_type = TESTO;

	// timer 0
	T0CONbits.TMR0ON = 0;
	T0CONbits.T0PS0 = 0;
	T0CONbits.T0PS1 = 0;
	T0CONbits.T0PS2 = 0;		// prescaler 1:2
	T0CONbits.T08BIT = 0;		// use timer0 16-bit counter
	T0CONbits.T0CS = 0;			// internal clock source
	T0CONbits.PSA = 1;			// disable timer0 prescaler
	INTCON2bits.TMR0IP = 1;		// high priority
	INTCONbits.TMR0IE = 1;		// Enable TMR0 Interrupt
	INTCONbits.TMR0IF = 0;

	INTCONbits.INT0IE = 1;		// enable ext int
	INTCON2bits.INTEDG0 = 1;	// rising edge
}

void testo_ir_disable() {
	codec_type = NONE;
	INTCONbits.INT0IE = 0;		// disable ext int
}

void rs232_tx_enable() {
	timer0_reload = TIMER0_RS232_1200;

	rs232_proto.state = INIT_STATE;
	rs232_proto.data_len = 0;
	
	IR_LED_PIN = 0;				// inverted rs232 output on ir, idle = no ir light

	codec_type = RS232_TX;

	// timer 0
	T0CONbits.TMR0ON = 0;
	T0CONbits.T0PS0 = 0;
	T0CONbits.T0PS1 = 0;
	T0CONbits.T0PS2 = 0;		// prescaler 1:2
	T0CONbits.T08BIT = 0;		// use timer0 16-bit counter
	T0CONbits.T0CS = 0;			// internal clock source
	T0CONbits.PSA = 1;			// disable timer0 prescaler
	INTCON2bits.TMR0IP = 1;		// high priority
	INTCONbits.TMR0IE = 1;		// Enable TMR0 Interrupt
	INTCONbits.TMR0IF = 0;

	INTCONbits.INT0IE = 0;		// disable ext int while sending with software uart
	T0CONbits.TMR0ON = 1;		// start timer 0
}

void rs232_tx_disable() {
	codec_type = NONE;
	IR_LED_PIN = 0;				// no need to set it to inverted idle
}

void rs232_rx_enable() {
	rs232_proto.state = START_BIT_WAIT;
	rs232_proto.data_len = 0;

	timer0_reload = TIMER0_RS232_1200;
	
	codec_type = RS232_RX;

	// timer 0
	T0CONbits.TMR0ON = 0;
	T0CONbits.T0PS0 = 0;
	T0CONbits.T0PS1 = 0;
	T0CONbits.T0PS2 = 0;		// prescaler 1:2
	T0CONbits.T08BIT = 0;		// use timer0 16-bit counter
	T0CONbits.T0CS = 0;			// internal clock source
	T0CONbits.PSA = 1;			// disable timer0 prescaler
	INTCON2bits.TMR0IP = 1;		// high priority
	INTCONbits.TMR0IE = 1;		// Enable TMR0 Interrupt
	INTCONbits.TMR0IF = 0;

	INTCONbits.INT0IE = 1;		// enable ext int
	INTCON2bits.INTEDG0 = 1;	// rising edge
}

void rs232_rx_disable() {
	INTCONbits.INT0IE = 0;		// disable ext int
	codec_type = NONE;
}

void rs232_tx_byte(unsigned char c) {
	rs232_proto.data = c;
	rs232_proto.data_len = 8;
}

void fsk_tx_enable() {
	timer0_reload = TIMER0_FSK;

	fsk_proto.state = INIT_STATE;
	codec_type = FSK_TX;
	
	// timer 0
	T0CONbits.TMR0ON = 1;
	T0CONbits.T0PS0 = 0;
	T0CONbits.T0PS1 = 0;
	T0CONbits.T0PS2 = 0;		// prescaler 1:2
	T0CONbits.T08BIT = 0;		// use timer0 16-bit counter
	T0CONbits.T0CS = 0;			// internal clock source
	T0CONbits.PSA = 1;			// disable timer0 prescaler
	INTCON2bits.TMR0IP = 1;		// high priority
	INTCONbits.TMR0IE = 1;		// Enable TMR0 Interrupt
}

void fsk_tx_disable() {
	codec_type = NONE;
}

void fsk_rx_enable() {
	fsk_proto.state = START_BIT_WAIT;
	fsk_proto.start_bit_time = 0;
	
	timer0_reload = TIMER0_FSK;

	codec_type = FSK_RX;
	
	// timer 0
	T0CONbits.TMR0ON = 1;
	T0CONbits.T0PS0 = 0;
	T0CONbits.T0PS1 = 0;
	T0CONbits.T0PS2 = 0;		// prescaler 1:2
	T0CONbits.T08BIT = 0;		// use timer0 16-bit counter
	T0CONbits.T0CS = 0;			// internal clock source
	T0CONbits.PSA = 1;			// disable timer0 prescaler
	INTCON2bits.TMR0IP = 1;		// high priority
	INTCONbits.TMR0IE = 0;		// Dont enable TMR0 Interrupt

	// When CVRR = 1: CVREF = ((CVR3:CVR0)/24) x (CVRSRC), When CVRR = 0: CVREF = (CVRSRC/4) + (((CVR3:CVR0)/32) x CVRSRC)
	CVRCONbits.CVREF = 0xf;	// 0V
	// Comparator VREF Source Selection bit
	CVRCONbits.CVRSS = 0;	// VDD – VSS
	CVRCONbits.CVRR = 0;	// high range, 0.25 CVRSRC to 0.75 CVRSRC, with CVRSRC/32 step size
	CVRCONbits.CVR = 9;		// 2,65625 V
	CVRCONbits.CVROE = 0;	// Comparator VREF Output disabled, CVREF voltage is disconnected from the RA2/AN2/VREF-/CVREF pin
	CVRCONbits.CVREN = 1;	// Comparator Voltage Reference Enable bit
	
	CMCONbits.CM = 0x6;		// four inputs multiplexed to two comparators
	CMCONbits.CIS = 0;		// multiplexed to RA0/AN0 and RA1/AN1
	CMCONbits.C1INV = 1;	// inverted output, C1 VIN+ < C1 VIN-

	IPR2bits.CMIP = 1;		// high priority
	PIE2bits.CMIE = 1;		// Enable comparator interrupt
}

void fsk_rx_disable() {
	PIE2bits.CMIE = 0;		// Disable comparator interrupt
	codec_type = NONE;
}

void send_fsk_high(void) {
	PWM_PIN = 1;
	__asm
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
	__endasm;
	PWM_PIN = 0;
	__asm
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
	__endasm;
	PWM_PIN = 1;
	__asm
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
	__endasm;
	PWM_PIN = 0;
	__asm
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
	__endasm;
	PWM_PIN = 1;
	__asm
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
	__endasm;
	PWM_PIN = 0;
	__asm
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
	__endasm;
	PWM_PIN = 1;
	__asm
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
	__endasm;
	PWM_PIN = 0;
	__asm
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
	__endasm;
	PWM_PIN = 1;
	__asm
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
	__endasm;
	PWM_PIN = 0;
	__asm
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
	__endasm;
	PWM_PIN = 1;
	__asm
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
	__endasm;
	PWM_PIN = 0;
}

void send_fsk_low(void) {
	PWM_PIN = 1;
	__asm
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
	__endasm;
	PWM_PIN = 0;
	__asm
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
	__endasm;
	PWM_PIN = 1;
	__asm
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
	__endasm;
	PWM_PIN = 0;
	__asm
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
	__endasm;
	PWM_PIN = 1;
	__asm
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
	__endasm;
	PWM_PIN = 0;
	__asm
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
	__endasm;
	PWM_PIN = 1;
	__asm
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
	__endasm;
	PWM_PIN = 0;
}

void fsk_tx_byte(unsigned char c) {
	fsk_proto.data = c;
	fsk_proto.data_len = 8;
}

unsigned int fifo_in_use() {
	return fifo_head - fifo_tail;
}

unsigned char fifo_put(unsigned char c) {
	if (fifo_in_use() != QUEUE_SIZE_COMBINED) {
        switch (fifo_head/QUEUE_SIZE) {
            case 0:
                fifo_buffer_0[fifo_head % QUEUE_SIZE] = c;
                break;
            case 1:
                fifo_buffer_1[fifo_head % QUEUE_SIZE] = c;
                break;
            case 2:
                fifo_buffer_2[fifo_head % QUEUE_SIZE] = c;
                break;
            case 3:
                fifo_buffer_3[fifo_head % QUEUE_SIZE] = c;
                break;
        }
        fifo_head++;
		// wrap
		if (fifo_head == QUEUE_SIZE_COMBINED) {
			fifo_head = 0;
		}
		return 1;
	}
	else {
		return 0;
	}
}

unsigned char fifo_get(unsigned char *c) {
	if (fifo_in_use() != 0) {
        switch (fifo_tail/QUEUE_SIZE) {
            case 0:
                *c = fifo_buffer_0[fifo_tail % QUEUE_SIZE];
                break;
            case 1:
                *c = fifo_buffer_1[fifo_tail % QUEUE_SIZE];
                break;
            case 2:
                *c = fifo_buffer_2[fifo_tail % QUEUE_SIZE];
                break;
            case 3:
                *c = fifo_buffer_3[fifo_tail % QUEUE_SIZE];
                break;
        }
        fifo_tail++;
		// wrap
		if (fifo_tail == QUEUE_SIZE_COMBINED) {
			fifo_tail = 0;
		}
		return 1;
	}
	else {
		return 0;
	}
}

unsigned char fifo_snoop(unsigned char *c, unsigned int pos) {
	if (fifo_in_use() > (pos)) {
        switch (fifo_tail/QUEUE_SIZE) {
            case 0:
                *c = fifo_buffer_0[(fifo_tail + pos) % QUEUE_SIZE];
                break;
            case 1:
                *c = fifo_buffer_1[(fifo_tail + pos) % QUEUE_SIZE];
                break;
            case 2:
                *c = fifo_buffer_2[(fifo_tail + pos) % QUEUE_SIZE];
                break;
            case 3:
                *c = fifo_buffer_3[(fifo_tail + pos) % QUEUE_SIZE];
                break;
        }
		return 1;
	}
	else {
		return 0;
	}
}

void flash_led(unsigned char ms) {
	led_flash.timer = ms;
	led_flash.state = LED_FLASH_RUN;
}

void _debug() {
	DEBUG_PIN = 0x1;
	__asm 
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
	__endasm;
	DEBUG_PIN = 0x0;
	__asm 
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
	__endasm;

}

void _debug2() {
	DEBUG2_PIN = 0x1;
	__asm 
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
	__endasm;
	DEBUG2_PIN = 0x0;
	__asm 
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
		nop
	__endasm;
}

