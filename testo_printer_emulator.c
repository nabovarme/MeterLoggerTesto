#include <pic18fregs.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <usart.h>
#include "config.h"
#include "testo_printer_emulator.h"

//#define DEBUG
//#define DEBUG_PHASE_SHIFT_DECODED
//#define DEBUG_SERIAL_PHASE_SHIFT_DECODED

//#define DEBUG_SERIAL_ERROR_CORRECTION

unsigned long timer_1_ms;
#ifdef DEBUG
unsigned char buffer[64];
#endif

enum codec_type_t {
	NONE,
	TESTO,
	RS232
};
enum codec_type_t codec_type;

enum rx_state_t {
	INIT_STATE,
	START_BIT_WAIT,
	DATA_WAIT,
	PARITY_WAIT,
	STOP_BIT_WAIT
};

typedef struct {
	enum rx_state_t state;
	unsigned char start_bit;
	unsigned char start_bit_len;
	unsigned int data;
	unsigned char data_len;
} testo_ir_proto_t;

typedef struct {
	enum rx_state_t state;
	unsigned char start_bit;
	unsigned char data;
	unsigned char data_len;
	unsigned char parity;
	unsigned char stop_bit;
} rs232_ir_proto_t;

volatile testo_ir_proto_t testo_ir_proto;
volatile rs232_ir_proto_t rs232_ir_proto;
unsigned int timer_0;

void main(void) {
    OSCCONbits.SCS = 0x10;
    OSCCONbits.IRCF = 0x7;	// 8 MHz

	timer_1_ms = 0;
	
	testo_ir_proto.state = INIT_STATE;
	testo_ir_proto.start_bit_len = 0;
	
	init_system();
	testo_ir_enable();
	
//	rs232_tx_enable();

#ifdef DEBUG
	usart_puts("Testo printer emulator... serial working\n\r");
#endif

	while (1) {
		// do nothing
		/*
		send_hijack_carrier();
		send_hijack_test();
		send_hijack_carrier();
		send_hijack_carrier();
		send_hijack_carrier();
		send_hijack_carrier();
		send_hijack_carrier();
		send_hijack_carrier();
		send_hijack_carrier();
		send_hijack_carrier();
		send_hijack_carrier();
		send_hijack_carrier();
		send_hijack_carrier();
		*/
//		TRIS_PWM_PIN = OUTPUT_STATE;
//		sleep_ms(100);
//		TRIS_PWM_PIN = INPUT_STATE;
//		sleep_ms(100);
	}
}

static void isr_high_prio(void) __interrupt 1 {
	if (INTCONbits.INT0IF) {
		timer_0 = (unsigned int)(TMR0L) | ((unsigned int)(TMR0H) << 8);
		TMR0H = (unsigned char)(TIMER0_RELOAD >> 8);
		TMR0L = (unsigned char)TIMER0_RELOAD;

		switch (codec_type) {
			case TESTO:
				switch (testo_ir_proto.state) {
			case INIT_STATE:
#ifdef DEBUG_PHASE_SHIFT_DECODED
				_debug();
#endif
				T0CONbits.TMR0ON = 1;		// Start TMR0
				testo_ir_proto.start_bit_len = 1;
				testo_ir_proto.state = START_BIT_WAIT;
				break;
			case START_BIT_WAIT:
				if ((TICK + TIMER0_RELOAD - TICK_ADJ < timer_0) && (timer_0 < TICK + TIMER0_RELOAD + TICK_ADJ)) {
#ifdef DEBUG_PHASE_SHIFT_DECODED
					_debug2();
					_debug2();
#endif
					if (testo_ir_proto.start_bit_len < 2) {
						testo_ir_proto.start_bit_len++;
#ifdef DEBUG_PHASE_SHIFT_DECODED
//						_debug();
#endif
					}
					else {
						// last start bits received, set state to DATA_WAIT
						testo_ir_proto.data = 0;
						testo_ir_proto.data_len = 0;
						testo_ir_proto.state = DATA_WAIT;
#ifdef DEBUG_PHASE_SHIFT_DECODED
						_debug();
						_debug();
#endif
					}
				}
				else {
					// error in bit time framing
					testo_ir_proto.start_bit_len = 1;
					testo_ir_proto.state = START_BIT_WAIT;
#ifdef DEBUG_PHASE_SHIFT_DECODED
						_debug2();
#endif
				}
				break;
			case DATA_WAIT:
				if (testo_ir_proto.data_len <= 12) {
					if (((TICK + TIMER0_RELOAD - TICK_ADJ < timer_0) && (timer_0 < TICK + TIMER0_RELOAD + TICK_ADJ)) || ((3 * TICK + TIMER0_RELOAD - TICK_ADJ < timer_0) && (timer_0 < 3 * TICK + TIMER0_RELOAD + TICK_ADJ))) {
						// phase shift
						if ((testo_ir_proto.data & 1) != 0) {
							// previous bit is set
							testo_ir_proto.data = testo_ir_proto.data << 1;		// bitshift once to left
							testo_ir_proto.data &= 0b111111111110;	// and clear bit 0
#ifdef DEBUG_PHASE_SHIFT_DECODED
//							_debug();
#endif
#ifdef DEBUG_SERIAL_PHASE_SHIFT_DECODED
							usart_putc('0');
#endif
						}
						else {
							// previous bit is zero
							testo_ir_proto.data = testo_ir_proto.data << 1;		// bitshift once to left
							testo_ir_proto.data |= 0b0000000000001;	// and set bit 0
#ifdef DEBUG_PHASE_SHIFT_DECODED
//							_debug();
//							_debug();
//							_debug();
#endif
#ifdef DEBUG_SERIAL_PHASE_SHIFT_DECODED
							usart_putc('1');
#endif
						}
						testo_ir_proto.data_len++;
					}
					else if ((2 * TICK + TIMER0_RELOAD - TICK_ADJ < timer_0) && (timer_0 < 2 * TICK + TIMER0_RELOAD + TICK_ADJ)) {
						// in phase
						if ((testo_ir_proto.data & 1) != 0) {
							// previous bit is set
							testo_ir_proto.data = testo_ir_proto.data << 1;		// bitshift once to left
							testo_ir_proto.data |= 0b0000000000001;	// and set bit 0
#ifdef DEBUG_PHASE_SHIFT_DECODED
//							_debug();
//							_debug();
//							_debug();
#endif
#ifdef DEBUG_SERIAL_PHASE_SHIFT_DECODED
							usart_putc('1');
#endif
						}
						else {
							// previous bit is zero
							testo_ir_proto.data = testo_ir_proto.data << 1;		// bitshift once to left
							testo_ir_proto.data &= 0b111111111110;	// and clear bit 0
#ifdef DEBUG_PHASE_SHIFT_DECODED
//							_debug();
#endif
#ifdef DEBUG_SERIAL_PHASE_SHIFT_DECODED
							usart_putc('0');
#endif
						}
						testo_ir_proto.data_len++;
					}
					else {
						// error in bit time framing
#ifdef DEBUG_SERIAL_PHASE_SHIFT_DECODED
						sprintf(buffer, "\t#%u\terror\tTMR0 %u\n", testo_ir_proto.data_len, timer_0);
						usart_puts(buffer);
#endif
						
						testo_ir_proto.start_bit_len = 1;
						testo_ir_proto.state = START_BIT_WAIT;
					}
					if (testo_ir_proto.data_len == 12) {
						// frame received!
						// calculate error correction and send via serial port
#ifdef DEBUG_SERIAL_PHASE_SHIFT_DECODED
						sprintf(buffer, "\t#%u\tdata %u\tTMR0 %u\n", testo_ir_proto.data_len, (testo_ir_proto.data & 0xff), timer_0);
						usart_puts(buffer);
#else
						if (valid_err_corr(testo_ir_proto.data & 0xffff)) {
							usart_putc(testo_ir_proto.data & 0xff);
						}
#endif
						testo_ir_proto.state = INIT_STATE;
					}
				}
				break;
		}
				break;
			case RS232:
				// do nothing
				break;
		}
				
		INTCONbits.INT0IF = 0;	/* Clear Interrupt Flag */
	}
	if (INTCONbits.TMR0IF) {
		// if timer overflow occurs - reset state
		TMR0H = (unsigned char)(TIMER0_RELOAD >> 8);
		TMR0L = (unsigned char)TIMER0_RELOAD;

		switch (codec_type) {
			case TESTO:
#ifdef DEBUG_PHASE_SHIFT_DECODED
				_debug2();
				_debug2();
				_debug2();
#endif
				T0CONbits.TMR0ON = 0;			// Stop TMR0
				testo_ir_proto.state = INIT_STATE;
				sleep();						// sleep until we receive next bit via interrupt on INT0
				if (testo_ir_proto.state != INIT_STATE) {
#ifdef DEBUG_SERIAL_PHASE_SHIFT_DECODED
					sprintf(buffer, "\t#%u\terror\tTMR0 %u\n", testo_ir_proto.data_len, timer_0);
					usart_puts(buffer);
#endif
				}
				break;
			case RS232:
				// do nothing
				rs232_ir_proto.data ^= 1;
				TRIS_PWM_PIN = rs232_ir_proto.data;
				break;
		}
		
		INTCONbits.TMR0IF = 0;
	}
}

static void isr_low_prio(void) __interrupt 2 {
	unsigned char c;

	if (PIR1bits.TMR1IF) {
		TMR1H = (unsigned char)(TIMER1_RELOAD >> 8);    // 262,158ms @ 8MHz
		TMR1L = (unsigned char)TIMER1_RELOAD;
		PIR1bits.TMR1IF = 0;    /* Clear the Timer Flag  */
		timer_1_ms++;
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
	
	TRIS_DEBUG_PIN = OUTPUT_STATE;	// as output
	DEBUG_PIN = 0;					// and clear
	TRIS_DEBUG2_PIN = OUTPUT_STATE;	// as output
	DEBUG2_PIN = 0;					// and clear
	
	// pwm
//	TRIS_PWM_PIN = INPUT_STATE;		// disable output from pwm module at init
	TRIS_PWM_PIN = OUTPUT_STATE;	// enable output from pwm module at init
	PWM_PIN = 0;					// and clear

	// serial
	TRIS_RX_PIN = INPUT_STATE;		// as input
	TRIS_TX_PIN = OUTPUT_STATE;		// as input



	// TIMERS
	// timer 0
	T0CONbits.TMR0ON = 0;
	T0CONbits.T0PS0 = 0;
	T0CONbits.T0PS1 = 0;
	T0CONbits.T0PS2 = 0;	// prescaler 1:2
	T0CONbits.T08BIT = 0;   // use timer0 16-bit counter
	T0CONbits.T0CS = 0;             // internal clock source
	T0CONbits.PSA = 1;              // disable timer0 prescaler
	INTCON2bits.TMR0IP = 1; // high priority
	INTCONbits.T0IE = 1;    // Enable TMR0 Interrupt
	INTCONbits.TMR0IF = 1;  // Force Instant entry to Timer 0 Interrupt

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



	// PWM
//	PR2 = 90;				// pwm period 22kHz
//	CCPR2L = 45;			// duty cycle msb
	
//	T2CONbits.T2CKPS = 0;	// timer 2 clock prescaler is 1
//	T2CONbits.T2OUTPS = 0;	// timer2 output 1:1 postscaler
//	T2CONbits.TMR2ON = 1;	// timer 2 on
	
//	CCP2CONbits.CCP2M = 0xc;// pwm mode: P1A, P1C active-high; P1B, P1D active-high
	
}

void my_usart_open() {
	SPBRG = 103;					// 8MHz => 19230 baud
	TXSTAbits.BRGH = 1;	// (1 = high speed)
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

unsigned char valid_err_corr(unsigned int c) {
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
	sprintf(buffer, "\nbyte:\t"BYTETOBINARYPATTERN, BYTETOBINARY(c));
	usart_puts(buffer);
	sprintf(buffer, "\nerr_corr:\t"BYTETOBINARYPATTERN, BYTETOBINARY((c >> 8)));
	usart_puts(buffer);
	sprintf(buffer, "\ncalc_err_corr:\t"BYTETOBINARYPATTERN, BYTETOBINARY(calculated_err_corr));
	usart_puts(buffer);
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
	// should configure timers here and set codec to TESTO
	codec_type = TESTO;
	INTCONbits.INT0IE = 1;		// enable ext int
	INTCON2bits.INTEDG0 = 1;	// rising edge
}

void testo_ir_disable() {
	codec_type = NONE;
	INTCONbits.INT0IE = 0;		// disable ext int
}

void rs232_tx_enable() {
	codec_type = RS232;
	INTCONbits.INT0IE = 0;		// disable ext int while sending with software uart
	T0CONbits.TMR0ON = 1;		// start timer 0
}

void send_hijack_carrier(void) {
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
	__endasm;
}

void send_hijack_test(void) {
	// the frame used consists of a start bit, eight data bits, one parity bits and one stop bit
	
	// start bit
	// 0
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
	__endasm;

	// bit 1
	// 1
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
	__endasm;

	// bit 2
	// 1
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
	__endasm;
	
	// bit 3
	// 0
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
	__endasm;

	// bit 4
	// 0
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
	__endasm;
	
	// bit 5
	// 1
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
	__endasm;

	// bit 6
	// 1
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
	__endasm;
	
	// bit 7
	// 1
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
	__endasm;
	
	// bit 8
	// 0
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
	__endasm;

	// stop bit
	// 1
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
	__endasm;

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

