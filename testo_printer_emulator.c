#include <pic18fregs.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <usart.h>
#include "config.h"
#include "testo_printer_emulator.h"

#define DEBUG

unsigned char i;
unsigned long timer_0_ms;

// command queue
#define QUEUE_SIZE	100
volatile unsigned int fifo_head, fifo_tail;
volatile unsigned char fifo_buffer[QUEUE_SIZE];

enum ir_state_t {
	INIT_STATE,
	START_BIT_WAIT,
	ERR_CORR_WAIT,
	DATA_WAIT
};

typedef struct {
	enum ir_state_t state;
	unsigned char start_bit;
	unsigned char start_bit_len;
	unsigned char err_corr_bit;
	unsigned char err_corr_bit_len;
	unsigned char data;
	unsigned char data_len;
	
} ir_proto_t;

volatile ir_proto_t ir_proto;
unsigned int timer_0;

void main(void) {
    OSCCONbits.SCS = 0x10;
    OSCCONbits.IRCF = 0x7;	// 8 MHz
//	WDTCONbits.SWDTEN = 1;	// enable watchdog

	timer_0_ms = 0;
	
	ir_proto.state = INIT_STATE;
	ir_proto.start_bit = 0;
	ir_proto.start_bit_len = 0;
	
	init_system();

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
				12     // 19200 kbps @ 4 MHz
	);
	*/
	my_usart_open();

	sleep_ms(1000);	// let stuff settle...
	usart_puts("OpenStoker starting... serial working\n\r");

	TRISBbits.RB0 = 0x1;	// input
	TRISCbits.RC0 = 0x1;	// input
	TRISDbits.RD4 = 0x0;	// output
	PORTDbits.RD4 = 0;		// clear output

	while (1) {
		sleep_ms(1);
		/*
		if (PORTCbits.RC0) {
			PORTDbits.RD4 = 0x0;
		}
		else {
			PORTDbits.RD4 = 0x1;
		}
		*/
	}
}

static void isr_high_prio(void) __interrupt 1 {
	if (INTCONbits.INT0IF) {
		timer_0 = (unsigned int)(TMR0L) | ((unsigned int)(TMR0H) << 8);
		TMR0H = 0x00;
		TMR0L = 0x00;

		switch (ir_proto.state) {
			case INIT_STATE:
				ir_proto.start_bit = 1;
				ir_proto.start_bit_len = 1;
				ir_proto.state = START_BIT_WAIT;
				//_debug();
				break;
			case START_BIT_WAIT:
				if ((TICK_LOW < timer_0) && (timer_0 < TICK_HIGH)) {
					if (ir_proto.start_bit++ < 3) {
						//_debug();
						
						ir_proto.start_bit = (ir_proto.start_bit << 1) & 0b111;
						ir_proto.start_bit |= 1;
					}
					else {
						ir_proto.err_corr_bit = 0;
						ir_proto.err_corr_bit_len = 0;
						ir_proto.state = ERR_CORR_WAIT;
						_debug();
					}
				}
				else {
					ir_proto.start_bit = 0;
					ir_proto.state = START_BIT_WAIT;
				}
				break;
			case ERR_CORR_WAIT:
			//	if (ir_proto.data_len++ < 8) {
					if ((TICK_LOW < timer_0) && (timer_0 < TICK_HIGH)) {
						_debug();
						//ir_proto.start_bit = 0b111;
						//ir_proto.state = INIT_STATE;//ERR_CORR_1;
					}
					else if ((2 * TICK_LOW < timer_0) && (timer_0 < 2 * TICK_HIGH)) {
						_debug();
						_debug();
						//ir_proto.state = INIT_STATE;//ERR_CORR_1;
					}
					else if ((3 * TICK_LOW < timer_0) && (timer_0 < 3 * TICK_HIGH)) {
						_debug();
						_debug();
						_debug();
						//ir_proto.state = INIT_STATE;//ERR_CORR_1;
					}
					else {
						//ir_proto.start_bit = 0b000;
						ir_proto.state = INIT_STATE;//START_BIT_WAIT_1;
					}
			//	}
			//	else {
					//ir_proto.start_bit = 0b000;
			//		ir_proto.state = INIT_STATE;//START_BIT_WAIT_1;
			//	}
				break;
		}

				
		INTCONbits.INT0IF = 0;	/* Clear Interrupt Flag */
	}
	if (INTCONbits.TMR0IF) {
		// if timer overflow occurs - reset state
		ir_proto.start_bit = 0;
		ir_proto.state = INIT_STATE;
		
		INTCONbits.TMR0IF = 0;
	}
}

static void isr_low_prio(void) __interrupt 2 {
	unsigned char c;

	if (PIR1bits.TMR1IF) {
		TMR1H = (unsigned char)(TIMER1_RELOAD >> 8);    // 262,158ms @ 8MHz
		TMR1L = (unsigned char)TIMER1_RELOAD;
		PIR1bits.TMR1IF = 0;    /* Clear the Timer Flag  */
		timer_0_ms++;
	}

	// serial rx interrupt
	if (usart_drdy()) {
		// retransmit it
		c = usart_getc();
		usart_putc(c);
	}

}

void sleep_ms(unsigned long ms) {
	unsigned long start_timer_0_ms;
	start_timer_0_ms = timer_0_ms;	

// while the absolute value of the time diff < ms
	while ( (((signed long)(timer_0_ms - start_timer_0_ms) < 0) ? (-1 * (timer_0_ms - start_timer_0_ms)) : (timer_0_ms - start_timer_0_ms)) < ms) {
		// do nothing
	}
}

void init_system() {
	// timer 0
	T0CONbits.TMR0ON = 1;
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

    // set up interrupt and timers
    RCONbits.IPEN = 1;
	
	INTCONbits.INT0IE = 1;		// enable ext int
	INTCON2bits.INTEDG0 = 0;	// on falling edge

	INTCONbits.PEIE = 1;
	INTCONbits.GIE = 1;	/* Enable Global interrupts   */	
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

void _debug() {
	PORTDbits.RD4 = 0x1;
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
	PORTDbits.RD4 = 0x0;
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