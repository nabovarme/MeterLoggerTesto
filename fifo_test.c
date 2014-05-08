#include <pic18fregs.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <usart.h>
#include "config.h"
#include "testo_printer_emulator.h"

#define DEBUG

#define QUEUE_SIZE 256
#define QUEUE_SIZE_COMBINED (4 * QUEUE_SIZE)

// Global variables
unsigned long timer_1_ms;
volatile unsigned int timer0_reload;
unsigned char buffer[64];

// command queue
unsigned int fifo_head, fifo_tail;
unsigned char fifo_buffer_0[QUEUE_SIZE];
unsigned char fifo_buffer_1[QUEUE_SIZE];
unsigned char fifo_buffer_2[QUEUE_SIZE];
unsigned char fifo_buffer_3[QUEUE_SIZE];
unsigned char c;


void main(void) {
	unsigned int i;
	unsigned char foo;
	
	unsigned char *ptr;
    OSCCONbits.SCS = 0x10;
//    OSCCONbits.SCS = 0x00;	// external osc
    OSCCONbits.IRCF = 0x7;	// 8 MHz

	fifo_head = 0;
	fifo_tail = 0;
	
	init_system();

#ifdef DEBUG
//	usart_puts("Testo printer emulator... serial working\n\r");
#endif
//	ptr = &fifo_buffer_0[0];
	for (i = 0; i < 1024; i++) {
		//fifo_buffer_0[i] = 'a';
		fifo_put('a');
		//sprintf(buffer, "%c \n", fifo_buffer_0[i]);
		//usart_puts(buffer);
		}
	/*
	fifo_buffer_0[0] = 'a';
	fifo_buffer_0[1] = 'b';
	fifo_buffer_0[2] = 'c';
	fifo_buffer_0[3] = 'd';
	sprintf(buffer, "%c \n", fifo_buffer_0[0]);
	usart_puts(buffer);
	sprintf(buffer, "%c \n", fifo_buffer_0[1]);
	usart_puts(buffer);
	sprintf(buffer, "%c \n", fifo_buffer_0[2]);
	usart_puts(buffer);
	sprintf(buffer, "%c \n", fifo_buffer_0[3]);
	usart_puts(buffer);
	*/
//	fifo_put('0');
//	fifo_put('1');
//	fifo_put('2');
//	fifo_put('a');
	
	while (1) {
		if (fifo_get(&foo)) {
			sprintf(buffer, "%c \n", foo);
			usart_puts(buffer);
			//usart_putc(foo);
		}
	}
}

void init_system() {
	// serial
	TRIS_RX_PIN = INPUT_STATE;		// as input
	TRIS_TX_PIN = OUTPUT_STATE;		// as input

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
