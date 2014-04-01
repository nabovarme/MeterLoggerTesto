//
//  main.c
//  ir_proto_test
//
//  Created by stoffer on 30/03/14.
//  Copyright (c) 2014 stoffer. All rights reserved.
//

#include <stdio.h>

#define DEBUG_PHASE_SHIFT_DECODED

#define TICK 10
#define TICK_ADJ 1
#define TICK_LOW (TICK - TICK_ADJ)
#define TICK_HIGH (TICK + TICK_ADJ)

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
	unsigned char err_corr_bits;
	unsigned char err_corr_bits_len;
	unsigned char data;
	unsigned char data_len;
	
} ir_proto_t;

volatile ir_proto_t ir_proto;
//unsigned int timer_0;

void isr_high_prio(unsigned int timer_0) {
    
    switch (ir_proto.state) {
        case INIT_STATE:
            ir_proto.start_bit_len = 1;
            ir_proto.state = START_BIT_WAIT;
            //_debug();
            break;
        case START_BIT_WAIT:
            if ((TICK_LOW < timer_0) && (timer_0 < TICK_HIGH)) {
                if (ir_proto.start_bit_len < 2) {
                    //_debug();
                    ir_proto.start_bit_len++;
                }
                else {
                    ir_proto.data = 0;
                    ir_proto.data_len = 0;
                    ir_proto.state = DATA_WAIT;
#ifdef DEBUG_PHASE_SHIFT_DECODED
                    printf("0");
#endif
                }
            }
            else {
                ir_proto.start_bit_len = 1;
                ir_proto.state = START_BIT_WAIT;
            }
            break;
        case DATA_WAIT:
            if (ir_proto.data_len < 11) {
                if (((TICK_LOW < timer_0) && (timer_0 < TICK_HIGH)) || ((3 * TICK_LOW < timer_0) && (timer_0 < 3 * TICK_HIGH))) {
                    // phase shift
                    if ((ir_proto.data & 1) != 0) {
                        // previous bit is set
                        ir_proto.data = ir_proto.data << 1;		// bitshift once to left
                        ir_proto.data &= 0b111111111110;	// and clear bit 0
#ifdef DEBUG_PHASE_SHIFT_DECODED
                        printf("0");
#endif
                    }
                    else {
                        // previous bit is zero
                        ir_proto.data = ir_proto.data << 1;		// bitshift once to left
                        ir_proto.data |= 0b0000000000001;	// and set bit 0
#ifdef DEBUG_PHASE_SHIFT_DECODED
                        printf("1");
#endif
                    }
                }
                else if ((2 * TICK_LOW < timer_0) && (timer_0 < 2 * TICK_HIGH)) {
                    // in phase
                    if ((ir_proto.data & 1) != 0) {
                        // previous bit is set
                        ir_proto.data = ir_proto.data << 1;		// bitshift once to left
                        ir_proto.data |= 0b0000000000001;	// and set bit 0
#ifdef DEBUG_PHASE_SHIFT_DECODED
                        printf("1");
#endif
                    }
                    else {
                        // previous bit is zero
                        ir_proto.data = ir_proto.data << 1;		// bitshift once to left
                        ir_proto.data &= 0b111111111110;	// and clear bit 0
#ifdef DEBUG_PHASE_SHIFT_DECODED
                        printf("0");
#endif
                    }
                }
                else {
                    ir_proto.state = INIT_STATE;
                }
                ir_proto.data_len++;
            }
            else {
                // frame received!
                // calculate error correction and send via serial port
                //usart_putc((unsigned char)(ir_proto.data & 0xff));
				//	usart_putc(0x5a);
                ir_proto.state = INIT_STATE;
            }
            break;
		}
        
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
    
    if ((c >> 8) == calculated_err_corr) {
        return 1;
    }
    else {
        return 0;
    }

}

int main(int argc, const char * argv[])
{

/*
    isr_high_prio(10);
    isr_high_prio(10);
    isr_high_prio(10);
    
    isr_high_prio(20);
    isr_high_prio(10);
    isr_high_prio(20);
    isr_high_prio(20);

    isr_high_prio(10);
    isr_high_prio(10);
    isr_high_prio(20);
    isr_high_prio(20);
    isr_high_prio(10);
    isr_high_prio(20);
    isr_high_prio(20);
    isr_high_prio(20);
    return 0;
 */
    if (valid_err_corr(0x0d41)) {
        printf("yes");
    }
}

