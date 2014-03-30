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
						
						//ir_proto.start_bit = (ir_proto.start_bit << 1) & 0b111;
						//ir_proto.start_bit |= 1;
						ir_proto.start_bit_len++;
					}
					else {
						ir_proto.err_corr_bits = 0;
						ir_proto.err_corr_bits_len = 0;
						ir_proto.state = ERR_CORR_WAIT;
						printf("start bit received\n");
					}
				}
				else {
					ir_proto.start_bit_len = 1;
					ir_proto.state = START_BIT_WAIT;
				}
				break;
			case ERR_CORR_WAIT:
				if (ir_proto.err_corr_bits_len < 4) {
					if (((TICK_LOW < timer_0) && (timer_0 < TICK_HIGH)) || ((3 * TICK_LOW < timer_0) && (timer_0 < 3 * TICK_HIGH))) {
						// phase shift
						if ((ir_proto.err_corr_bits & 1) != 0) {
							// previous bit is set
							ir_proto.err_corr_bits << 1;		// bitshift once to left
							ir_proto.err_corr_bits &= 0b1110;	// and clear bit 0
#ifdef DEBUG_PHASE_SHIFT_DECODED
							printf("0");
#endif
						}
						else {
							// previous bit is zero
							ir_proto.err_corr_bits << 1;		// bitshift once to left
							ir_proto.err_corr_bits |= 0b0001;	// and set bit 0
#ifdef DEBUG_PHASE_SHIFT_DECODED
							printf("1");
#endif
						}
					}
					else if ((2 * TICK_LOW < timer_0) && (timer_0 < 2 * TICK_HIGH)) {
						// in phase
						if ((ir_proto.err_corr_bits & 1) != 0) {
							// previous bit is set
							ir_proto.err_corr_bits << 1;		// bitshift once to left
							ir_proto.err_corr_bits |= 0b0001;	// and set bit 0
#ifdef DEBUG_PHASE_SHIFT_DECODED
							printf("1");
#endif
						}
						else {
							// previous bit is zero
							ir_proto.err_corr_bits << 1;		// bitshift once to left
							ir_proto.err_corr_bits &= 0b1110;	// and clear bit 0
#ifdef DEBUG_PHASE_SHIFT_DECODED
							printf("0");
#endif
						}
					}
					else {
						ir_proto.state = INIT_STATE;
					}
					ir_proto.err_corr_bits_len++;
				}
				else {
					//ir_proto.data = 0x00;
					ir_proto.state = INIT_STATE;//DATA_WAIT;
                    printf("\n");
				}
				break;
		}
        
}

int main(int argc, const char * argv[])
{

    // insert code here...
    isr_high_prio(10);
    isr_high_prio(10);
    isr_high_prio(10);
    isr_high_prio(20);
    isr_high_prio(10);
    isr_high_prio(20);
    isr_high_prio(20);
    return 0;
}

