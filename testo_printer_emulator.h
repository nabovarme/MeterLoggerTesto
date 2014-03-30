#define TIMER0_RELOAD 0xFCC8	// 427us @ 8MHz
#define TIMER1_RELOAD 0xF853    // 1 ms @ 8MHz
#define TIMER2_RELOAD	0x00	// ~ 5 ms @ 8Mhz
#define TIMER3_RELOAD	0x0000	// ~ 131 ms @ 4MHz

#define TICK 855
#define TICK_ADJ 50
#define TICK_LOW (TICK - TICK_ADJ)
#define TICK_HIGH (TICK + TICK_ADJ)

void sleep_ms(unsigned long ms);

void init_system();

void my_usart_open();

void _debug();
