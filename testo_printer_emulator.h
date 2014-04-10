#define BYTETOBINARYPATTERN "%d%d%d%d%d%d%d%d"
#define BYTETOBINARY(byte)  \
	(byte & 0x80 ? 1 : 0), \
	(byte & 0x40 ? 1 : 0), \
	(byte & 0x20 ? 1 : 0), \
	(byte & 0x10 ? 1 : 0), \
	(byte & 0x08 ? 1 : 0), \
	(byte & 0x04 ? 1 : 0), \
	(byte & 0x02 ? 1 : 0), \
	(byte & 0x01 ? 1 : 0)

//#define TIMER0_RELOAD 0xFCC8	// 427us @ 8MHz
//#define TIMER0_RELOAD 0xF000	// ~ 2ms @ 8MHz
#define TIMER0_RELOAD 0xF323	// ~ 4 * 427us @ 8MHz
//#define TIMER0_RELOAD 0x0000
#define TIMER1_RELOAD 0xF853    // 1 ms @ 8MHz
#define TIMER2_RELOAD	0x00	// ~ 5 ms @ 8Mhz
#define TIMER3_RELOAD	0x0000	// ~ 131 ms @ 4MHz

#define TICK 855
#define TICK_ADJ 200

void sleep_ms(unsigned long ms);

void init_system();

void my_usart_open();

unsigned char reverse(unsigned char b);

unsigned char valid_err_corr(unsigned int c);

void _debug();
