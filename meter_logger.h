#define DEBUG_BUFFER_MAX (128)

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

#define sleep() __asm sleep __endasm

//#define TIMER0_RELOAD 0xFCC8	// 427us @ 8MHz
//#define TIMER0_RELOAD 0xF000	// ~ 2ms @ 8MHz
#define TIMER0_TESTO			 0xF323	// ~ 4 * 427us @ 8MHz
#define TIMER0_RS232_300		0xe61b	// ~ 3.3ms @ 8 MHz
#define TIMER0_RS232_1200		0xf9ae
#define TIMER0_RS232_1200_START	0xf700
#define TIMER0_FSK				0xF99F	// @ 8MHz
//#define TIMER0_FSK			0xF00F
//#define TIMER0_RELOAD 0x0000
#define TIMER1_RELOAD 0xF853    // 1 ms @ 8MHz
#define TIMER2_RELOAD	0x00	// ~ 5 ms @ 8Mhz
#define TIMER3_RELOAD	0x0000	// ~ 131 ms @ 4MHz

#define SOFT_MODEM_BAUD_RATE	(1225)
#define SOFT_MODEM_LOW_FREQ		(4900)
#define SOFT_MODEM_HIGH_FREQ	(7350)
		
#define FSK_TX_SLEEP			(2)
#define RS232_TX_SLEEP			(12)

#define OUTPUT_STATE (0)
#define INPUT_STATE (1)

#define TRIS_COMP1		TRISAbits.RA0
#define TRIS_COMP2		TRISAbits.RA1
#define COMP1_PIN		PORTAbits.RA0
#define COMP2_PIN		PORTAbits.RA1

#define TRIS_V_SENSE	TRISAbits.RA5

#define TRIS_IR_PIN		TRISBbits.RB0
#define IR_PIN			PORTBbits.RB0

#define TRIS_LED_PIN	TRISBbits.RB4
#define LED_PIN			PORTBbits.RB4

#define TRIS_IR_LED_PIN	TRISBbits.RB1 
#define IR_LED_PIN		PORTBbits.RB1

#define TRIS_DEBUG_PIN	TRISBbits.RB2
#define DEBUG_PIN		PORTBbits.RB2

#define TRIS_DEBUG2_PIN	TRISBbits.RB3
#define DEBUG2_PIN		PORTBbits.RB3

#define TRIS_DEBUG3_PIN	TRISBbits.RB4
#define DEBUG3_PIN		PORTBbits.RB4

#define TRIS_PWM_PIN	TRISCbits.RC1
#define PWM_PIN			PORTCbits.RC1

#define TRIS_RX_PIN		TRISCbits.RC7
#define RX_PIN			PORTCbits.RC7

#define TRIS_TX_PIN		TRISCbits.RC6
#define TX_PIN			PORTCbits.RC6

#define TICK 855
#define TICK_ADJ 200

#define PROTO_TESTO				(254)
#define PROTO_TESTO_DEMO		(255)
#define PROTO_KAMSTRUP			(253)
#define PROTO_KAMSTRUP_MULTICAL	(252)
#define PROTO_BATTERY_LEVEL		(251)

void sleep_ms(unsigned int ms);

void init_system();

unsigned int get_dev_id();

void my_usart_open();

unsigned char reverse(unsigned char b);

unsigned char testo_valid_err_corr(unsigned int c);

void testo_ir_enable();
void testo_ir_disable();

void rs232_8n2_tx_enable(unsigned int t);
void rs232_8n2_tx_disable();

void rs232_8n2_rx_enable(unsigned int t);
void rs232_8n2_rx_disable();
void rs232_8n2_tx_byte(unsigned char c);

void rs232_7e1_tx_enable(unsigned int t);
void rs232_7e1_tx_disable();

void rs232_7e1_rx_enable(unsigned int t);
void rs232_7e1_rx_disable();
void rs232_7e1_tx_byte(unsigned char c);

void fsk_tx_enable();
void fsk_tx_disable();

void fsk_rx_enable();
void fsk_rx_disable();

void send_fsk_high(void);
void send_fsk_low(void);
void fsk_tx_byte(unsigned char c);

unsigned int fifo_in_use();
unsigned char fifo_put(unsigned char c);
unsigned char fifo_get(unsigned char  *c);
unsigned char fifo_snoop(unsigned char *c, unsigned int pos);

unsigned int get_battery_level();

void flash_led(unsigned char ms);

void _debug();
void _debug2();

