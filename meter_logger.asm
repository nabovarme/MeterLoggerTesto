;--------------------------------------------------------
; File Created by SDCC : free open source ANSI-C Compiler
; Version 3.4.0 #8981 (Jun  6 2014) (Mac OS X x86_64)
; This file was generated Wed Jun 11 21:04:41 2014
;--------------------------------------------------------
; PIC16 port for the Microchip 16-bit core micros
;--------------------------------------------------------
	list	p=18f2550
	radix	dec
	CONFIG	PLLDIV=2
	CONFIG	CPUDIV=OSC1_PLL2
	CONFIG	USBDIV=1
	CONFIG	FOSC=INTOSC_XT
	CONFIG	FCMEN=OFF
	CONFIG	IESO=OFF
	CONFIG	PWRT=OFF
	CONFIG	BOR=OFF
	CONFIG	BORV=3
	CONFIG	VREGEN=OFF
	CONFIG	WDT=OFF
	CONFIG	WDTPS=32768
	CONFIG	CCP2MX=ON
	CONFIG	PBADEN=OFF
	CONFIG	LPT1OSC=OFF
	CONFIG	MCLRE=ON
	CONFIG	STVREN=ON
	CONFIG	LVP=ON
	CONFIG	XINST=OFF
	CONFIG	CP0=OFF
	CONFIG	CP1=OFF
	CONFIG	CP2=OFF
	CONFIG	CP3=OFF
	CONFIG	CPB=OFF
	CONFIG	CPD=OFF
	CONFIG	WRT0=OFF
	CONFIG	WRT1=OFF
	CONFIG	WRT2=OFF
	CONFIG	WRT3=OFF
	CONFIG	WRTC=OFF
	CONFIG	WRTB=OFF
	CONFIG	WRTD=OFF
	CONFIG	EBTR0=OFF
	CONFIG	EBTR1=OFF
	CONFIG	EBTR2=OFF
	CONFIG	EBTR3=OFF
	CONFIG	EBTRB=OFF


;--------------------------------------------------------
; public variables in this module
;--------------------------------------------------------
	global	_c
	global	_debug_buffer
	global	_sleep_ms
	global	_init_system
	global	_my_usart_open
	global	_reverse
	global	_testo_valid_err_corr
	global	_testo_ir_enable
	global	_testo_ir_disable
	global	_rs232_tx_enable
	global	_rs232_tx_disable
	global	_rs232_rx_enable
	global	_rs232_rx_disable
	global	_rs232_tx_byte
	global	_fsk_tx_enable
	global	_fsk_tx_disable
	global	_fsk_rx_enable
	global	_fsk_rx_disable
	global	_send_fsk_high
	global	_send_fsk_low
	global	_fsk_tx_byte
	global	_fifo_in_use
	global	_fifo_put
	global	_fifo_get
	global	_fifo_snoop
	global	_flash_led
	global	__debug
	global	__debug2
	global	_timer_0
	global	_last_timer_0
	global	_timer_1_ms
	global	_timer0_reload
	global	_fifo_head
	global	_fifo_tail
	global	_fifo_buffer_0
	global	_fifo_buffer_1
	global	_fifo_buffer_2
	global	_fifo_buffer_3
	global	_codec_type
	global	_led_flash
	global	_testo_ir_proto
	global	_rs232_proto
	global	_fsk_proto
	global	_main

;--------------------------------------------------------
; extern variables in this module
;--------------------------------------------------------
	extern	__gptrput1
	extern	_UFRMLbits
	extern	_UFRMHbits
	extern	_UIRbits
	extern	_UIEbits
	extern	_UEIRbits
	extern	_UEIEbits
	extern	_USTATbits
	extern	_UCONbits
	extern	_UADDRbits
	extern	_UCFGbits
	extern	_UEP0bits
	extern	_UEP1bits
	extern	_UEP2bits
	extern	_UEP3bits
	extern	_UEP4bits
	extern	_UEP5bits
	extern	_UEP6bits
	extern	_UEP7bits
	extern	_UEP8bits
	extern	_UEP9bits
	extern	_UEP10bits
	extern	_UEP11bits
	extern	_UEP12bits
	extern	_UEP13bits
	extern	_UEP14bits
	extern	_UEP15bits
	extern	_PORTAbits
	extern	_PORTBbits
	extern	_PORTCbits
	extern	_PORTEbits
	extern	_LATAbits
	extern	_LATBbits
	extern	_LATCbits
	extern	_DDRAbits
	extern	_TRISAbits
	extern	_DDRBbits
	extern	_TRISBbits
	extern	_DDRCbits
	extern	_TRISCbits
	extern	_OSCTUNEbits
	extern	_PIE1bits
	extern	_PIR1bits
	extern	_IPR1bits
	extern	_PIE2bits
	extern	_PIR2bits
	extern	_IPR2bits
	extern	_EECON1bits
	extern	_RCSTAbits
	extern	_TXSTAbits
	extern	_T3CONbits
	extern	_CMCONbits
	extern	_CVRCONbits
	extern	_CCP1ASbits
	extern	_ECCP1ASbits
	extern	_CCP1DELbits
	extern	_ECCP1DELbits
	extern	_BAUDCONbits
	extern	_BAUDCTLbits
	extern	_CCP2CONbits
	extern	_CCP1CONbits
	extern	_ADCON2bits
	extern	_ADCON1bits
	extern	_ADCON0bits
	extern	_SSPCON2bits
	extern	_SSPCON1bits
	extern	_SSPSTATbits
	extern	_T2CONbits
	extern	_T1CONbits
	extern	_RCONbits
	extern	_WDTCONbits
	extern	_HLVDCONbits
	extern	_LVDCONbits
	extern	_OSCCONbits
	extern	_T0CONbits
	extern	_STATUSbits
	extern	_INTCON3bits
	extern	_INTCON2bits
	extern	_INTCONbits
	extern	_STKPTRbits
	extern	_stdin
	extern	_stdout
	extern	_UFRM
	extern	_UFRML
	extern	_UFRMH
	extern	_UIR
	extern	_UIE
	extern	_UEIR
	extern	_UEIE
	extern	_USTAT
	extern	_UCON
	extern	_UADDR
	extern	_UCFG
	extern	_UEP0
	extern	_UEP1
	extern	_UEP2
	extern	_UEP3
	extern	_UEP4
	extern	_UEP5
	extern	_UEP6
	extern	_UEP7
	extern	_UEP8
	extern	_UEP9
	extern	_UEP10
	extern	_UEP11
	extern	_UEP12
	extern	_UEP13
	extern	_UEP14
	extern	_UEP15
	extern	_PORTA
	extern	_PORTB
	extern	_PORTC
	extern	_PORTE
	extern	_LATA
	extern	_LATB
	extern	_LATC
	extern	_DDRA
	extern	_TRISA
	extern	_DDRB
	extern	_TRISB
	extern	_DDRC
	extern	_TRISC
	extern	_OSCTUNE
	extern	_PIE1
	extern	_PIR1
	extern	_IPR1
	extern	_PIE2
	extern	_PIR2
	extern	_IPR2
	extern	_EECON1
	extern	_EECON2
	extern	_EEDATA
	extern	_EEADR
	extern	_RCSTA
	extern	_TXSTA
	extern	_TXREG
	extern	_RCREG
	extern	_SPBRG
	extern	_SPBRGH
	extern	_T3CON
	extern	_TMR3
	extern	_TMR3L
	extern	_TMR3H
	extern	_CMCON
	extern	_CVRCON
	extern	_CCP1AS
	extern	_ECCP1AS
	extern	_CCP1DEL
	extern	_ECCP1DEL
	extern	_BAUDCON
	extern	_BAUDCTL
	extern	_CCP2CON
	extern	_CCPR2
	extern	_CCPR2L
	extern	_CCPR2H
	extern	_CCP1CON
	extern	_CCPR1
	extern	_CCPR1L
	extern	_CCPR1H
	extern	_ADCON2
	extern	_ADCON1
	extern	_ADCON0
	extern	_ADRES
	extern	_ADRESL
	extern	_ADRESH
	extern	_SSPCON2
	extern	_SSPCON1
	extern	_SSPSTAT
	extern	_SSPADD
	extern	_SSPBUF
	extern	_T2CON
	extern	_PR2
	extern	_TMR2
	extern	_T1CON
	extern	_TMR1
	extern	_TMR1L
	extern	_TMR1H
	extern	_RCON
	extern	_WDTCON
	extern	_HLVDCON
	extern	_LVDCON
	extern	_OSCCON
	extern	_T0CON
	extern	_TMR0
	extern	_TMR0L
	extern	_TMR0H
	extern	_STATUS
	extern	_FSR2L
	extern	_FSR2H
	extern	_PLUSW2
	extern	_PREINC2
	extern	_POSTDEC2
	extern	_POSTINC2
	extern	_INDF2
	extern	_BSR
	extern	_FSR1L
	extern	_FSR1H
	extern	_PLUSW1
	extern	_PREINC1
	extern	_POSTDEC1
	extern	_POSTINC1
	extern	_INDF1
	extern	_WREG
	extern	_FSR0L
	extern	_FSR0H
	extern	_PLUSW0
	extern	_PREINC0
	extern	_POSTDEC0
	extern	_POSTINC0
	extern	_INDF0
	extern	_INTCON3
	extern	_INTCON2
	extern	_INTCON
	extern	_PROD
	extern	_PRODL
	extern	_PRODH
	extern	_TABLAT
	extern	_TBLPTR
	extern	_TBLPTRL
	extern	_TBLPTRH
	extern	_TBLPTRU
	extern	_PC
	extern	_PCL
	extern	_PCLATH
	extern	_PCLATU
	extern	_STKPTR
	extern	_TOS
	extern	_TOSL
	extern	_TOSH
	extern	_TOSU
	extern	_usart_drdy
	extern	_usart_getc
	extern	_usart_putc
	extern	_usart_puts
	extern	__mullong

;--------------------------------------------------------
;	Equates to used internal registers
;--------------------------------------------------------
STATUS	equ	0xfd8
PCL	equ	0xff9
PCLATH	equ	0xffa
PCLATU	equ	0xffb
WREG	equ	0xfe8
BSR	equ	0xfe0
FSR0L	equ	0xfe9
FSR0H	equ	0xfea
FSR1L	equ	0xfe1
FSR2L	equ	0xfd9
INDF0	equ	0xfef
POSTINC1	equ	0xfe6
POSTDEC1	equ	0xfe5
PREINC1	equ	0xfe4
PLUSW2	equ	0xfdb
PRODL	equ	0xff3
PRODH	equ	0xff4


; Internal registers
.registers	udata_ovr	0x0000
r0x00	res	1
r0x01	res	1
r0x02	res	1
r0x03	res	1
r0x04	res	1
r0x05	res	1
r0x06	res	1
r0x07	res	1
r0x08	res	1
r0x09	res	1
r0x0a	res	1
r0x0b	res	1
r0x0c	res	1
r0x0d	res	1
r0x0e	res	1
r0x0f	res	1

udata_meter_logger_0	udata
_debug_buffer	res	128

udata_meter_logger_1	udata
_c	res	1

udata_meter_logger_2	udata
_timer_1_ms	res	4

udata_meter_logger_3	udata
_fifo_head	res	2

udata_meter_logger_4	udata
_fifo_tail	res	2

udata_meter_logger_5	udata
_main_cmd_1_79	res	1

udata_meter_logger_6	udata
_main_sub_cmd_1_79	res	1

udata_meter_logger_7	udata
_timer_0	res	2

udata_meter_logger_8	udata
_timer0_reload	res	2

udata_meter_logger_9	udata
_testo_ir_proto	res	6

udata_meter_logger_10	udata
_rs232_proto	res	6

udata_meter_logger_11	udata
_fsk_proto	res	16

udata_meter_logger_12	udata
_last_timer_0	res	2

udata_meter_logger_13	udata
_codec_type	res	1

udata_meter_logger_14	udata
_led_flash	res	2

udata_meter_logger_15	udata
_fifo_buffer_0	res	256

udata_meter_logger_16	udata
_fifo_buffer_1	res	256

udata_meter_logger_17	udata
_fifo_buffer_2	res	256

udata_meter_logger_18	udata
_fifo_buffer_3	res	256

;--------------------------------------------------------
; interrupt vector
;--------------------------------------------------------

;--------------------------------------------------------
; global & static initialisations
;--------------------------------------------------------
; ; Starting pCode block for absolute section
; ;-----------------------------------------
S_meter_logger_ivec_0x1_isr_high_prio	code	0X000008
ivec_0x1_isr_high_prio:
	GOTO	_isr_high_prio

; ; Starting pCode block for absolute section
; ;-----------------------------------------
S_meter_logger_ivec_0x2_isr_low_prio	code	0X000018
ivec_0x2_isr_low_prio:
	GOTO	_isr_low_prio

; I code from now on!
; ; Starting pCode block
S_meter_logger__main	code
_main:
;	.line	108; meter_logger.c	OSCCONbits.SCS = 0x10;
	MOVF	_OSCCONbits, W
	ANDLW	0xfc
	MOVWF	_OSCCONbits
;	.line	110; meter_logger.c	OSCCONbits.IRCF = 0x7;	// 8 MHz
	MOVF	_OSCCONbits, W
	ANDLW	0x8f
	IORLW	0x70
	MOVWF	_OSCCONbits
	BANKSEL	_timer_1_ms
;	.line	113; meter_logger.c	timer_1_ms = 0;
	CLRF	_timer_1_ms, B
; removed redundant BANKSEL
	CLRF	(_timer_1_ms + 1), B
; removed redundant BANKSEL
	CLRF	(_timer_1_ms + 2), B
; removed redundant BANKSEL
	CLRF	(_timer_1_ms + 3), B
	BANKSEL	_fifo_head
;	.line	115; meter_logger.c	fifo_head = 0;
	CLRF	_fifo_head, B
; removed redundant BANKSEL
	CLRF	(_fifo_head + 1), B
	BANKSEL	_fifo_tail
;	.line	116; meter_logger.c	fifo_tail = 0;
	CLRF	_fifo_tail, B
; removed redundant BANKSEL
	CLRF	(_fifo_tail + 1), B
;	.line	118; meter_logger.c	init_system();
	CALL	_init_system
;	.line	120; meter_logger.c	usart_puts("\n\rMeterLogger... serial working\n\r");
	MOVLW	UPPER(___str_0)
	MOVWF	r0x02
	MOVLW	HIGH(___str_0)
	MOVWF	r0x01
	MOVLW	LOW(___str_0)
	MOVWF	r0x00
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	_usart_puts
	MOVLW	0x03
	ADDWF	FSR1L, F
;	.line	121; meter_logger.c	sleep_ms(100);
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x64
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVLW	0x04
	ADDWF	FSR1L, F
;	.line	123; meter_logger.c	fsk_rx_enable();
	CALL	_fsk_rx_enable
_00139_DS_:
;	.line	125; meter_logger.c	if (fifo_get(&cmd)) {
	MOVLW	HIGH(_main_cmd_1_79)
	MOVWF	r0x01
	MOVLW	LOW(_main_cmd_1_79)
	MOVWF	r0x00
	MOVLW	0x80
	MOVWF	r0x02
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	_fifo_get
	MOVWF	r0x00
	MOVLW	0x03
	ADDWF	FSR1L, F
	MOVF	r0x00, W
	BZ	_00139_DS_
;	.line	126; meter_logger.c	switch (cmd) {
	MOVLW	0xfd
	BANKSEL	_main_cmd_1_79
	SUBWF	_main_cmd_1_79, W, B
	BNC	_00139_DS_
	MOVLW	0x03
; removed redundant BANKSEL
	ADDWF	_main_cmd_1_79, W, B
	MOVWF	r0x00
	CLRF	PCLATH
	CLRF	PCLATU
	RLCF	r0x00, W
	RLCF	PCLATH, F
	RLCF	WREG, W
	RLCF	PCLATH, F
	ANDLW	0xfc
	ADDLW	LOW(_00204_DS_)
	MOVWF	POSTDEC1
	MOVLW	HIGH(_00204_DS_)
	ADDWFC	PCLATH, F
	MOVLW	UPPER(_00204_DS_)
	ADDWFC	PCLATU, F
	MOVF	PREINC1, W
	MOVWF	PCL
_00204_DS_:
	GOTO	_00119_DS_
	GOTO	_00105_DS_
	GOTO	_00112_DS_
_00105_DS_:
;	.line	128; meter_logger.c	fsk_rx_disable();
	CALL	_fsk_rx_disable
;	.line	129; meter_logger.c	usart_puts("\n\rpress print on testo\n\r");
	MOVLW	UPPER(___str_1)
	MOVWF	r0x02
	MOVLW	HIGH(___str_1)
	MOVWF	r0x01
	MOVLW	LOW(___str_1)
	MOVWF	r0x00
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	_usart_puts
	MOVLW	0x03
	ADDWF	FSR1L, F
;	.line	130; meter_logger.c	testo_ir_enable();
	CALL	_testo_ir_enable
;	.line	132; meter_logger.c	last_fifo_size = 0;
	CLRF	r0x00
	CLRF	r0x01
;	.line	133; meter_logger.c	sleep_ms(10000);						// 10 seconds to start printing
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x27
	MOVWF	POSTDEC1
	MOVLW	0x10
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVLW	0x04
	ADDWF	FSR1L, F
;	.line	134; meter_logger.c	fifo_size = fifo_in_use();
	CALL	_fifo_in_use
	MOVWF	r0x02
	MOVFF	PRODL, r0x03
_00106_DS_:
;	.line	135; meter_logger.c	while (fifo_size > last_fifo_size) {	// and wait while we are still receiving data
	MOVF	r0x03, W
	SUBWF	r0x01, W
	BNZ	_00205_DS_
	MOVF	r0x02, W
	SUBWF	r0x00, W
_00205_DS_:
	BC	_00108_DS_
;	.line	136; meter_logger.c	last_fifo_size = fifo_size;
	MOVFF	r0x02, r0x00
	MOVFF	r0x03, r0x01
;	.line	137; meter_logger.c	sleep_ms(200);						// return data when no data for 200 ms
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0xc8
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVLW	0x04
	ADDWF	FSR1L, F
;	.line	138; meter_logger.c	fifo_size = fifo_in_use();
	CALL	_fifo_in_use
	MOVWF	r0x02
	MOVFF	PRODL, r0x03
	BRA	_00106_DS_
_00108_DS_:
;	.line	140; meter_logger.c	testo_ir_disable();
	CALL	_testo_ir_disable
;	.line	141; meter_logger.c	usart_puts("\n\rdone receiving - sending via serial/fsk\n\r");
	MOVLW	UPPER(___str_2)
	MOVWF	r0x06
	MOVLW	HIGH(___str_2)
	MOVWF	r0x05
	MOVLW	LOW(___str_2)
	MOVWF	r0x04
	MOVF	r0x06, W
	MOVWF	POSTDEC1
	MOVF	r0x05, W
	MOVWF	POSTDEC1
	MOVF	r0x04, W
	MOVWF	POSTDEC1
	CALL	_usart_puts
	MOVLW	0x03
	ADDWF	FSR1L, F
;	.line	143; meter_logger.c	fsk_tx_enable();
	CALL	_fsk_tx_enable
;	.line	144; meter_logger.c	sleep_ms(FSK_TX_SLEEP);
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x0c
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVLW	0x04
	ADDWF	FSR1L, F
_00109_DS_:
;	.line	146; meter_logger.c	while (fifo_get(&cmd)) {	// and print them to serial
	MOVLW	HIGH(_main_cmd_1_79)
	MOVWF	r0x05
	MOVLW	LOW(_main_cmd_1_79)
	MOVWF	r0x04
	MOVLW	0x80
	MOVWF	r0x06
	MOVF	r0x06, W
	MOVWF	POSTDEC1
	MOVF	r0x05, W
	MOVWF	POSTDEC1
	MOVF	r0x04, W
	MOVWF	POSTDEC1
	CALL	_fifo_get
	MOVWF	r0x04
	MOVLW	0x03
	ADDWF	FSR1L, F
	MOVF	r0x04, W
	BZ	_00111_DS_
	BANKSEL	_main_cmd_1_79
;	.line	151; meter_logger.c	fsk_tx_byte(cmd);
	MOVF	_main_cmd_1_79, W, B
	MOVWF	POSTDEC1
	CALL	_fsk_tx_byte
	MOVF	POSTINC1, F
;	.line	152; meter_logger.c	sleep_ms(FSK_TX_SLEEP);
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x0c
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVLW	0x04
	ADDWF	FSR1L, F
	BRA	_00109_DS_
_00111_DS_:
;	.line	156; meter_logger.c	fsk_tx_disable();
	CALL	_fsk_tx_disable
;	.line	158; meter_logger.c	usart_puts("\n\rwaiting for new command\n\r");
	MOVLW	UPPER(___str_3)
	MOVWF	r0x06
	MOVLW	HIGH(___str_3)
	MOVWF	r0x05
	MOVLW	LOW(___str_3)
	MOVWF	r0x04
	MOVF	r0x06, W
	MOVWF	POSTDEC1
	MOVF	r0x05, W
	MOVWF	POSTDEC1
	MOVF	r0x04, W
	MOVWF	POSTDEC1
	CALL	_usart_puts
	MOVLW	0x03
	ADDWF	FSR1L, F
;	.line	159; meter_logger.c	fsk_rx_enable();
	CALL	_fsk_rx_enable
;	.line	160; meter_logger.c	break;
	BRA	_00139_DS_
_00112_DS_:
;	.line	163; meter_logger.c	fsk_rx_disable();
	CALL	_fsk_rx_disable
;	.line	164; meter_logger.c	usart_puts("\n\recho test - send some data\n\r");
	MOVLW	UPPER(___str_4)
	MOVWF	r0x06
	MOVLW	HIGH(___str_4)
	MOVWF	r0x05
	MOVLW	LOW(___str_4)
	MOVWF	r0x04
	MOVF	r0x06, W
	MOVWF	POSTDEC1
	MOVF	r0x05, W
	MOVWF	POSTDEC1
	MOVF	r0x04, W
	MOVWF	POSTDEC1
	CALL	_usart_puts
	MOVLW	0x03
	ADDWF	FSR1L, F
;	.line	165; meter_logger.c	fsk_rx_enable();
	CALL	_fsk_rx_enable
;	.line	167; meter_logger.c	last_fifo_size = 0;
	CLRF	r0x00
	CLRF	r0x01
;	.line	168; meter_logger.c	sleep_ms(1000);							// 1 second
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x03
	MOVWF	POSTDEC1
	MOVLW	0xe8
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVLW	0x04
	ADDWF	FSR1L, F
;	.line	169; meter_logger.c	fifo_size = fifo_in_use();
	CALL	_fifo_in_use
	MOVWF	r0x02
	MOVFF	PRODL, r0x03
_00113_DS_:
;	.line	170; meter_logger.c	while (fifo_size > last_fifo_size) {	// and wait while we are still receiving data
	MOVF	r0x03, W
	SUBWF	r0x01, W
	BNZ	_00206_DS_
	MOVF	r0x02, W
	SUBWF	r0x00, W
_00206_DS_:
	BC	_00115_DS_
;	.line	171; meter_logger.c	last_fifo_size = fifo_size;
	MOVFF	r0x02, r0x00
	MOVFF	r0x03, r0x01
;	.line	172; meter_logger.c	sleep_ms(500);						// return data when no data for 500 ms
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x01
	MOVWF	POSTDEC1
	MOVLW	0xf4
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVLW	0x04
	ADDWF	FSR1L, F
;	.line	173; meter_logger.c	fifo_size = fifo_in_use();
	CALL	_fifo_in_use
	MOVWF	r0x02
	MOVFF	PRODL, r0x03
	BRA	_00113_DS_
_00115_DS_:
;	.line	175; meter_logger.c	fsk_rx_disable();
	CALL	_fsk_rx_disable
;	.line	177; meter_logger.c	fsk_tx_enable();
	CALL	_fsk_tx_enable
;	.line	178; meter_logger.c	sleep_ms(FSK_TX_SLEEP);
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x0c
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVLW	0x04
	ADDWF	FSR1L, F
_00116_DS_:
;	.line	180; meter_logger.c	while (fifo_get(&sub_cmd)) {
	MOVLW	HIGH(_main_sub_cmd_1_79)
	MOVWF	r0x05
	MOVLW	LOW(_main_sub_cmd_1_79)
	MOVWF	r0x04
	MOVLW	0x80
	MOVWF	r0x06
	MOVF	r0x06, W
	MOVWF	POSTDEC1
	MOVF	r0x05, W
	MOVWF	POSTDEC1
	MOVF	r0x04, W
	MOVWF	POSTDEC1
	CALL	_fifo_get
	MOVWF	r0x04
	MOVLW	0x03
	ADDWF	FSR1L, F
	MOVF	r0x04, W
	BZ	_00118_DS_
	BANKSEL	_main_sub_cmd_1_79
;	.line	185; meter_logger.c	fsk_tx_byte(sub_cmd);
	MOVF	_main_sub_cmd_1_79, W, B
	MOVWF	POSTDEC1
	CALL	_fsk_tx_byte
	MOVF	POSTINC1, F
;	.line	186; meter_logger.c	sleep_ms(FSK_TX_SLEEP);
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x0c
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVLW	0x04
	ADDWF	FSR1L, F
	BRA	_00116_DS_
_00118_DS_:
;	.line	190; meter_logger.c	fsk_tx_disable();
	CALL	_fsk_tx_disable
;	.line	192; meter_logger.c	usart_puts("\n\rwaiting for new command\n\r");
	MOVLW	UPPER(___str_3)
	MOVWF	r0x06
	MOVLW	HIGH(___str_3)
	MOVWF	r0x05
	MOVLW	LOW(___str_3)
	MOVWF	r0x04
	MOVF	r0x06, W
	MOVWF	POSTDEC1
	MOVF	r0x05, W
	MOVWF	POSTDEC1
	MOVF	r0x04, W
	MOVWF	POSTDEC1
	CALL	_usart_puts
	MOVLW	0x03
	ADDWF	FSR1L, F
;	.line	193; meter_logger.c	fsk_rx_enable();
	CALL	_fsk_rx_enable
;	.line	194; meter_logger.c	break;
	BRA	_00139_DS_
_00119_DS_:
;	.line	197; meter_logger.c	fsk_rx_disable();
	CALL	_fsk_rx_disable
;	.line	201; meter_logger.c	fsk_rx_enable();
	CALL	_fsk_rx_enable
;	.line	204; meter_logger.c	last_fifo_size = 0;
	CLRF	r0x00
	CLRF	r0x01
;	.line	205; meter_logger.c	sleep_ms(400);							// sleep 400 ms to let some data come in
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x01
	MOVWF	POSTDEC1
	MOVLW	0x90
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVLW	0x04
	ADDWF	FSR1L, F
;	.line	206; meter_logger.c	fifo_size = fifo_in_use();
	CALL	_fifo_in_use
	MOVWF	r0x02
	MOVFF	PRODL, r0x03
_00120_DS_:
;	.line	207; meter_logger.c	while (fifo_size > last_fifo_size) {	// and wait while we are still receiving data
	MOVF	r0x03, W
	SUBWF	r0x01, W
	BNZ	_00207_DS_
	MOVF	r0x02, W
	SUBWF	r0x00, W
_00207_DS_:
	BC	_00122_DS_
;	.line	208; meter_logger.c	last_fifo_size = fifo_size;
	MOVFF	r0x02, r0x00
	MOVFF	r0x03, r0x01
;	.line	209; meter_logger.c	sleep_ms(200);						// return data when no data for 100 ms
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0xc8
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVLW	0x04
	ADDWF	FSR1L, F
;	.line	210; meter_logger.c	fifo_size = fifo_in_use();
	CALL	_fifo_in_use
	MOVWF	r0x02
	MOVFF	PRODL, r0x03
	BRA	_00120_DS_
_00122_DS_:
;	.line	212; meter_logger.c	fsk_rx_disable();
	CALL	_fsk_rx_disable
;	.line	217; meter_logger.c	rs232_tx_enable();
	CALL	_rs232_tx_enable
_00123_DS_:
;	.line	218; meter_logger.c	while (fifo_get(&sub_cmd)) {
	MOVLW	HIGH(_main_sub_cmd_1_79)
	MOVWF	r0x05
	MOVLW	LOW(_main_sub_cmd_1_79)
	MOVWF	r0x04
	MOVLW	0x80
	MOVWF	r0x06
	MOVF	r0x06, W
	MOVWF	POSTDEC1
	MOVF	r0x05, W
	MOVWF	POSTDEC1
	MOVF	r0x04, W
	MOVWF	POSTDEC1
	CALL	_fifo_get
	MOVWF	r0x04
	MOVLW	0x03
	ADDWF	FSR1L, F
	MOVF	r0x04, W
	BZ	_00125_DS_
	BANKSEL	_main_sub_cmd_1_79
;	.line	223; meter_logger.c	rs232_tx_byte(sub_cmd);
	MOVF	_main_sub_cmd_1_79, W, B
	MOVWF	POSTDEC1
	CALL	_rs232_tx_byte
	MOVF	POSTINC1, F
;	.line	224; meter_logger.c	sleep_ms(RS232_TX_SLEEP);
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x0c
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVLW	0x04
	ADDWF	FSR1L, F
	BRA	_00123_DS_
_00125_DS_:
;	.line	226; meter_logger.c	rs232_tx_disable();
	CALL	_rs232_tx_disable
;	.line	232; meter_logger.c	rs232_rx_enable();
	CALL	_rs232_rx_enable
;	.line	233; meter_logger.c	last_fifo_size = 0;
	CLRF	r0x00
	CLRF	r0x01
;	.line	234; meter_logger.c	sleep_ms(400);							// sleep 200 ms to let some data come in
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x01
	MOVWF	POSTDEC1
	MOVLW	0x90
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVLW	0x04
	ADDWF	FSR1L, F
;	.line	235; meter_logger.c	fifo_size = fifo_in_use();
	CALL	_fifo_in_use
	MOVWF	r0x02
	MOVFF	PRODL, r0x03
_00126_DS_:
;	.line	237; meter_logger.c	while (fifo_size > last_fifo_size) {	// and wait while we are still receiving data
	MOVF	r0x03, W
	SUBWF	r0x01, W
	BNZ	_00208_DS_
	MOVF	r0x02, W
	SUBWF	r0x00, W
_00208_DS_:
	BC	_00128_DS_
;	.line	238; meter_logger.c	last_fifo_size = fifo_size;
	MOVFF	r0x02, r0x00
	MOVFF	r0x03, r0x01
;	.line	239; meter_logger.c	sleep_ms(200);						// return data when no data for 100 ms
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0xc8
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVLW	0x04
	ADDWF	FSR1L, F
;	.line	240; meter_logger.c	fifo_size = fifo_in_use();
	CALL	_fifo_in_use
	MOVWF	r0x02
	MOVFF	PRODL, r0x03
	BRA	_00126_DS_
_00128_DS_:
;	.line	243; meter_logger.c	rs232_rx_disable();
	CALL	_rs232_rx_disable
;	.line	250; meter_logger.c	DEBUG2_PIN = 1;
	BSF	_PORTBbits, 3
	nop
	nop
	
;	.line	255; meter_logger.c	DEBUG2_PIN = 0;
	BCF	_PORTBbits, 3
	nop
	nop
	
;	.line	260; meter_logger.c	DEBUG2_PIN = 1;
	BSF	_PORTBbits, 3
	nop
	nop
	
;	.line	265; meter_logger.c	DEBUG2_PIN = 0;
	BCF	_PORTBbits, 3
;	.line	267; meter_logger.c	if (fifo_in_use()) {
	CALL	_fifo_in_use
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVF	r0x00, W
	IORWF	r0x01, W
	BTFSC	STATUS, 2
	BRA	_00133_DS_
;	.line	269; meter_logger.c	DEBUG2_PIN = 1;
	BSF	_PORTBbits, 3
	nop
	nop
	
;	.line	274; meter_logger.c	DEBUG2_PIN = 0;
	BCF	_PORTBbits, 3
;	.line	276; meter_logger.c	fsk_tx_enable();
	CALL	_fsk_tx_enable
;	.line	278; meter_logger.c	sleep_ms(FSK_TX_SLEEP);
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x0c
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVLW	0x04
	ADDWF	FSR1L, F
_00129_DS_:
;	.line	279; meter_logger.c	while (fifo_get(&sub_cmd)) {
	MOVLW	HIGH(_main_sub_cmd_1_79)
	MOVWF	r0x01
	MOVLW	LOW(_main_sub_cmd_1_79)
	MOVWF	r0x00
	MOVLW	0x80
	MOVWF	r0x02
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	_fifo_get
	MOVWF	r0x00
	MOVLW	0x03
	ADDWF	FSR1L, F
	MOVF	r0x00, W
	BZ	_00131_DS_
	BANKSEL	_main_sub_cmd_1_79
;	.line	284; meter_logger.c	fsk_tx_byte(sub_cmd);
	MOVF	_main_sub_cmd_1_79, W, B
	MOVWF	POSTDEC1
	CALL	_fsk_tx_byte
	MOVF	POSTINC1, F
;	.line	285; meter_logger.c	sleep_ms(FSK_TX_SLEEP);
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x0c
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVLW	0x04
	ADDWF	FSR1L, F
	BRA	_00129_DS_
_00131_DS_:
;	.line	289; meter_logger.c	fsk_tx_disable();
	CALL	_fsk_tx_disable
	BRA	_00134_DS_
_00133_DS_:
;	.line	294; meter_logger.c	DEBUG2_PIN = 1;
	BSF	_PORTBbits, 3
	nop
	nop
	
;	.line	299; meter_logger.c	DEBUG2_PIN = 0;
	BCF	_PORTBbits, 3
	nop
	nop
	
;	.line	304; meter_logger.c	DEBUG2_PIN = 1;
	BSF	_PORTBbits, 3
	nop
	nop
	
;	.line	309; meter_logger.c	DEBUG2_PIN = 0;
	BCF	_PORTBbits, 3
;	.line	314; meter_logger.c	fsk_tx_enable();
	CALL	_fsk_tx_enable
;	.line	315; meter_logger.c	sleep_ms(FSK_TX_SLEEP);
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x0c
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVLW	0x04
	ADDWF	FSR1L, F
;	.line	316; meter_logger.c	fsk_tx_byte(0x0d);
	MOVLW	0x0d
	MOVWF	POSTDEC1
	CALL	_fsk_tx_byte
	MOVF	POSTINC1, F
;	.line	317; meter_logger.c	sleep_ms(FSK_TX_SLEEP);
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x0c
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVLW	0x04
	ADDWF	FSR1L, F
;	.line	318; meter_logger.c	fsk_tx_disable();
	CALL	_fsk_tx_disable
_00134_DS_:
;	.line	324; meter_logger.c	fsk_rx_enable();
	CALL	_fsk_rx_enable
;	.line	326; meter_logger.c	}
	GOTO	_00139_DS_
	RETURN	

; ; Starting pCode block
S_meter_logger___debug2	code
__debug2:
;	.line	4255; meter_logger.c	void _debug2() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
;	.line	4256; meter_logger.c	DEBUG2_PIN = 0x1;
	BSF	_PORTBbits, 3
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	
;	.line	4304; meter_logger.c	DEBUG2_PIN = 0x0;
	BCF	_PORTBbits, 3
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger___debug	code
__debug:
;	.line	4155; meter_logger.c	void _debug() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
;	.line	4156; meter_logger.c	DEBUG_PIN = 0x1;
	BSF	_PORTBbits, 2
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	
;	.line	4204; meter_logger.c	DEBUG_PIN = 0x0;
	BCF	_PORTBbits, 2
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__flash_led	code
_flash_led:
;	.line	4150; meter_logger.c	void flash_led(unsigned char ms) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
;	.line	4151; meter_logger.c	led_flash.timer = ms;
	MOVF	r0x00, W
	BANKSEL	(_led_flash + 1)
	MOVWF	(_led_flash + 1), B
; removed redundant BANKSEL
;	.line	4152; meter_logger.c	led_flash.state = LED_FLASH_RUN;
	CLRF	_led_flash, B
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__fifo_snoop	code
_fifo_snoop:
;	.line	4127; meter_logger.c	unsigned char fifo_snoop(unsigned char *c, unsigned int pos) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVFF	r0x01, POSTDEC1
	MOVFF	r0x02, POSTDEC1
	MOVFF	r0x03, POSTDEC1
	MOVFF	r0x04, POSTDEC1
	MOVFF	r0x05, POSTDEC1
	MOVFF	r0x06, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
	MOVLW	0x03
	MOVFF	PLUSW2, r0x01
	MOVLW	0x04
	MOVFF	PLUSW2, r0x02
	MOVLW	0x05
	MOVFF	PLUSW2, r0x03
	MOVLW	0x06
	MOVFF	PLUSW2, r0x04
;	.line	4128; meter_logger.c	if (fifo_in_use() > (pos)) {
	CALL	_fifo_in_use
	MOVWF	r0x05
	MOVFF	PRODL, r0x06
	MOVF	r0x06, W
	SUBWF	r0x04, W
	BNZ	_00895_DS_
	MOVF	r0x05, W
	SUBWF	r0x03, W
_00895_DS_:
	BTFSC	STATUS, 0
	BRA	_00885_DS_
	BANKSEL	(_fifo_tail + 1)
;	.line	4129; meter_logger.c	switch (fifo_tail/QUEUE_SIZE) {
	MOVF	(_fifo_tail + 1), W, B
	MOVWF	r0x05
	CLRF	r0x06
	MOVLW	0x00
	SUBWF	r0x06, W
	BNZ	_00896_DS_
	MOVLW	0x04
	SUBWF	r0x05, W
_00896_DS_:
	BTFSC	STATUS, 0
	BRA	_00883_DS_
	CLRF	PCLATH
	CLRF	PCLATU
	RLCF	r0x05, W
	RLCF	PCLATH, F
	RLCF	WREG, W
	RLCF	PCLATH, F
	ANDLW	0xfc
	ADDLW	LOW(_00897_DS_)
	MOVWF	POSTDEC1
	MOVLW	HIGH(_00897_DS_)
	ADDWFC	PCLATH, F
	MOVLW	UPPER(_00897_DS_)
	ADDWFC	PCLATU, F
	MOVF	PREINC1, W
	MOVWF	PCL
_00897_DS_:
	GOTO	_00879_DS_
	GOTO	_00880_DS_
	GOTO	_00881_DS_
	GOTO	_00882_DS_
_00879_DS_:
;	.line	4131; meter_logger.c	*c = fifo_buffer_0[(fifo_tail + pos) % QUEUE_SIZE];
	MOVF	r0x03, W
	BANKSEL	_fifo_tail
	ADDWF	_fifo_tail, W, B
	MOVWF	r0x05
	MOVF	r0x04, W
; removed redundant BANKSEL
	ADDWFC	(_fifo_tail + 1), W, B
	MOVWF	r0x06
	CLRF	r0x06
	MOVLW	LOW(_fifo_buffer_0)
	ADDWF	r0x05, F
	MOVLW	HIGH(_fifo_buffer_0)
	ADDWFC	r0x06, F
	MOVFF	r0x05, FSR0L
	MOVFF	r0x06, FSR0H
	MOVFF	INDF0, r0x05
	MOVFF	r0x05, POSTDEC1
	MOVFF	r0x00, FSR0L
	MOVFF	r0x01, PRODL
	MOVF	r0x02, W
	CALL	__gptrput1
;	.line	4132; meter_logger.c	break;
	BRA	_00883_DS_
_00880_DS_:
;	.line	4134; meter_logger.c	*c = fifo_buffer_1[(fifo_tail + pos) % QUEUE_SIZE];
	MOVF	r0x03, W
	BANKSEL	_fifo_tail
	ADDWF	_fifo_tail, W, B
	MOVWF	r0x05
	MOVF	r0x04, W
; removed redundant BANKSEL
	ADDWFC	(_fifo_tail + 1), W, B
	MOVWF	r0x06
	CLRF	r0x06
	MOVLW	LOW(_fifo_buffer_1)
	ADDWF	r0x05, F
	MOVLW	HIGH(_fifo_buffer_1)
	ADDWFC	r0x06, F
	MOVFF	r0x05, FSR0L
	MOVFF	r0x06, FSR0H
	MOVFF	INDF0, r0x05
	MOVFF	r0x05, POSTDEC1
	MOVFF	r0x00, FSR0L
	MOVFF	r0x01, PRODL
	MOVF	r0x02, W
	CALL	__gptrput1
;	.line	4135; meter_logger.c	break;
	BRA	_00883_DS_
_00881_DS_:
;	.line	4137; meter_logger.c	*c = fifo_buffer_2[(fifo_tail + pos) % QUEUE_SIZE];
	MOVF	r0x03, W
	BANKSEL	_fifo_tail
	ADDWF	_fifo_tail, W, B
	MOVWF	r0x05
	MOVF	r0x04, W
; removed redundant BANKSEL
	ADDWFC	(_fifo_tail + 1), W, B
	MOVWF	r0x06
	CLRF	r0x06
	MOVLW	LOW(_fifo_buffer_2)
	ADDWF	r0x05, F
	MOVLW	HIGH(_fifo_buffer_2)
	ADDWFC	r0x06, F
	MOVFF	r0x05, FSR0L
	MOVFF	r0x06, FSR0H
	MOVFF	INDF0, r0x05
	MOVFF	r0x05, POSTDEC1
	MOVFF	r0x00, FSR0L
	MOVFF	r0x01, PRODL
	MOVF	r0x02, W
	CALL	__gptrput1
;	.line	4138; meter_logger.c	break;
	BRA	_00883_DS_
_00882_DS_:
	BANKSEL	_fifo_tail
;	.line	4140; meter_logger.c	*c = fifo_buffer_3[(fifo_tail + pos) % QUEUE_SIZE];
	MOVF	_fifo_tail, W, B
	ADDWF	r0x03, F
; removed redundant BANKSEL
	MOVF	(_fifo_tail + 1), W, B
	ADDWFC	r0x04, F
	CLRF	r0x04
	MOVLW	LOW(_fifo_buffer_3)
	ADDWF	r0x03, F
	MOVLW	HIGH(_fifo_buffer_3)
	ADDWFC	r0x04, F
	MOVFF	r0x03, FSR0L
	MOVFF	r0x04, FSR0H
	MOVFF	INDF0, r0x03
	MOVFF	r0x03, POSTDEC1
	MOVFF	r0x00, FSR0L
	MOVFF	r0x01, PRODL
	MOVF	r0x02, W
	CALL	__gptrput1
_00883_DS_:
;	.line	4143; meter_logger.c	return 1;
	MOVLW	0x01
	BRA	_00887_DS_
_00885_DS_:
;	.line	4146; meter_logger.c	return 0;
	CLRF	WREG
_00887_DS_:
	MOVFF	PREINC1, r0x06
	MOVFF	PREINC1, r0x05
	MOVFF	PREINC1, r0x04
	MOVFF	PREINC1, r0x03
	MOVFF	PREINC1, r0x02
	MOVFF	PREINC1, r0x01
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__fifo_get	code
_fifo_get:
;	.line	4099; meter_logger.c	unsigned char fifo_get(unsigned char *c) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVFF	r0x01, POSTDEC1
	MOVFF	r0x02, POSTDEC1
	MOVFF	r0x03, POSTDEC1
	MOVFF	r0x04, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
	MOVLW	0x03
	MOVFF	PLUSW2, r0x01
	MOVLW	0x04
	MOVFF	PLUSW2, r0x02
;	.line	4100; meter_logger.c	if (fifo_in_use() != 0) {
	CALL	_fifo_in_use
	MOVWF	r0x03
	MOVFF	PRODL, r0x04
	MOVF	r0x03, W
	IORWF	r0x04, W
	BTFSC	STATUS, 2
	BRA	_00854_DS_
	BANKSEL	(_fifo_tail + 1)
;	.line	4101; meter_logger.c	switch (fifo_tail/QUEUE_SIZE) {
	MOVF	(_fifo_tail + 1), W, B
	MOVWF	r0x03
	CLRF	r0x04
	MOVLW	0x00
	SUBWF	r0x04, W
	BNZ	_00867_DS_
	MOVLW	0x04
	SUBWF	r0x03, W
_00867_DS_:
	BTFSC	STATUS, 0
	BRA	_00850_DS_
	CLRF	PCLATH
	CLRF	PCLATU
	RLCF	r0x03, W
	RLCF	PCLATH, F
	RLCF	WREG, W
	RLCF	PCLATH, F
	ANDLW	0xfc
	ADDLW	LOW(_00868_DS_)
	MOVWF	POSTDEC1
	MOVLW	HIGH(_00868_DS_)
	ADDWFC	PCLATH, F
	MOVLW	UPPER(_00868_DS_)
	ADDWFC	PCLATU, F
	MOVF	PREINC1, W
	MOVWF	PCL
_00868_DS_:
	GOTO	_00846_DS_
	GOTO	_00847_DS_
	GOTO	_00848_DS_
	GOTO	_00849_DS_
_00846_DS_:
	BANKSEL	_fifo_tail
;	.line	4103; meter_logger.c	*c = fifo_buffer_0[fifo_tail % QUEUE_SIZE];
	MOVF	_fifo_tail, W, B
	MOVWF	r0x03
	CLRF	r0x04
	MOVLW	LOW(_fifo_buffer_0)
	ADDWF	r0x03, F
	MOVLW	HIGH(_fifo_buffer_0)
	ADDWFC	r0x04, F
	MOVFF	r0x03, FSR0L
	MOVFF	r0x04, FSR0H
	MOVFF	INDF0, r0x03
	MOVFF	r0x03, POSTDEC1
	MOVFF	r0x00, FSR0L
	MOVFF	r0x01, PRODL
	MOVF	r0x02, W
	CALL	__gptrput1
;	.line	4104; meter_logger.c	break;
	BRA	_00850_DS_
_00847_DS_:
	BANKSEL	_fifo_tail
;	.line	4106; meter_logger.c	*c = fifo_buffer_1[fifo_tail % QUEUE_SIZE];
	MOVF	_fifo_tail, W, B
	MOVWF	r0x03
	CLRF	r0x04
	MOVLW	LOW(_fifo_buffer_1)
	ADDWF	r0x03, F
	MOVLW	HIGH(_fifo_buffer_1)
	ADDWFC	r0x04, F
	MOVFF	r0x03, FSR0L
	MOVFF	r0x04, FSR0H
	MOVFF	INDF0, r0x03
	MOVFF	r0x03, POSTDEC1
	MOVFF	r0x00, FSR0L
	MOVFF	r0x01, PRODL
	MOVF	r0x02, W
	CALL	__gptrput1
;	.line	4107; meter_logger.c	break;
	BRA	_00850_DS_
_00848_DS_:
	BANKSEL	_fifo_tail
;	.line	4109; meter_logger.c	*c = fifo_buffer_2[fifo_tail % QUEUE_SIZE];
	MOVF	_fifo_tail, W, B
	MOVWF	r0x03
	CLRF	r0x04
	MOVLW	LOW(_fifo_buffer_2)
	ADDWF	r0x03, F
	MOVLW	HIGH(_fifo_buffer_2)
	ADDWFC	r0x04, F
	MOVFF	r0x03, FSR0L
	MOVFF	r0x04, FSR0H
	MOVFF	INDF0, r0x03
	MOVFF	r0x03, POSTDEC1
	MOVFF	r0x00, FSR0L
	MOVFF	r0x01, PRODL
	MOVF	r0x02, W
	CALL	__gptrput1
;	.line	4110; meter_logger.c	break;
	BRA	_00850_DS_
_00849_DS_:
	BANKSEL	_fifo_tail
;	.line	4112; meter_logger.c	*c = fifo_buffer_3[fifo_tail % QUEUE_SIZE];
	MOVF	_fifo_tail, W, B
	MOVWF	r0x03
	CLRF	r0x04
	MOVLW	LOW(_fifo_buffer_3)
	ADDWF	r0x03, F
	MOVLW	HIGH(_fifo_buffer_3)
	ADDWFC	r0x04, F
	MOVFF	r0x03, FSR0L
	MOVFF	r0x04, FSR0H
	MOVFF	INDF0, r0x03
	MOVFF	r0x03, POSTDEC1
	MOVFF	r0x00, FSR0L
	MOVFF	r0x01, PRODL
	MOVF	r0x02, W
	CALL	__gptrput1
_00850_DS_:
	BANKSEL	_fifo_tail
;	.line	4115; meter_logger.c	fifo_tail++;
	INCFSZ	_fifo_tail, F, B
	BRA	_10912_DS_
; removed redundant BANKSEL
	INCF	(_fifo_tail + 1), F, B
_10912_DS_:
	BANKSEL	_fifo_tail
;	.line	4117; meter_logger.c	if (fifo_tail == QUEUE_SIZE_COMBINED) {
	MOVF	_fifo_tail, W, B
	BNZ	_00873_DS_
; removed redundant BANKSEL
	MOVF	(_fifo_tail + 1), W, B
	XORLW	0x04
	BZ	_00874_DS_
_00873_DS_:
	BRA	_00852_DS_
_00874_DS_:
	BANKSEL	_fifo_tail
;	.line	4118; meter_logger.c	fifo_tail = 0;
	CLRF	_fifo_tail, B
; removed redundant BANKSEL
	CLRF	(_fifo_tail + 1), B
_00852_DS_:
;	.line	4120; meter_logger.c	return 1;
	MOVLW	0x01
	BRA	_00856_DS_
_00854_DS_:
;	.line	4123; meter_logger.c	return 0;
	CLRF	WREG
_00856_DS_:
	MOVFF	PREINC1, r0x04
	MOVFF	PREINC1, r0x03
	MOVFF	PREINC1, r0x02
	MOVFF	PREINC1, r0x01
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__fifo_put	code
_fifo_put:
;	.line	4071; meter_logger.c	unsigned char fifo_put(unsigned char c) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVFF	r0x01, POSTDEC1
	MOVFF	r0x02, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
;	.line	4072; meter_logger.c	if (fifo_in_use() != QUEUE_SIZE_COMBINED) {
	CALL	_fifo_in_use
	MOVWF	r0x01
	MOVFF	PRODL, r0x02
	MOVF	r0x01, W
	BNZ	_00833_DS_
	MOVF	r0x02, W
	XORLW	0x04
	BNZ	_00833_DS_
	BRA	_00819_DS_
_00833_DS_:
	BANKSEL	(_fifo_head + 1)
;	.line	4073; meter_logger.c	switch (fifo_head/QUEUE_SIZE) {
	MOVF	(_fifo_head + 1), W, B
	MOVWF	r0x01
	CLRF	r0x02
	MOVLW	0x00
	SUBWF	r0x02, W
	BNZ	_00834_DS_
	MOVLW	0x04
	SUBWF	r0x01, W
_00834_DS_:
	BTFSC	STATUS, 0
	BRA	_00815_DS_
	CLRF	PCLATH
	CLRF	PCLATU
	RLCF	r0x01, W
	RLCF	PCLATH, F
	RLCF	WREG, W
	RLCF	PCLATH, F
	ANDLW	0xfc
	ADDLW	LOW(_00835_DS_)
	MOVWF	POSTDEC1
	MOVLW	HIGH(_00835_DS_)
	ADDWFC	PCLATH, F
	MOVLW	UPPER(_00835_DS_)
	ADDWFC	PCLATU, F
	MOVF	PREINC1, W
	MOVWF	PCL
_00835_DS_:
	GOTO	_00811_DS_
	GOTO	_00812_DS_
	GOTO	_00813_DS_
	GOTO	_00814_DS_
_00811_DS_:
	BANKSEL	_fifo_head
;	.line	4075; meter_logger.c	fifo_buffer_0[fifo_head % QUEUE_SIZE] = c;
	MOVF	_fifo_head, W, B
	MOVWF	r0x01
	CLRF	r0x02
	MOVLW	LOW(_fifo_buffer_0)
	ADDWF	r0x01, F
	MOVLW	HIGH(_fifo_buffer_0)
	ADDWFC	r0x02, F
	MOVFF	r0x01, FSR0L
	MOVFF	r0x02, FSR0H
	MOVFF	r0x00, INDF0
;	.line	4076; meter_logger.c	break;
	BRA	_00815_DS_
_00812_DS_:
	BANKSEL	_fifo_head
;	.line	4078; meter_logger.c	fifo_buffer_1[fifo_head % QUEUE_SIZE] = c;
	MOVF	_fifo_head, W, B
	MOVWF	r0x01
	CLRF	r0x02
	MOVLW	LOW(_fifo_buffer_1)
	ADDWF	r0x01, F
	MOVLW	HIGH(_fifo_buffer_1)
	ADDWFC	r0x02, F
	MOVFF	r0x01, FSR0L
	MOVFF	r0x02, FSR0H
	MOVFF	r0x00, INDF0
;	.line	4079; meter_logger.c	break;
	BRA	_00815_DS_
_00813_DS_:
	BANKSEL	_fifo_head
;	.line	4081; meter_logger.c	fifo_buffer_2[fifo_head % QUEUE_SIZE] = c;
	MOVF	_fifo_head, W, B
	MOVWF	r0x01
	CLRF	r0x02
	MOVLW	LOW(_fifo_buffer_2)
	ADDWF	r0x01, F
	MOVLW	HIGH(_fifo_buffer_2)
	ADDWFC	r0x02, F
	MOVFF	r0x01, FSR0L
	MOVFF	r0x02, FSR0H
	MOVFF	r0x00, INDF0
;	.line	4082; meter_logger.c	break;
	BRA	_00815_DS_
_00814_DS_:
	BANKSEL	_fifo_head
;	.line	4084; meter_logger.c	fifo_buffer_3[fifo_head % QUEUE_SIZE] = c;
	MOVF	_fifo_head, W, B
	MOVWF	r0x01
	CLRF	r0x02
	MOVLW	LOW(_fifo_buffer_3)
	ADDWF	r0x01, F
	MOVLW	HIGH(_fifo_buffer_3)
	ADDWFC	r0x02, F
	MOVFF	r0x01, FSR0L
	MOVFF	r0x02, FSR0H
	MOVFF	r0x00, INDF0
_00815_DS_:
	BANKSEL	_fifo_head
;	.line	4087; meter_logger.c	fifo_head++;
	INCFSZ	_fifo_head, F, B
	BRA	_20913_DS_
; removed redundant BANKSEL
	INCF	(_fifo_head + 1), F, B
_20913_DS_:
	BANKSEL	_fifo_head
;	.line	4089; meter_logger.c	if (fifo_head == QUEUE_SIZE_COMBINED) {
	MOVF	_fifo_head, W, B
	BNZ	_00840_DS_
; removed redundant BANKSEL
	MOVF	(_fifo_head + 1), W, B
	XORLW	0x04
	BZ	_00841_DS_
_00840_DS_:
	BRA	_00817_DS_
_00841_DS_:
	BANKSEL	_fifo_head
;	.line	4090; meter_logger.c	fifo_head = 0;
	CLRF	_fifo_head, B
; removed redundant BANKSEL
	CLRF	(_fifo_head + 1), B
_00817_DS_:
;	.line	4092; meter_logger.c	return 1;
	MOVLW	0x01
	BRA	_00821_DS_
_00819_DS_:
;	.line	4095; meter_logger.c	return 0;
	CLRF	WREG
_00821_DS_:
	MOVFF	PREINC1, r0x02
	MOVFF	PREINC1, r0x01
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__fifo_in_use	code
_fifo_in_use:
;	.line	4067; meter_logger.c	unsigned int fifo_in_use() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVFF	r0x01, POSTDEC1
	BANKSEL	_fifo_tail
;	.line	4068; meter_logger.c	return fifo_head - fifo_tail;
	MOVF	_fifo_tail, W, B
	BANKSEL	_fifo_head
	SUBWF	_fifo_head, W, B
	MOVWF	r0x00
	BANKSEL	(_fifo_tail + 1)
	MOVF	(_fifo_tail + 1), W, B
	BANKSEL	(_fifo_head + 1)
	SUBWFB	(_fifo_head + 1), W, B
	MOVWF	r0x01
	MOVFF	r0x01, PRODL
	MOVF	r0x00, W
	MOVFF	PREINC1, r0x01
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__fsk_tx_byte	code
_fsk_tx_byte:
;	.line	4062; meter_logger.c	void fsk_tx_byte(unsigned char c) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
;	.line	4063; meter_logger.c	fsk_proto.data = c;
	MOVF	r0x00, W
	BANKSEL	(_fsk_proto + 12)
	MOVWF	(_fsk_proto + 12), B
;	.line	4064; meter_logger.c	fsk_proto.data_len = 8;
	MOVLW	0x08
; removed redundant BANKSEL
	MOVWF	(_fsk_proto + 13), B
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__send_fsk_low	code
_send_fsk_low:
;	.line	2609; meter_logger.c	void send_fsk_low(void) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
;	.line	2610; meter_logger.c	PWM_PIN = 1;
	BSF	_PORTCbits, 1
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	
;	.line	2817; meter_logger.c	PWM_PIN = 0;
	BCF	_PORTCbits, 1
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	
;	.line	3024; meter_logger.c	PWM_PIN = 1;
	BSF	_PORTCbits, 1
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	
;	.line	3231; meter_logger.c	PWM_PIN = 0;
	BCF	_PORTCbits, 1
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	
;	.line	3438; meter_logger.c	PWM_PIN = 1;
	BSF	_PORTCbits, 1
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	
;	.line	3645; meter_logger.c	PWM_PIN = 0;
	BCF	_PORTCbits, 1
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	
;	.line	3852; meter_logger.c	PWM_PIN = 1;
	BSF	_PORTCbits, 1
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	
;	.line	4059; meter_logger.c	PWM_PIN = 0;
	BCF	_PORTCbits, 1
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__send_fsk_high	code
_send_fsk_high:
;	.line	1087; meter_logger.c	void send_fsk_high(void) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
;	.line	1088; meter_logger.c	PWM_PIN = 1;
	BSF	_PORTCbits, 1
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	
;	.line	1226; meter_logger.c	PWM_PIN = 0;
	BCF	_PORTCbits, 1
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	
;	.line	1364; meter_logger.c	PWM_PIN = 1;
	BSF	_PORTCbits, 1
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	
;	.line	1502; meter_logger.c	PWM_PIN = 0;
	BCF	_PORTCbits, 1
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	
;	.line	1640; meter_logger.c	PWM_PIN = 1;
	BSF	_PORTCbits, 1
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	
;	.line	1778; meter_logger.c	PWM_PIN = 0;
	BCF	_PORTCbits, 1
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	
;	.line	1916; meter_logger.c	PWM_PIN = 1;
	BSF	_PORTCbits, 1
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	
;	.line	2054; meter_logger.c	PWM_PIN = 0;
	BCF	_PORTCbits, 1
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	
;	.line	2192; meter_logger.c	PWM_PIN = 1;
	BSF	_PORTCbits, 1
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	
;	.line	2330; meter_logger.c	PWM_PIN = 0;
	BCF	_PORTCbits, 1
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	
;	.line	2468; meter_logger.c	PWM_PIN = 1;
	BSF	_PORTCbits, 1
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	
;	.line	2606; meter_logger.c	PWM_PIN = 0;
	BCF	_PORTCbits, 1
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__fsk_rx_disable	code
_fsk_rx_disable:
;	.line	1082; meter_logger.c	void fsk_rx_disable() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
;	.line	1083; meter_logger.c	PIE2bits.CMIE = 0;		// Disable comparator interrupt
	BCF	_PIE2bits, 6
	BANKSEL	_codec_type
;	.line	1084; meter_logger.c	codec_type = NONE;
	CLRF	_codec_type, B
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__fsk_rx_enable	code
_fsk_rx_enable:
;	.line	1046; meter_logger.c	void fsk_rx_enable() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
;	.line	1047; meter_logger.c	fsk_proto.state = START_BIT_WAIT;
	MOVLW	0x02
	BANKSEL	_fsk_proto
	MOVWF	_fsk_proto, B
; removed redundant BANKSEL
;	.line	1048; meter_logger.c	fsk_proto.start_bit_time = 0;
	CLRF	(_fsk_proto + 10), B
; removed redundant BANKSEL
	CLRF	(_fsk_proto + 11), B
;	.line	1050; meter_logger.c	timer0_reload = TIMER0_FSK;
	MOVLW	0x9f
	BANKSEL	_timer0_reload
	MOVWF	_timer0_reload, B
	MOVLW	0xf9
; removed redundant BANKSEL
	MOVWF	(_timer0_reload + 1), B
;	.line	1052; meter_logger.c	codec_type = FSK_RX;
	MOVLW	0x04
	BANKSEL	_codec_type
	MOVWF	_codec_type, B
;	.line	1055; meter_logger.c	T0CONbits.TMR0ON = 1;
	BSF	_T0CONbits, 7
;	.line	1056; meter_logger.c	T0CONbits.T0PS0 = 0;
	BCF	_T0CONbits, 0
;	.line	1057; meter_logger.c	T0CONbits.T0PS1 = 0;
	BCF	_T0CONbits, 1
;	.line	1058; meter_logger.c	T0CONbits.T0PS2 = 0;		// prescaler 1:2
	BCF	_T0CONbits, 2
;	.line	1059; meter_logger.c	T0CONbits.T08BIT = 0;		// use timer0 16-bit counter
	BCF	_T0CONbits, 6
;	.line	1060; meter_logger.c	T0CONbits.T0CS = 0;			// internal clock source
	BCF	_T0CONbits, 5
;	.line	1061; meter_logger.c	T0CONbits.PSA = 1;			// disable timer0 prescaler
	BSF	_T0CONbits, 3
;	.line	1062; meter_logger.c	INTCON2bits.TMR0IP = 1;		// high priority
	BSF	_INTCON2bits, 2
;	.line	1063; meter_logger.c	INTCONbits.TMR0IE = 0;		// Dont enable TMR0 Interrupt
	BCF	_INTCONbits, 5
;	.line	1066; meter_logger.c	CVRCONbits.CVREF = 0xf;	// 0V
	BSF	_CVRCONbits, 4
;	.line	1068; meter_logger.c	CVRCONbits.CVRSS = 0;	// VDD – VSS
	BCF	_CVRCONbits, 4
;	.line	1069; meter_logger.c	CVRCONbits.CVRR = 0;	// high range, 0.25 CVRSRC to 0.75 CVRSRC, with CVRSRC/32 step size
	BCF	_CVRCONbits, 5
;	.line	1070; meter_logger.c	CVRCONbits.CVR = 9;		// 2,65625 V
	MOVF	_CVRCONbits, W
	ANDLW	0xf0
	IORLW	0x09
	MOVWF	_CVRCONbits
;	.line	1071; meter_logger.c	CVRCONbits.CVROE = 0;	// Comparator VREF Output disabled, CVREF voltage is disconnected from the RA2/AN2/VREF-/CVREF pin
	BCF	_CVRCONbits, 6
;	.line	1072; meter_logger.c	CVRCONbits.CVREN = 1;	// Comparator Voltage Reference Enable bit
	BSF	_CVRCONbits, 7
;	.line	1074; meter_logger.c	CMCONbits.CM = 0x6;		// four inputs multiplexed to two comparators
	MOVF	_CMCONbits, W
	ANDLW	0xf8
	IORLW	0x06
	MOVWF	_CMCONbits
;	.line	1075; meter_logger.c	CMCONbits.CIS = 0;		// multiplexed to RA0/AN0 and RA1/AN1
	BCF	_CMCONbits, 3
;	.line	1076; meter_logger.c	CMCONbits.C1INV = 1;	// inverted output, C1 VIN+ < C1 VIN-
	BSF	_CMCONbits, 4
;	.line	1078; meter_logger.c	IPR2bits.CMIP = 1;		// high priority
	BSF	_IPR2bits, 6
;	.line	1079; meter_logger.c	PIE2bits.CMIE = 1;		// Enable comparator interrupt
	BSF	_PIE2bits, 6
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__fsk_tx_disable	code
_fsk_tx_disable:
;	.line	1040; meter_logger.c	void fsk_tx_disable() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	BANKSEL	_codec_type
;	.line	1041; meter_logger.c	codec_type = NONE;
	CLRF	_codec_type, B
;	.line	1042; meter_logger.c	T0CONbits.TMR0ON = 0;	// Disable TMR0 
	BCF	_T0CONbits, 7
;	.line	1043; meter_logger.c	PIE2bits.CMIE = 1;		// Disable comparator interrupt
	BSF	_PIE2bits, 6
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__fsk_tx_enable	code
_fsk_tx_enable:
;	.line	1022; meter_logger.c	void fsk_tx_enable() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
;	.line	1023; meter_logger.c	timer0_reload = TIMER0_FSK;
	MOVLW	0x9f
	BANKSEL	_timer0_reload
	MOVWF	_timer0_reload, B
	MOVLW	0xf9
; removed redundant BANKSEL
	MOVWF	(_timer0_reload + 1), B
	BANKSEL	_fsk_proto
;	.line	1025; meter_logger.c	fsk_proto.state = INIT_STATE;
	CLRF	_fsk_proto, B
;	.line	1026; meter_logger.c	codec_type = FSK_TX;
	MOVLW	0x05
	BANKSEL	_codec_type
	MOVWF	_codec_type, B
;	.line	1029; meter_logger.c	T0CONbits.TMR0ON = 1;
	BSF	_T0CONbits, 7
;	.line	1030; meter_logger.c	T0CONbits.T0PS0 = 0;
	BCF	_T0CONbits, 0
;	.line	1031; meter_logger.c	T0CONbits.T0PS1 = 0;
	BCF	_T0CONbits, 1
;	.line	1032; meter_logger.c	T0CONbits.T0PS2 = 0;		// prescaler 1:2
	BCF	_T0CONbits, 2
;	.line	1033; meter_logger.c	T0CONbits.T08BIT = 0;		// use timer0 16-bit counter
	BCF	_T0CONbits, 6
;	.line	1034; meter_logger.c	T0CONbits.T0CS = 0;			// internal clock source
	BCF	_T0CONbits, 5
;	.line	1035; meter_logger.c	T0CONbits.PSA = 1;			// disable timer0 prescaler
	BSF	_T0CONbits, 3
;	.line	1036; meter_logger.c	INTCON2bits.TMR0IP = 1;		// high priority
	BSF	_INTCON2bits, 2
;	.line	1037; meter_logger.c	INTCONbits.TMR0IE = 1;		// Enable TMR0 Interrupt
	BSF	_INTCONbits, 5
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__rs232_tx_byte	code
_rs232_tx_byte:
;	.line	1015; meter_logger.c	void rs232_tx_byte(unsigned char c) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
;	.line	1016; meter_logger.c	rs232_proto.data = c;
	MOVF	r0x00, W
	BANKSEL	(_rs232_proto + 2)
	MOVWF	(_rs232_proto + 2), B
;	.line	1017; meter_logger.c	rs232_proto.data_len = 8;
	MOVLW	0x08
; removed redundant BANKSEL
	MOVWF	(_rs232_proto + 3), B
;	.line	1018; meter_logger.c	T0CONbits.TMR0ON = 1;		// start timer 0
	BSF	_T0CONbits, 7
;	.line	1019; meter_logger.c	INTCONbits.TMR0IF = 1;		// enter timer interrupt handler now
	BSF	_INTCONbits, 2
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__rs232_rx_disable	code
_rs232_rx_disable:
;	.line	1009; meter_logger.c	void rs232_rx_disable() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
;	.line	1010; meter_logger.c	INTCONbits.INT0IE = 0;		// disable ext int
	BCF	_INTCONbits, 4
	BANKSEL	_codec_type
;	.line	1011; meter_logger.c	codec_type = NONE;
	CLRF	_codec_type, B
;	.line	1012; meter_logger.c	T0CONbits.TMR0ON = 0;
	BCF	_T0CONbits, 7
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__rs232_rx_enable	code
_rs232_rx_enable:
;	.line	985; meter_logger.c	void rs232_rx_enable() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
;	.line	986; meter_logger.c	rs232_proto.state = START_BIT_WAIT;
	MOVLW	0x02
	BANKSEL	_rs232_proto
	MOVWF	_rs232_proto, B
; removed redundant BANKSEL
;	.line	987; meter_logger.c	rs232_proto.data_len = 0;
	CLRF	(_rs232_proto + 3), B
;	.line	989; meter_logger.c	timer0_reload = TIMER0_RS232_1200;
	MOVLW	0xae
	BANKSEL	_timer0_reload
	MOVWF	_timer0_reload, B
	MOVLW	0xf9
; removed redundant BANKSEL
	MOVWF	(_timer0_reload + 1), B
;	.line	991; meter_logger.c	codec_type = RS232_RX;
	MOVLW	0x02
	BANKSEL	_codec_type
	MOVWF	_codec_type, B
;	.line	994; meter_logger.c	T0CONbits.TMR0ON = 0;
	BCF	_T0CONbits, 7
;	.line	995; meter_logger.c	T0CONbits.T0PS0 = 0;
	BCF	_T0CONbits, 0
;	.line	996; meter_logger.c	T0CONbits.T0PS1 = 0;
	BCF	_T0CONbits, 1
;	.line	997; meter_logger.c	T0CONbits.T0PS2 = 0;		// prescaler 1:2
	BCF	_T0CONbits, 2
;	.line	998; meter_logger.c	T0CONbits.T08BIT = 0;		// use timer0 16-bit counter
	BCF	_T0CONbits, 6
;	.line	999; meter_logger.c	T0CONbits.T0CS = 0;			// internal clock source
	BCF	_T0CONbits, 5
;	.line	1000; meter_logger.c	T0CONbits.PSA = 1;			// disable timer0 prescaler
	BSF	_T0CONbits, 3
;	.line	1001; meter_logger.c	INTCON2bits.TMR0IP = 1;		// high priority
	BSF	_INTCON2bits, 2
;	.line	1002; meter_logger.c	INTCONbits.TMR0IE = 1;		// Enable TMR0 Interrupt
	BSF	_INTCONbits, 5
;	.line	1003; meter_logger.c	INTCONbits.TMR0IF = 0;
	BCF	_INTCONbits, 2
;	.line	1005; meter_logger.c	INTCONbits.INT0IE = 1;		// enable ext int
	BSF	_INTCONbits, 4
;	.line	1006; meter_logger.c	INTCON2bits.INTEDG0 = 1;	// rising edge
	BSF	_INTCON2bits, 6
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__rs232_tx_disable	code
_rs232_tx_disable:
;	.line	979; meter_logger.c	void rs232_tx_disable() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	BANKSEL	_codec_type
;	.line	980; meter_logger.c	codec_type = NONE;
	CLRF	_codec_type, B
;	.line	981; meter_logger.c	IR_LED_PIN = 0;				// no need to set it to inverted idle
	BCF	_PORTBbits, 1
;	.line	982; meter_logger.c	T0CONbits.TMR0ON = 0;
	BCF	_T0CONbits, 7
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__rs232_tx_enable	code
_rs232_tx_enable:
;	.line	953; meter_logger.c	void rs232_tx_enable() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
;	.line	954; meter_logger.c	timer0_reload = TIMER0_RS232_1200;
	MOVLW	0xae
	BANKSEL	_timer0_reload
	MOVWF	_timer0_reload, B
	MOVLW	0xf9
; removed redundant BANKSEL
	MOVWF	(_timer0_reload + 1), B
	BANKSEL	_rs232_proto
;	.line	956; meter_logger.c	rs232_proto.state = INIT_STATE;
	CLRF	_rs232_proto, B
; removed redundant BANKSEL
;	.line	957; meter_logger.c	rs232_proto.data_len = 0;
	CLRF	(_rs232_proto + 3), B
;	.line	959; meter_logger.c	IR_LED_PIN = 0;				// inverted rs232 output on ir, idle = no ir light
	BCF	_PORTBbits, 1
;	.line	961; meter_logger.c	codec_type = RS232_TX;
	MOVLW	0x03
	BANKSEL	_codec_type
	MOVWF	_codec_type, B
;	.line	964; meter_logger.c	T0CONbits.TMR0ON = 0;
	BCF	_T0CONbits, 7
;	.line	965; meter_logger.c	T0CONbits.T0PS0 = 0;
	BCF	_T0CONbits, 0
;	.line	966; meter_logger.c	T0CONbits.T0PS1 = 0;
	BCF	_T0CONbits, 1
;	.line	967; meter_logger.c	T0CONbits.T0PS2 = 0;		// prescaler 1:2
	BCF	_T0CONbits, 2
;	.line	968; meter_logger.c	T0CONbits.T08BIT = 0;		// use timer0 16-bit counter
	BCF	_T0CONbits, 6
;	.line	969; meter_logger.c	T0CONbits.T0CS = 0;			// internal clock source
	BCF	_T0CONbits, 5
;	.line	970; meter_logger.c	T0CONbits.PSA = 1;			// disable timer0 prescaler
	BSF	_T0CONbits, 3
;	.line	971; meter_logger.c	INTCON2bits.TMR0IP = 1;		// high priority
	BSF	_INTCON2bits, 2
;	.line	972; meter_logger.c	INTCONbits.TMR0IE = 1;		// Enable TMR0 Interrupt
	BSF	_INTCONbits, 5
;	.line	973; meter_logger.c	INTCONbits.TMR0IF = 0;
	BCF	_INTCONbits, 2
;	.line	976; meter_logger.c	T0CONbits.TMR0ON = 0;		// timer 0 started in rs232_tx_byte()
	BCF	_T0CONbits, 7
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__testo_ir_disable	code
_testo_ir_disable:
;	.line	948; meter_logger.c	void testo_ir_disable() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	BANKSEL	_codec_type
;	.line	949; meter_logger.c	codec_type = NONE;
	CLRF	_codec_type, B
;	.line	950; meter_logger.c	INTCONbits.INT0IE = 0;		// disable ext int
	BCF	_INTCONbits, 4
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__testo_ir_enable	code
_testo_ir_enable:
;	.line	924; meter_logger.c	void testo_ir_enable() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	BANKSEL	_testo_ir_proto
;	.line	925; meter_logger.c	testo_ir_proto.state = INIT_STATE;
	CLRF	_testo_ir_proto, B
; removed redundant BANKSEL
;	.line	926; meter_logger.c	testo_ir_proto.start_bit_len = 0;
	CLRF	(_testo_ir_proto + 2), B
;	.line	928; meter_logger.c	timer0_reload = TIMER0_TESTO;
	MOVLW	0x23
	BANKSEL	_timer0_reload
	MOVWF	_timer0_reload, B
	MOVLW	0xf3
; removed redundant BANKSEL
	MOVWF	(_timer0_reload + 1), B
;	.line	930; meter_logger.c	codec_type = TESTO;
	MOVLW	0x01
	BANKSEL	_codec_type
	MOVWF	_codec_type, B
;	.line	933; meter_logger.c	T0CONbits.TMR0ON = 0;
	BCF	_T0CONbits, 7
;	.line	934; meter_logger.c	T0CONbits.T0PS0 = 0;
	BCF	_T0CONbits, 0
;	.line	935; meter_logger.c	T0CONbits.T0PS1 = 0;
	BCF	_T0CONbits, 1
;	.line	936; meter_logger.c	T0CONbits.T0PS2 = 0;		// prescaler 1:2
	BCF	_T0CONbits, 2
;	.line	937; meter_logger.c	T0CONbits.T08BIT = 0;		// use timer0 16-bit counter
	BCF	_T0CONbits, 6
;	.line	938; meter_logger.c	T0CONbits.T0CS = 0;			// internal clock source
	BCF	_T0CONbits, 5
;	.line	939; meter_logger.c	T0CONbits.PSA = 1;			// disable timer0 prescaler
	BSF	_T0CONbits, 3
;	.line	940; meter_logger.c	INTCON2bits.TMR0IP = 1;		// high priority
	BSF	_INTCON2bits, 2
;	.line	941; meter_logger.c	INTCONbits.TMR0IE = 1;		// Enable TMR0 Interrupt
	BSF	_INTCONbits, 5
;	.line	942; meter_logger.c	INTCONbits.TMR0IF = 0;
	BCF	_INTCONbits, 2
;	.line	944; meter_logger.c	INTCONbits.INT0IE = 1;		// enable ext int
	BSF	_INTCONbits, 4
;	.line	945; meter_logger.c	INTCON2bits.INTEDG0 = 1;	// rising edge
	BSF	_INTCON2bits, 6
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__testo_valid_err_corr	code
_testo_valid_err_corr:
;	.line	869; meter_logger.c	unsigned char testo_valid_err_corr(unsigned int c) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVFF	r0x01, POSTDEC1
	MOVFF	r0x02, POSTDEC1
	MOVFF	r0x03, POSTDEC1
	MOVFF	r0x04, POSTDEC1
	MOVFF	r0x05, POSTDEC1
	MOVFF	r0x06, POSTDEC1
	MOVFF	r0x07, POSTDEC1
	MOVFF	r0x08, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
	MOVLW	0x03
	MOVFF	PLUSW2, r0x01
;	.line	876; meter_logger.c	calculated_err_corr_bit = 0;
	CLRF	r0x02
;	.line	877; meter_logger.c	for (i = 0; i < 8; i++) {
	MOVLW	0x78
	ANDWF	r0x00, W
	MOVWF	r0x03
	CLRF	r0x04
	CLRF	r0x05
_00654_DS_:
;	.line	878; meter_logger.c	calculated_err_corr_bit ^= (((c & 0x78) & (1 << i)) != 0);   // 0b01111000
	MOVLW	0x01
	MOVWF	r0x06
	MOVLW	0x00
	MOVWF	r0x07
	MOVF	r0x05, W
	BZ	_00696_DS_
	NEGF	WREG
	BCF	STATUS, 0
_00697_DS_:
	RLCF	r0x06, F
	RLCF	r0x07, F
	ADDLW	0x01
	BNC	_00697_DS_
_00696_DS_:
	MOVF	r0x03, W
	ANDWF	r0x06, F
	MOVF	r0x04, W
	ANDWF	r0x07, F
	MOVF	r0x06, W
	BNZ	_00699_DS_
	MOVF	r0x07, W
	BNZ	_00699_DS_
	CLRF	r0x06
	INCF	r0x06, F
	BRA	_00700_DS_
_00699_DS_:
	CLRF	r0x06
_00700_DS_:
	MOVF	r0x06, W
	BSF	STATUS, 0
	TSTFSZ	WREG
	BCF	STATUS, 0
	CLRF	r0x06
	RLCF	r0x06, F
	MOVF	r0x02, W
	MOVWF	r0x07
	MOVF	r0x06, W
	XORWF	r0x07, W
	MOVWF	r0x02
;	.line	877; meter_logger.c	for (i = 0; i < 8; i++) {
	INCF	r0x05, F
	MOVLW	0x08
	SUBWF	r0x05, W
	BNC	_00654_DS_
;	.line	881; meter_logger.c	calculated_err_corr = calculated_err_corr << 1;
	RLNCF	r0x02, W
	ANDLW	0xfe
	MOVWF	r0x03
;	.line	884; meter_logger.c	calculated_err_corr_bit = 0;
	CLRF	r0x02
;	.line	885; meter_logger.c	for (i = 0; i < 8; i++) {
	MOVLW	0xe6
	ANDWF	r0x00, W
	MOVWF	r0x04
	CLRF	r0x05
	CLRF	r0x06
_00656_DS_:
;	.line	886; meter_logger.c	calculated_err_corr_bit ^= (((c & 0xe6) & (1 << i)) != 0);   // 0b11100110
	MOVLW	0x01
	MOVWF	r0x07
	MOVLW	0x00
	MOVWF	r0x08
	MOVF	r0x06, W
	BZ	_00704_DS_
	NEGF	WREG
	BCF	STATUS, 0
_00705_DS_:
	RLCF	r0x07, F
	RLCF	r0x08, F
	ADDLW	0x01
	BNC	_00705_DS_
_00704_DS_:
	MOVF	r0x04, W
	ANDWF	r0x07, F
	MOVF	r0x05, W
	ANDWF	r0x08, F
	MOVF	r0x07, W
	BNZ	_00707_DS_
	MOVF	r0x08, W
	BNZ	_00707_DS_
	CLRF	r0x07
	INCF	r0x07, F
	BRA	_00708_DS_
_00707_DS_:
	CLRF	r0x07
_00708_DS_:
	MOVF	r0x07, W
	BSF	STATUS, 0
	TSTFSZ	WREG
	BCF	STATUS, 0
	CLRF	r0x07
	RLCF	r0x07, F
	MOVF	r0x02, W
	MOVWF	r0x08
	MOVF	r0x07, W
	XORWF	r0x08, W
	MOVWF	r0x02
;	.line	885; meter_logger.c	for (i = 0; i < 8; i++) {
	INCF	r0x06, F
	MOVLW	0x08
	SUBWF	r0x06, W
	BNC	_00656_DS_
;	.line	888; meter_logger.c	calculated_err_corr |= calculated_err_corr_bit;
	MOVF	r0x02, W
	IORWF	r0x03, F
;	.line	889; meter_logger.c	calculated_err_corr = calculated_err_corr << 1;
	BCF	STATUS, 0
	RLCF	r0x03, F
;	.line	892; meter_logger.c	calculated_err_corr_bit = 0;
	CLRF	r0x02
;	.line	893; meter_logger.c	for (i = 0; i < 8; i++) {
	MOVLW	0xd5
	ANDWF	r0x00, W
	MOVWF	r0x04
	CLRF	r0x05
	CLRF	r0x06
_00658_DS_:
;	.line	894; meter_logger.c	calculated_err_corr_bit ^= (((c & 0xd5) & (1 << i)) != 0);   // 0b11010101
	MOVLW	0x01
	MOVWF	r0x07
	MOVLW	0x00
	MOVWF	r0x08
	MOVF	r0x06, W
	BZ	_00713_DS_
	NEGF	WREG
	BCF	STATUS, 0
_00714_DS_:
	RLCF	r0x07, F
	RLCF	r0x08, F
	ADDLW	0x01
	BNC	_00714_DS_
_00713_DS_:
	MOVF	r0x04, W
	ANDWF	r0x07, F
	MOVF	r0x05, W
	ANDWF	r0x08, F
	MOVF	r0x07, W
	BNZ	_00716_DS_
	MOVF	r0x08, W
	BNZ	_00716_DS_
	CLRF	r0x07
	INCF	r0x07, F
	BRA	_00717_DS_
_00716_DS_:
	CLRF	r0x07
_00717_DS_:
	MOVF	r0x07, W
	BSF	STATUS, 0
	TSTFSZ	WREG
	BCF	STATUS, 0
	CLRF	r0x07
	RLCF	r0x07, F
	MOVF	r0x02, W
	MOVWF	r0x08
	MOVF	r0x07, W
	XORWF	r0x08, W
	MOVWF	r0x02
;	.line	893; meter_logger.c	for (i = 0; i < 8; i++) {
	INCF	r0x06, F
	MOVLW	0x08
	SUBWF	r0x06, W
	BNC	_00658_DS_
;	.line	896; meter_logger.c	calculated_err_corr |= calculated_err_corr_bit;
	MOVF	r0x02, W
	IORWF	r0x03, F
;	.line	897; meter_logger.c	calculated_err_corr = calculated_err_corr << 1;
	BCF	STATUS, 0
	RLCF	r0x03, F
;	.line	900; meter_logger.c	calculated_err_corr_bit = 0;
	CLRF	r0x02
;	.line	901; meter_logger.c	for (i = 0; i < 8; i++) {
	MOVLW	0x8b
	ANDWF	r0x00, W
	MOVWF	r0x04
	CLRF	r0x05
	CLRF	r0x06
_00660_DS_:
;	.line	902; meter_logger.c	calculated_err_corr_bit ^= (((c & 0x8b) & (1 << i)) != 0);   // 0b10001011
	MOVLW	0x01
	MOVWF	r0x07
	MOVLW	0x00
	MOVWF	r0x08
	MOVF	r0x06, W
	BZ	_00722_DS_
	NEGF	WREG
	BCF	STATUS, 0
_00723_DS_:
	RLCF	r0x07, F
	RLCF	r0x08, F
	ADDLW	0x01
	BNC	_00723_DS_
_00722_DS_:
	MOVF	r0x04, W
	ANDWF	r0x07, F
	MOVF	r0x05, W
	ANDWF	r0x08, F
	MOVF	r0x07, W
	BNZ	_00725_DS_
	MOVF	r0x08, W
	BNZ	_00725_DS_
	CLRF	r0x07
	INCF	r0x07, F
	BRA	_00726_DS_
_00725_DS_:
	CLRF	r0x07
_00726_DS_:
	MOVF	r0x07, W
	BSF	STATUS, 0
	TSTFSZ	WREG
	BCF	STATUS, 0
	CLRF	r0x07
	RLCF	r0x07, F
	MOVF	r0x02, W
	MOVWF	r0x08
	MOVF	r0x07, W
	XORWF	r0x08, W
	MOVWF	r0x02
;	.line	901; meter_logger.c	for (i = 0; i < 8; i++) {
	INCF	r0x06, F
	MOVLW	0x08
	SUBWF	r0x06, W
	BNC	_00660_DS_
;	.line	904; meter_logger.c	calculated_err_corr |= calculated_err_corr_bit;
	MOVF	r0x02, W
	IORWF	r0x03, F
;	.line	915; meter_logger.c	if ((c >> 8) == calculated_err_corr) {
	MOVF	r0x01, W
	MOVWF	r0x00
	CLRF	r0x01
	CLRF	r0x02
	MOVF	r0x00, W
	XORWF	r0x03, W
	BNZ	_00730_DS_
	MOVF	r0x01, W
	XORWF	r0x02, W
	BZ	_00731_DS_
_00730_DS_:
	BRA	_00652_DS_
_00731_DS_:
;	.line	916; meter_logger.c	return 1;
	MOVLW	0x01
	BRA	_00662_DS_
_00652_DS_:
;	.line	919; meter_logger.c	return 0;
	CLRF	WREG
_00662_DS_:
	MOVFF	PREINC1, r0x08
	MOVFF	PREINC1, r0x07
	MOVFF	PREINC1, r0x06
	MOVFF	PREINC1, r0x05
	MOVFF	PREINC1, r0x04
	MOVFF	PREINC1, r0x03
	MOVFF	PREINC1, r0x02
	MOVFF	PREINC1, r0x01
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__reverse	code
_reverse:
;	.line	861; meter_logger.c	unsigned char reverse(unsigned char b) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVFF	r0x01, POSTDEC1
	MOVFF	r0x02, POSTDEC1
	MOVFF	r0x03, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
;	.line	863; meter_logger.c	c  = ((b >>  1) & 0x55) | ((b <<  1) & 0xaa);
	RRNCF	r0x00, W
	ANDLW	0x7f
	MOVWF	r0x01
	MOVLW	0x55
	ANDWF	r0x01, F
	RLNCF	r0x00, W
	ANDLW	0xfe
	MOVWF	r0x02
	MOVLW	0xaa
	ANDWF	r0x02, F
	MOVF	r0x02, W
	IORWF	r0x01, F
;	.line	864; meter_logger.c	c |= ((b >>  2) & 0x33) | ((b <<  2) & 0xcc);
	RRNCF	r0x00, W
	RRNCF	WREG, W
	ANDLW	0x3f
	MOVWF	r0x02
	MOVLW	0x33
	ANDWF	r0x02, F
	RLNCF	r0x00, W
	RLNCF	WREG, W
	ANDLW	0xfc
	MOVWF	r0x03
	MOVLW	0xcc
	ANDWF	r0x03, F
	MOVF	r0x03, W
	IORWF	r0x02, F
	MOVF	r0x02, W
	IORWF	r0x01, F
;	.line	865; meter_logger.c	c |= ((b >>  4) & 0x0f) | ((b <<  4) & 0xf0);
	SWAPF	r0x00, W
	ANDLW	0x0f
	MOVWF	r0x02
	MOVLW	0x0f
	ANDWF	r0x02, F
	SWAPF	r0x00, W
	ANDLW	0xf0
	MOVWF	r0x03
	MOVLW	0xf0
	ANDWF	r0x03, W
	MOVWF	r0x00
	MOVF	r0x00, W
	IORWF	r0x02, F
	MOVF	r0x02, W
	IORWF	r0x01, F
;	.line	866; meter_logger.c	return(c);
	MOVF	r0x01, W
	MOVFF	PREINC1, r0x03
	MOVFF	PREINC1, r0x02
	MOVFF	PREINC1, r0x01
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__my_usart_open	code
_my_usart_open:
;	.line	831; meter_logger.c	void my_usart_open() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
;	.line	832; meter_logger.c	SPBRG = 103;					// 8MHz => 19230 baud
	MOVLW	0x67
	MOVWF	_SPBRG
;	.line	833; meter_logger.c	TXSTAbits.BRGH = 1;	// (0 = low speed)
	BSF	_TXSTAbits, 2
;	.line	834; meter_logger.c	TXSTAbits.SYNC = 0;	// (0 = asynchronous)
	BCF	_TXSTAbits, 4
;	.line	835; meter_logger.c	BAUDCONbits.BRG16 = 1;
	BSF	_BAUDCONbits, 3
;	.line	838; meter_logger.c	RCSTAbits.SPEN = 1; // (1 = serial port enabled)
	BSF	_RCSTAbits, 7
;	.line	841; meter_logger.c	PIE1bits.TXIE = 0; // (1 = enabled)
	BCF	_PIE1bits, 4
;	.line	842; meter_logger.c	IPR1bits.TXIP = 0; // USART Tx on low priority interrupt
	BCF	_IPR1bits, 4
;	.line	845; meter_logger.c	PIE1bits.RCIE = 1; // (1 = enabled)
	BSF	_PIE1bits, 5
;	.line	846; meter_logger.c	IPR1bits.RCIP = 0; // USART Rx on low priority interrupt
	BCF	_IPR1bits, 5
;	.line	849; meter_logger.c	TXSTAbits.TX9 = 0; // (0 = 8-bit transmit)
	BCF	_TXSTAbits, 6
;	.line	852; meter_logger.c	RCSTAbits.RX9 = 0; // (0 = 8-bit reception)
	BCF	_RCSTAbits, 6
;	.line	855; meter_logger.c	RCSTAbits.CREN = 1; // (1 = Enables receiver)
	BSF	_RCSTAbits, 4
;	.line	858; meter_logger.c	TXSTAbits.TXEN = 1; // (1 = transmit enabled)
	BSF	_TXSTAbits, 5
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__init_system	code
_init_system:
;	.line	732; meter_logger.c	void init_system() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
;	.line	734; meter_logger.c	TRIS_COMP1 = INPUT_STATE;		// as input
	BSF	_TRISAbits, 0
;	.line	735; meter_logger.c	TRIS_COMP2 = INPUT_STATE;		// as input
	BSF	_TRISAbits, 1
;	.line	737; meter_logger.c	TRIS_IR_PIN = INPUT_STATE;		// as input
	BSF	_TRISBbits, 0
;	.line	739; meter_logger.c	TRIS_LED_PIN = OUTPUT_STATE;	// as output
	BCF	_TRISBbits, 4
;	.line	740; meter_logger.c	LED_PIN = 0;					// and clear
	BCF	_PORTBbits, 4
;	.line	742; meter_logger.c	TRIS_IR_LED_PIN = OUTPUT_STATE;	// as output
	BCF	_TRISBbits, 1
;	.line	743; meter_logger.c	IR_LED_PIN = 0;					// and clear
	BCF	_PORTBbits, 1
;	.line	745; meter_logger.c	TRIS_DEBUG_PIN = OUTPUT_STATE;	// as output
	BCF	_TRISBbits, 2
;	.line	746; meter_logger.c	DEBUG_PIN = 0;					// and clear
	BCF	_PORTBbits, 2
;	.line	748; meter_logger.c	TRIS_DEBUG2_PIN = OUTPUT_STATE;	// as output
	BCF	_TRISBbits, 3
;	.line	749; meter_logger.c	DEBUG2_PIN = 0;					// and clear
	BCF	_PORTBbits, 3
;	.line	751; meter_logger.c	TRIS_DEBUG3_PIN = OUTPUT_STATE;	// as output
	BCF	_TRISBbits, 4
;	.line	752; meter_logger.c	DEBUG3_PIN = 0;					// and clear
	BCF	_PORTBbits, 4
;	.line	756; meter_logger.c	TRIS_PWM_PIN = OUTPUT_STATE;	// enable output from pwm module at init
	BCF	_TRISCbits, 1
;	.line	757; meter_logger.c	PWM_PIN = 0;					// and clear
	BCF	_PORTCbits, 1
;	.line	760; meter_logger.c	TRIS_RX_PIN = INPUT_STATE;		// as input
	BSF	_TRISCbits, 7
;	.line	761; meter_logger.c	TRIS_TX_PIN = OUTPUT_STATE;		// as input
	BCF	_TRISCbits, 6
;	.line	766; meter_logger.c	T1CONbits.TMR1ON = 1;
	BSF	_T1CONbits, 0
;	.line	767; meter_logger.c	T1CONbits.RD16 = 1;
	BSF	_T1CONbits, 7
;	.line	768; meter_logger.c	T1CONbits.TMR1CS = 0;   // internal clock source
	BCF	_T1CONbits, 1
;	.line	769; meter_logger.c	T1CONbits.T1OSCEN = 0;  // dont put t1 on pin
	BCF	_T1CONbits, 3
;	.line	770; meter_logger.c	T1CONbits.T1CKPS0 = 0;
	BCF	_T1CONbits, 4
;	.line	771; meter_logger.c	T1CONbits.T1CKPS1 = 0;
	BCF	_T1CONbits, 5
;	.line	772; meter_logger.c	IPR1bits.TMR1IP = 0;	// low priority
	BCF	_IPR1bits, 0
;	.line	773; meter_logger.c	PIE1bits.TMR1IE = 1;	// Ensure that TMR1 Interrupt is enabled
	BSF	_PIE1bits, 0
;	.line	774; meter_logger.c	PIR1bits.TMR1IF = 1;	// Force Instant entry to Timer 1 Interrupt
	BSF	_PIR1bits, 0
;	.line	805; meter_logger.c	RCONbits.IPEN = 1;
	BSF	_RCONbits, 7
;	.line	807; meter_logger.c	INTCONbits.INT0IE = 0;		// disable ext int, enabled when ir demodulator is started
	BCF	_INTCONbits, 4
;	.line	808; meter_logger.c	INTCON2bits.INTEDG0 = 1;	// rising edge
	BSF	_INTCON2bits, 6
;	.line	810; meter_logger.c	INTCONbits.PEIE = 1;
	BSF	_INTCONbits, 6
;	.line	811; meter_logger.c	INTCONbits.GIE = 1;	/* Enable Global interrupts   */	
	BSF	_INTCONbits, 7
;	.line	816; meter_logger.c	IPR1bits.RCIP = 0;
	BCF	_IPR1bits, 5
;	.line	817; meter_logger.c	IPR1bits.TXIP = 0;
	BCF	_IPR1bits, 4
;	.line	828; meter_logger.c	my_usart_open();
	CALL	_my_usart_open
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__sleep_ms	code
_sleep_ms:
;	.line	722; meter_logger.c	void sleep_ms(unsigned long ms) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVFF	r0x01, POSTDEC1
	MOVFF	r0x02, POSTDEC1
	MOVFF	r0x03, POSTDEC1
	MOVFF	r0x04, POSTDEC1
	MOVFF	r0x05, POSTDEC1
	MOVFF	r0x06, POSTDEC1
	MOVFF	r0x07, POSTDEC1
	MOVFF	r0x08, POSTDEC1
	MOVFF	r0x09, POSTDEC1
	MOVFF	r0x0a, POSTDEC1
	MOVFF	r0x0b, POSTDEC1
	MOVFF	r0x0c, POSTDEC1
	MOVFF	r0x0d, POSTDEC1
	MOVFF	r0x0e, POSTDEC1
	MOVFF	r0x0f, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
	MOVLW	0x03
	MOVFF	PLUSW2, r0x01
	MOVLW	0x04
	MOVFF	PLUSW2, r0x02
	MOVLW	0x05
	MOVFF	PLUSW2, r0x03
;	.line	724; meter_logger.c	start_timer_1_ms = timer_1_ms;	
	MOVFF	_timer_1_ms, r0x04
	MOVFF	(_timer_1_ms + 1), r0x05
	MOVFF	(_timer_1_ms + 2), r0x06
	MOVFF	(_timer_1_ms + 3), r0x07
_00614_DS_:
;	.line	727; meter_logger.c	while ( (((signed long)(timer_1_ms - start_timer_1_ms) < 0) ? (-1 * (timer_1_ms - start_timer_1_ms)) : (timer_1_ms - start_timer_1_ms)) < ms) {
	MOVF	r0x04, W
	BANKSEL	_timer_1_ms
	SUBWF	_timer_1_ms, W, B
	MOVWF	r0x08
	MOVF	r0x05, W
; removed redundant BANKSEL
	SUBWFB	(_timer_1_ms + 1), W, B
	MOVWF	r0x09
	MOVF	r0x06, W
; removed redundant BANKSEL
	SUBWFB	(_timer_1_ms + 2), W, B
	MOVWF	r0x0a
	MOVF	r0x07, W
; removed redundant BANKSEL
	SUBWFB	(_timer_1_ms + 3), W, B
	MOVWF	r0x0b
	MOVF	r0x08, W
	MOVWF	r0x0c
	MOVF	r0x09, W
	MOVWF	r0x0d
	MOVF	r0x0a, W
	MOVWF	r0x0e
	MOVF	r0x0b, W
	MOVWF	r0x0f
	BSF	STATUS, 0
	BTFSS	r0x0f, 7
	BCF	STATUS, 0
	BNC	_00619_DS_
	MOVF	r0x0b, W
	MOVWF	POSTDEC1
	MOVF	r0x0a, W
	MOVWF	POSTDEC1
	MOVF	r0x09, W
	MOVWF	POSTDEC1
	MOVF	r0x08, W
	MOVWF	POSTDEC1
	MOVLW	0xff
	MOVWF	POSTDEC1
	MOVLW	0xff
	MOVWF	POSTDEC1
	MOVLW	0xff
	MOVWF	POSTDEC1
	MOVLW	0xff
	MOVWF	POSTDEC1
	CALL	__mullong
	MOVWF	r0x0c
	MOVFF	PRODL, r0x0d
	MOVFF	PRODH, r0x0e
	MOVFF	FSR0L, r0x0f
	MOVLW	0x08
	ADDWF	FSR1L, F
	BRA	_00620_DS_
_00619_DS_:
	MOVFF	r0x08, r0x0c
	MOVFF	r0x09, r0x0d
	MOVFF	r0x0a, r0x0e
	MOVFF	r0x0b, r0x0f
_00620_DS_:
	MOVF	r0x03, W
	SUBWF	r0x0f, W
	BNZ	_00627_DS_
	MOVF	r0x02, W
	SUBWF	r0x0e, W
	BNZ	_00627_DS_
	MOVF	r0x01, W
	SUBWF	r0x0d, W
	BNZ	_00627_DS_
	MOVF	r0x00, W
	SUBWF	r0x0c, W
_00627_DS_:
	BTFSS	STATUS, 0
	BRA	_00614_DS_
	MOVFF	PREINC1, r0x0f
	MOVFF	PREINC1, r0x0e
	MOVFF	PREINC1, r0x0d
	MOVFF	PREINC1, r0x0c
	MOVFF	PREINC1, r0x0b
	MOVFF	PREINC1, r0x0a
	MOVFF	PREINC1, r0x09
	MOVFF	PREINC1, r0x08
	MOVFF	PREINC1, r0x07
	MOVFF	PREINC1, r0x06
	MOVFF	PREINC1, r0x05
	MOVFF	PREINC1, r0x04
	MOVFF	PREINC1, r0x03
	MOVFF	PREINC1, r0x02
	MOVFF	PREINC1, r0x01
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__isr_low_prio	code
_isr_low_prio:
;	.line	690; meter_logger.c	static void isr_low_prio(void) __interrupt 2 {
	MOVFF	STATUS, POSTDEC1
	MOVFF	BSR, POSTDEC1
	MOVWF	POSTDEC1
	MOVFF	PRODL, POSTDEC1
	MOVFF	PRODH, POSTDEC1
	MOVFF	FSR0L, POSTDEC1
	MOVFF	FSR0H, POSTDEC1
	MOVFF	PCLATH, POSTDEC1
	MOVFF	PCLATU, POSTDEC1
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVFF	r0x01, POSTDEC1
;	.line	693; meter_logger.c	if (PIR1bits.TMR1IF) {
	BTFSS	_PIR1bits, 0
	BRA	_00585_DS_
;	.line	694; meter_logger.c	TMR1H = (unsigned char)(TIMER1_RELOAD >> 8);    // 262,158ms @ 8MHz
	MOVLW	0xf8
	MOVWF	_TMR1H
;	.line	695; meter_logger.c	TMR1L = (unsigned char)TIMER1_RELOAD;
	MOVLW	0x53
	MOVWF	_TMR1L
;	.line	697; meter_logger.c	switch (led_flash.state) {
	MOVFF	_led_flash, r0x00
	MOVF	r0x00, W
	MOVWF	r0x01
	MOVF	r0x01, W
	BZ	_00579_DS_
	MOVF	r0x00, W
	XORLW	0x01
	BZ	_00580_DS_
	BRA	_00583_DS_
_00579_DS_:
;	.line	699; meter_logger.c	LED_PIN = 1;
	BSF	_PORTBbits, 4
;	.line	700; meter_logger.c	led_flash.state = LED_FLASH_RUNNING;
	MOVLW	0x01
	BANKSEL	_led_flash
	MOVWF	_led_flash, B
;	.line	701; meter_logger.c	break;
	BRA	_00583_DS_
_00580_DS_:
;	.line	703; meter_logger.c	if (led_flash.timer-- == 0) {
	MOVFF	(_led_flash + 1), r0x00
	DECF	r0x00, W
	MOVWF	r0x01
	MOVF	r0x01, W
	BANKSEL	(_led_flash + 1)
	MOVWF	(_led_flash + 1), B
	MOVF	r0x00, W
	BNZ	_00583_DS_
;	.line	704; meter_logger.c	LED_PIN = 0;
	BCF	_PORTBbits, 4
;	.line	705; meter_logger.c	led_flash.state = LED_FLASH_STOPPED;
	MOVLW	0x02
; removed redundant BANKSEL
	MOVWF	_led_flash, B
_00583_DS_:
	BANKSEL	_timer_1_ms
;	.line	709; meter_logger.c	timer_1_ms++;
	INCF	_timer_1_ms, F, B
	BNC	_00609_DS_
; removed redundant BANKSEL
	INCF	(_timer_1_ms + 1), F, B
	BNC	_00609_DS_
; removed redundant BANKSEL
	INCFSZ	(_timer_1_ms + 2), F, B
	BRA	_30914_DS_
; removed redundant BANKSEL
	INCF	(_timer_1_ms + 3), F, B
_30914_DS_:
_00609_DS_:
;	.line	710; meter_logger.c	PIR1bits.TMR1IF = 0;    /* Clear the Timer Flag  */
	BCF	_PIR1bits, 0
_00585_DS_:
;	.line	714; meter_logger.c	if (usart_drdy()) {
	CALL	_usart_drdy
	MOVWF	r0x00
	MOVF	r0x00, W
	BZ	_00588_DS_
;	.line	716; meter_logger.c	c = usart_getc();
	CALL	_usart_getc
	MOVWF	r0x00
;	.line	717; meter_logger.c	usart_putc(c);
	MOVF	r0x00, W
	CALL	_usart_putc
_00588_DS_:
	MOVFF	PREINC1, r0x01
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	MOVFF	PREINC1, PCLATU
	MOVFF	PREINC1, PCLATH
	MOVFF	PREINC1, FSR0H
	MOVFF	PREINC1, FSR0L
	MOVFF	PREINC1, PRODH
	MOVFF	PREINC1, PRODL
	MOVF	PREINC1, W
	MOVFF	PREINC1, BSR
	MOVFF	PREINC1, STATUS
	RETFIE	

; ; Starting pCode block
S_meter_logger__isr_high_prio	code
_isr_high_prio:
;	.line	331; meter_logger.c	static void isr_high_prio(void) __interrupt 1 {
	MOVFF	STATUS, POSTDEC1
	MOVFF	BSR, POSTDEC1
	MOVWF	POSTDEC1
	MOVFF	PRODL, POSTDEC1
	MOVFF	PRODH, POSTDEC1
	MOVFF	FSR0L, POSTDEC1
	MOVFF	FSR0H, POSTDEC1
	MOVFF	PCLATH, POSTDEC1
	MOVFF	PCLATU, POSTDEC1
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVFF	r0x01, POSTDEC1
	MOVFF	r0x02, POSTDEC1
	MOVFF	r0x03, POSTDEC1
	MOVFF	r0x04, POSTDEC1
	MOVFF	r0x05, POSTDEC1
;	.line	333; meter_logger.c	if (INTCONbits.INT0IF && INTCONbits.INT0IE) {
	BTFSS	_INTCONbits, 1
	BRA	_00252_DS_
	BTFSS	_INTCONbits, 4
	BRA	_00252_DS_
;	.line	334; meter_logger.c	timer_0 = (unsigned int)(TMR0L) | ((unsigned int)(TMR0H) << 8);
	MOVFF	_TMR0L, r0x00
	CLRF	r0x01
	MOVFF	_TMR0H, r0x02
	CLRF	r0x03
	MOVF	r0x02, W
	MOVWF	r0x05
	CLRF	r0x04
	MOVF	r0x04, W
	IORWF	r0x00, W
	BANKSEL	_timer_0
	MOVWF	_timer_0, B
	MOVF	r0x05, W
	IORWF	r0x01, W
; removed redundant BANKSEL
	MOVWF	(_timer_0 + 1), B
	BANKSEL	(_timer0_reload + 1)
;	.line	335; meter_logger.c	TMR0H = (unsigned char)(timer0_reload >> 8);
	MOVF	(_timer0_reload + 1), W, B
	MOVWF	r0x00
	CLRF	r0x01
	MOVF	r0x00, W
	MOVWF	_TMR0H
; removed redundant BANKSEL
;	.line	336; meter_logger.c	TMR0L = (unsigned char)timer0_reload;
	MOVF	_timer0_reload, W, B
	MOVWF	_TMR0L
	BANKSEL	_codec_type
;	.line	338; meter_logger.c	switch (codec_type) {
	MOVF	_codec_type, W, B
	XORLW	0x01
	BZ	_00213_DS_
_00496_DS_:
	BANKSEL	_codec_type
	MOVF	_codec_type, W, B
	XORLW	0x02
	BNZ	_00498_DS_
	BRA	_00247_DS_
_00498_DS_:
	BRA	_00250_DS_
_00213_DS_:
;	.line	340; meter_logger.c	flash_led(100);
	MOVLW	0x64
	MOVWF	POSTDEC1
	CALL	_flash_led
	MOVF	POSTINC1, F
;	.line	341; meter_logger.c	switch (testo_ir_proto.state) {
	MOVFF	_testo_ir_proto, r0x00
	MOVF	r0x00, W
	MOVWF	r0x01
	MOVF	r0x01, W
	BZ	_00214_DS_
	MOVF	r0x00, W
	XORLW	0x02
	BZ	_00215_DS_
	MOVF	r0x00, W
	XORLW	0x04
	BNZ	_00504_DS_
	BRA	_00223_DS_
_00504_DS_:
	BRA	_00250_DS_
_00214_DS_:
;	.line	343; meter_logger.c	T0CONbits.TMR0ON = 1;		// Start TMR0
	BSF	_T0CONbits, 7
;	.line	344; meter_logger.c	testo_ir_proto.start_bit_len = 1;
	MOVLW	0x01
	BANKSEL	(_testo_ir_proto + 2)
	MOVWF	(_testo_ir_proto + 2), B
;	.line	345; meter_logger.c	testo_ir_proto.state = START_BIT_WAIT;
	MOVLW	0x02
; removed redundant BANKSEL
	MOVWF	_testo_ir_proto, B
;	.line	346; meter_logger.c	break;
	BRA	_00250_DS_
_00215_DS_:
	BANKSEL	_timer0_reload
;	.line	348; meter_logger.c	if ((TICK + timer0_reload - TICK_ADJ < timer_0) && (timer_0 < TICK + timer0_reload + TICK_ADJ)) {
	MOVF	_timer0_reload, W, B
	ADDLW	0x8f
	MOVWF	r0x00
	MOVLW	0x02
; removed redundant BANKSEL
	ADDWFC	(_timer0_reload + 1), W, B
	MOVWF	r0x01
	BANKSEL	(_timer_0 + 1)
	MOVF	(_timer_0 + 1), W, B
	SUBWF	r0x01, W
	BNZ	_00505_DS_
; removed redundant BANKSEL
	MOVF	_timer_0, W, B
	SUBWF	r0x00, W
_00505_DS_:
	BC	_00220_DS_
	BANKSEL	_timer0_reload
	MOVF	_timer0_reload, W, B
	ADDLW	0x1f
	MOVWF	r0x00
	MOVLW	0x04
; removed redundant BANKSEL
	ADDWFC	(_timer0_reload + 1), W, B
	MOVWF	r0x01
	MOVF	r0x01, W
	BANKSEL	(_timer_0 + 1)
	SUBWF	(_timer_0 + 1), W, B
	BNZ	_00506_DS_
	MOVF	r0x00, W
; removed redundant BANKSEL
	SUBWF	_timer_0, W, B
_00506_DS_:
	BC	_00220_DS_
;	.line	349; meter_logger.c	if (testo_ir_proto.start_bit_len < 2) {
	MOVLW	0x02
	BANKSEL	(_testo_ir_proto + 2)
	SUBWF	(_testo_ir_proto + 2), W, B
	BC	_00217_DS_
;	.line	350; meter_logger.c	testo_ir_proto.start_bit_len++;
	MOVFF	(_testo_ir_proto + 2), r0x00
	INCF	r0x00, F
	MOVF	r0x00, W
; removed redundant BANKSEL
	MOVWF	(_testo_ir_proto + 2), B
	BRA	_00250_DS_
_00217_DS_:
	BANKSEL	(_testo_ir_proto + 3)
;	.line	354; meter_logger.c	testo_ir_proto.data = 0;
	CLRF	(_testo_ir_proto + 3), B
; removed redundant BANKSEL
	CLRF	(_testo_ir_proto + 4), B
; removed redundant BANKSEL
;	.line	355; meter_logger.c	testo_ir_proto.data_len = 0;
	CLRF	(_testo_ir_proto + 5), B
;	.line	356; meter_logger.c	testo_ir_proto.state = DATA_WAIT;
	MOVLW	0x04
; removed redundant BANKSEL
	MOVWF	_testo_ir_proto, B
	BRA	_00250_DS_
_00220_DS_:
;	.line	361; meter_logger.c	testo_ir_proto.start_bit_len = 1;
	MOVLW	0x01
	BANKSEL	(_testo_ir_proto + 2)
	MOVWF	(_testo_ir_proto + 2), B
;	.line	362; meter_logger.c	testo_ir_proto.state = START_BIT_WAIT;
	MOVLW	0x02
; removed redundant BANKSEL
	MOVWF	_testo_ir_proto, B
;	.line	364; meter_logger.c	break;
	BRA	_00250_DS_
_00223_DS_:
;	.line	366; meter_logger.c	if (testo_ir_proto.data_len <= 12) {
	MOVLW	0x0d
	BANKSEL	(_testo_ir_proto + 5)
	SUBWF	(_testo_ir_proto + 5), W, B
	BTFSC	STATUS, 0
	BRA	_00250_DS_
	BANKSEL	_timer0_reload
;	.line	367; meter_logger.c	if (((TICK + timer0_reload - TICK_ADJ < timer_0) && (timer_0 < TICK + timer0_reload + TICK_ADJ)) || ((3 * TICK + timer0_reload - TICK_ADJ < timer_0) && (timer_0 < 3 * TICK + timer0_reload + TICK_ADJ))) {
	MOVF	_timer0_reload, W, B
	ADDLW	0x8f
	MOVWF	r0x00
	MOVLW	0x02
; removed redundant BANKSEL
	ADDWFC	(_timer0_reload + 1), W, B
	MOVWF	r0x01
	BANKSEL	(_timer_0 + 1)
	MOVF	(_timer_0 + 1), W, B
	SUBWF	r0x01, W
	BNZ	_00509_DS_
; removed redundant BANKSEL
	MOVF	_timer_0, W, B
	SUBWF	r0x00, W
_00509_DS_:
	BC	_00239_DS_
	BANKSEL	_timer0_reload
	MOVF	_timer0_reload, W, B
	ADDLW	0x1f
	MOVWF	r0x00
	MOVLW	0x04
; removed redundant BANKSEL
	ADDWFC	(_timer0_reload + 1), W, B
	MOVWF	r0x01
	MOVF	r0x01, W
	BANKSEL	(_timer_0 + 1)
	SUBWF	(_timer_0 + 1), W, B
	BNZ	_00510_DS_
	MOVF	r0x00, W
; removed redundant BANKSEL
	SUBWF	_timer_0, W, B
_00510_DS_:
	BNC	_00234_DS_
_00239_DS_:
	BANKSEL	_timer0_reload
	MOVF	_timer0_reload, W, B
	ADDLW	0x3d
	MOVWF	r0x00
	MOVLW	0x09
; removed redundant BANKSEL
	ADDWFC	(_timer0_reload + 1), W, B
	MOVWF	r0x01
	BANKSEL	(_timer_0 + 1)
	MOVF	(_timer_0 + 1), W, B
	SUBWF	r0x01, W
	BNZ	_00511_DS_
; removed redundant BANKSEL
	MOVF	_timer_0, W, B
	SUBWF	r0x00, W
_00511_DS_:
	BC	_00235_DS_
	BANKSEL	_timer0_reload
	MOVF	_timer0_reload, W, B
	ADDLW	0xcd
	MOVWF	r0x00
	MOVLW	0x0a
; removed redundant BANKSEL
	ADDWFC	(_timer0_reload + 1), W, B
	MOVWF	r0x01
	MOVF	r0x01, W
	BANKSEL	(_timer_0 + 1)
	SUBWF	(_timer_0 + 1), W, B
	BNZ	_00512_DS_
	MOVF	r0x00, W
; removed redundant BANKSEL
	SUBWF	_timer_0, W, B
_00512_DS_:
	BC	_00235_DS_
_00234_DS_:
	BANKSEL	(_testo_ir_proto + 3)
;	.line	369; meter_logger.c	if ((testo_ir_proto.data & 1) != 0) {
	BTFSS	(_testo_ir_proto + 3), 0
	BRA	_00225_DS_
; removed redundant BANKSEL
;	.line	371; meter_logger.c	testo_ir_proto.data <<= 1;		// bitshift once to left
	MOVF	(_testo_ir_proto + 3), W, B
	MOVWF	r0x00
	ADDWF	r0x00, F
; removed redundant BANKSEL
	RLCF	(_testo_ir_proto + 4), W, B
	MOVWF	r0x01
	MOVF	r0x00, W
; removed redundant BANKSEL
	MOVWF	(_testo_ir_proto + 3), B
	MOVF	r0x01, W
; removed redundant BANKSEL
	MOVWF	(_testo_ir_proto + 4), B
	BRA	_00226_DS_
_00225_DS_:
	BANKSEL	(_testo_ir_proto + 3)
;	.line	376; meter_logger.c	testo_ir_proto.data <<= 1;		// bitshift once to left
	MOVF	(_testo_ir_proto + 3), W, B
	MOVWF	r0x00
	ADDWF	r0x00, F
; removed redundant BANKSEL
	RLCF	(_testo_ir_proto + 4), W, B
	MOVWF	r0x01
	MOVF	r0x00, W
; removed redundant BANKSEL
	MOVWF	(_testo_ir_proto + 3), B
	MOVF	r0x01, W
; removed redundant BANKSEL
	MOVWF	(_testo_ir_proto + 4), B
;	.line	377; meter_logger.c	testo_ir_proto.data |= 1;	// and set bit 0
	MOVLW	0x01
; removed redundant BANKSEL
	IORWF	(_testo_ir_proto + 3), W, B
	MOVWF	r0x00
; removed redundant BANKSEL
	MOVF	(_testo_ir_proto + 4), W, B
	MOVWF	r0x01
	MOVF	r0x00, W
; removed redundant BANKSEL
	MOVWF	(_testo_ir_proto + 3), B
	MOVF	r0x01, W
; removed redundant BANKSEL
	MOVWF	(_testo_ir_proto + 4), B
_00226_DS_:
;	.line	379; meter_logger.c	testo_ir_proto.data_len++;
	MOVFF	(_testo_ir_proto + 5), r0x00
	INCF	r0x00, F
	MOVF	r0x00, W
	BANKSEL	(_testo_ir_proto + 5)
	MOVWF	(_testo_ir_proto + 5), B
	BRA	_00236_DS_
_00235_DS_:
	BANKSEL	_timer0_reload
;	.line	381; meter_logger.c	else if ((2 * TICK + timer0_reload - TICK_ADJ < timer_0) && (timer_0 < 2 * TICK + timer0_reload + TICK_ADJ)) {
	MOVF	_timer0_reload, W, B
	ADDLW	0xe6
	MOVWF	r0x00
	MOVLW	0x05
; removed redundant BANKSEL
	ADDWFC	(_timer0_reload + 1), W, B
	MOVWF	r0x01
	BANKSEL	(_timer_0 + 1)
	MOVF	(_timer_0 + 1), W, B
	SUBWF	r0x01, W
	BNZ	_00515_DS_
; removed redundant BANKSEL
	MOVF	_timer_0, W, B
	SUBWF	r0x00, W
_00515_DS_:
	BC	_00231_DS_
	BANKSEL	_timer0_reload
	MOVF	_timer0_reload, W, B
	ADDLW	0x76
	MOVWF	r0x00
	MOVLW	0x07
; removed redundant BANKSEL
	ADDWFC	(_timer0_reload + 1), W, B
	MOVWF	r0x01
	MOVF	r0x01, W
	BANKSEL	(_timer_0 + 1)
	SUBWF	(_timer_0 + 1), W, B
	BNZ	_00516_DS_
	MOVF	r0x00, W
; removed redundant BANKSEL
	SUBWF	_timer_0, W, B
_00516_DS_:
	BC	_00231_DS_
	BANKSEL	(_testo_ir_proto + 3)
;	.line	383; meter_logger.c	if ((testo_ir_proto.data & 1) != 0) {
	BTFSS	(_testo_ir_proto + 3), 0
	BRA	_00228_DS_
; removed redundant BANKSEL
;	.line	385; meter_logger.c	testo_ir_proto.data <<= 1;		// bitshift once to left
	MOVF	(_testo_ir_proto + 3), W, B
	MOVWF	r0x00
	ADDWF	r0x00, F
; removed redundant BANKSEL
	RLCF	(_testo_ir_proto + 4), W, B
	MOVWF	r0x01
	MOVF	r0x00, W
; removed redundant BANKSEL
	MOVWF	(_testo_ir_proto + 3), B
	MOVF	r0x01, W
; removed redundant BANKSEL
	MOVWF	(_testo_ir_proto + 4), B
;	.line	386; meter_logger.c	testo_ir_proto.data |= 1;	// and set bit 0
	MOVLW	0x01
; removed redundant BANKSEL
	IORWF	(_testo_ir_proto + 3), W, B
	MOVWF	r0x00
; removed redundant BANKSEL
	MOVF	(_testo_ir_proto + 4), W, B
	MOVWF	r0x01
	MOVF	r0x00, W
; removed redundant BANKSEL
	MOVWF	(_testo_ir_proto + 3), B
	MOVF	r0x01, W
; removed redundant BANKSEL
	MOVWF	(_testo_ir_proto + 4), B
	BRA	_00229_DS_
_00228_DS_:
	BANKSEL	(_testo_ir_proto + 3)
;	.line	390; meter_logger.c	testo_ir_proto.data <<= 1;		// bitshift once to left
	MOVF	(_testo_ir_proto + 3), W, B
	MOVWF	r0x00
	ADDWF	r0x00, F
; removed redundant BANKSEL
	RLCF	(_testo_ir_proto + 4), W, B
	MOVWF	r0x01
	MOVF	r0x00, W
; removed redundant BANKSEL
	MOVWF	(_testo_ir_proto + 3), B
	MOVF	r0x01, W
; removed redundant BANKSEL
	MOVWF	(_testo_ir_proto + 4), B
_00229_DS_:
;	.line	393; meter_logger.c	testo_ir_proto.data_len++;
	MOVFF	(_testo_ir_proto + 5), r0x00
	INCF	r0x00, F
	MOVF	r0x00, W
	BANKSEL	(_testo_ir_proto + 5)
	MOVWF	(_testo_ir_proto + 5), B
	BRA	_00236_DS_
_00231_DS_:
;	.line	398; meter_logger.c	testo_ir_proto.start_bit_len = 1;
	MOVLW	0x01
	BANKSEL	(_testo_ir_proto + 2)
	MOVWF	(_testo_ir_proto + 2), B
;	.line	399; meter_logger.c	testo_ir_proto.state = START_BIT_WAIT;
	MOVLW	0x02
; removed redundant BANKSEL
	MOVWF	_testo_ir_proto, B
_00236_DS_:
	BANKSEL	(_testo_ir_proto + 5)
;	.line	401; meter_logger.c	if (testo_ir_proto.data_len == 12) {
	MOVF	(_testo_ir_proto + 5), W, B
	XORLW	0x0c
	BNZ	_00250_DS_
_00520_DS_:
	BANKSEL	(_testo_ir_proto + 4)
;	.line	404; meter_logger.c	if (testo_valid_err_corr(testo_ir_proto.data & 0xffff)) {
	MOVF	(_testo_ir_proto + 4), W, B
	MOVWF	POSTDEC1
; removed redundant BANKSEL
	MOVF	(_testo_ir_proto + 3), W, B
	MOVWF	POSTDEC1
	CALL	_testo_valid_err_corr
	MOVWF	r0x00
	MOVF	POSTINC1, F
	MOVF	POSTINC1, F
	MOVF	r0x00, W
	BZ	_00241_DS_
	BANKSEL	(_testo_ir_proto + 3)
;	.line	406; meter_logger.c	fifo_put(testo_ir_proto.data & 0xff);
	MOVF	(_testo_ir_proto + 3), W, B
	MOVWF	r0x00
	CLRF	r0x01
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	_fifo_put
	MOVF	POSTINC1, F
;	.line	407; meter_logger.c	LED_PIN = 1;
	BSF	_PORTBbits, 4
_00241_DS_:
	BANKSEL	_testo_ir_proto
;	.line	409; meter_logger.c	testo_ir_proto.state = INIT_STATE;
	CLRF	_testo_ir_proto, B
;	.line	414; meter_logger.c	break;
	BRA	_00250_DS_
_00247_DS_:
	BANKSEL	_rs232_proto
;	.line	416; meter_logger.c	switch (rs232_proto.state) {
	MOVF	_rs232_proto, W, B
	XORLW	0x02
	BNZ	_00250_DS_
;	.line	427; meter_logger.c	TMR0H = (unsigned char)(TIMER0_RS232_1200_START >> 8);
	MOVLW	0xf7
	MOVWF	_TMR0H
;	.line	428; meter_logger.c	TMR0L = (unsigned char)TIMER0_RS232_1200_START;
	CLRF	_TMR0L
;	.line	429; meter_logger.c	INTCONbits.INT0IE = 0;		// disable ext int while we are using timer to receive data bits
	BCF	_INTCONbits, 4
;	.line	430; meter_logger.c	T0CONbits.TMR0ON = 1;		// Start TMR0
	BSF	_T0CONbits, 7
;	.line	431; meter_logger.c	rs232_proto.state = DATA_WAIT;
	MOVLW	0x04
	BANKSEL	_rs232_proto
	MOVWF	_rs232_proto, B
_00250_DS_:
;	.line	436; meter_logger.c	INTCONbits.INT0IF = 0;	/* Clear Interrupt Flag */
	BCF	_INTCONbits, 1
_00252_DS_:
;	.line	440; meter_logger.c	if (INTCONbits.TMR0IF && INTCONbits.TMR0IE) {
	BTFSS	_INTCONbits, 2
	BRA	_00305_DS_
	BTFSS	_INTCONbits, 5
	BRA	_00305_DS_
	BANKSEL	(_timer0_reload + 1)
;	.line	442; meter_logger.c	TMR0H = (unsigned char)(timer0_reload >> 8);
	MOVF	(_timer0_reload + 1), W, B
	MOVWF	r0x00
	CLRF	r0x01
	MOVF	r0x00, W
	MOVWF	_TMR0H
; removed redundant BANKSEL
;	.line	443; meter_logger.c	TMR0L = (unsigned char)timer0_reload;
	MOVF	_timer0_reload, W, B
	MOVWF	_TMR0L
;	.line	445; meter_logger.c	switch (codec_type) {
	MOVLW	0x01
	BANKSEL	_codec_type
	SUBWF	_codec_type, W, B
	BTFSS	STATUS, 0
	BRA	_00303_DS_
	MOVLW	0x06
; removed redundant BANKSEL
	SUBWF	_codec_type, W, B
	BTFSC	STATUS, 0
	BRA	_00303_DS_
; removed redundant BANKSEL
	DECF	_codec_type, W, B
	MOVWF	r0x00
	CLRF	PCLATH
	CLRF	PCLATU
	RLCF	r0x00, W
	RLCF	PCLATH, F
	RLCF	WREG, W
	RLCF	PCLATH, F
	ANDLW	0xfc
	ADDLW	LOW(_00526_DS_)
	MOVWF	POSTDEC1
	MOVLW	HIGH(_00526_DS_)
	ADDWFC	PCLATH, F
	MOVLW	UPPER(_00526_DS_)
	ADDWFC	PCLATU, F
	MOVF	PREINC1, W
	MOVWF	PCL
_00526_DS_:
	GOTO	_00254_DS_
	GOTO	_00266_DS_
	GOTO	_00255_DS_
	GOTO	_00276_DS_
	GOTO	_00287_DS_
_00254_DS_:
;	.line	447; meter_logger.c	T0CONbits.TMR0ON = 0;			// Stop TMR0
	BCF	_T0CONbits, 7
	BANKSEL	_testo_ir_proto
;	.line	448; meter_logger.c	testo_ir_proto.state = INIT_STATE;
	CLRF	_testo_ir_proto, B
	sleep 
;	.line	450; meter_logger.c	break;
	BRA	_00303_DS_
_00255_DS_:
;	.line	452; meter_logger.c	switch (rs232_proto.state) {
	MOVFF	_rs232_proto, r0x00
	MOVF	r0x00, W
	MOVWF	r0x01
	MOVF	r0x01, W
	BZ	_00256_DS_
	MOVF	r0x00, W
	XORLW	0x03
	BZ	_00259_DS_
	MOVF	r0x00, W
	XORLW	0x09
	BZ	_00263_DS_
	MOVF	r0x00, W
	XORLW	0x0a
	BZ	_00264_DS_
	BRA	_00303_DS_
_00256_DS_:
	BANKSEL	(_rs232_proto + 3)
;	.line	454; meter_logger.c	if (rs232_proto.data_len == 8) {
	MOVF	(_rs232_proto + 3), W, B
	XORLW	0x08
	BZ	_00536_DS_
	BRA	_00303_DS_
_00536_DS_:
;	.line	455; meter_logger.c	IR_LED_PIN = 1;		// inverted rs232 output on ir, start bit = ir light
	BSF	_PORTBbits, 1
;	.line	456; meter_logger.c	rs232_proto.state = START_BIT_SENT;
	MOVLW	0x03
	BANKSEL	_rs232_proto
	MOVWF	_rs232_proto, B
;	.line	458; meter_logger.c	break;
	BRA	_00303_DS_
_00259_DS_:
;	.line	460; meter_logger.c	if (rs232_proto.data_len >= 1) {
	MOVLW	0x01
	BANKSEL	(_rs232_proto + 3)
	SUBWF	(_rs232_proto + 3), W, B
	BNC	_00261_DS_
;	.line	461; meter_logger.c	IR_LED_PIN = (rs232_proto.data & 1) == 0;	// inverted rs232 output on ir
	MOVLW	0x01
; removed redundant BANKSEL
	ANDWF	(_rs232_proto + 2), W, B
	MOVWF	r0x00
	MOVF	r0x00, W
	BSF	STATUS, 0
	TSTFSZ	WREG
	BCF	STATUS, 0
	CLRF	r0x00
	RLCF	r0x00, F
	MOVF	r0x00, W
	ANDLW	0x01
	RLNCF	WREG, W
	MOVWF	PRODH
	MOVF	_PORTBbits, W
	ANDLW	0xfd
	IORWF	PRODH, W
	MOVWF	_PORTBbits
; removed redundant BANKSEL
;	.line	462; meter_logger.c	rs232_proto.data = rs232_proto.data >> 1;
	RRNCF	(_rs232_proto + 2), W, B
	ANDLW	0x7f
	MOVWF	r0x00
	MOVF	r0x00, W
; removed redundant BANKSEL
	MOVWF	(_rs232_proto + 2), B
;	.line	463; meter_logger.c	rs232_proto.data_len--;
	MOVFF	(_rs232_proto + 3), r0x00
	DECF	r0x00, F
	MOVF	r0x00, W
; removed redundant BANKSEL
	MOVWF	(_rs232_proto + 3), B
	BRA	_00303_DS_
_00261_DS_:
;	.line	466; meter_logger.c	IR_LED_PIN = 0;								// inverted rs232 output on ir					
	BCF	_PORTBbits, 1
;	.line	467; meter_logger.c	rs232_proto.state = STOP_BIT_SENT;
	MOVLW	0x09
	BANKSEL	_rs232_proto
	MOVWF	_rs232_proto, B
;	.line	469; meter_logger.c	break;
	BRA	_00303_DS_
_00263_DS_:
;	.line	471; meter_logger.c	IR_LED_PIN = 0;									// inverted rs232 output on ir
	BCF	_PORTBbits, 1
;	.line	472; meter_logger.c	rs232_proto.state = STOP_BIT2_SENT;
	MOVLW	0x0a
	BANKSEL	_rs232_proto
	MOVWF	_rs232_proto, B
;	.line	473; meter_logger.c	break;
	BRA	_00303_DS_
_00264_DS_:
;	.line	475; meter_logger.c	IR_LED_PIN = 0;									// inverted rs232 output on ir
	BCF	_PORTBbits, 1
	BANKSEL	_rs232_proto
;	.line	476; meter_logger.c	rs232_proto.state = INIT_STATE;
	CLRF	_rs232_proto, B
;	.line	477; meter_logger.c	T0CONbits.TMR0ON = 0;							// stop timer 0
	BCF	_T0CONbits, 7
;	.line	480; meter_logger.c	break;
	BRA	_00303_DS_
_00266_DS_:
;	.line	482; meter_logger.c	switch (rs232_proto.state) {
	MOVFF	_rs232_proto, r0x00
	MOVF	r0x00, W
	XORLW	0x04
	BZ	_00267_DS_
	MOVF	r0x00, W
	XORLW	0x07
	BZ	_00274_DS_
	BRA	_00303_DS_
_00267_DS_:
;	.line	484; meter_logger.c	rs232_proto.data_len++;
	MOVFF	(_rs232_proto + 3), r0x00
	INCF	r0x00, F
	MOVF	r0x00, W
	BANKSEL	(_rs232_proto + 3)
	MOVWF	(_rs232_proto + 3), B
;	.line	485; meter_logger.c	if (rs232_proto.data_len <= 8) {
	MOVLW	0x09
; removed redundant BANKSEL
	SUBWF	(_rs232_proto + 3), W, B
	BC	_00272_DS_
;	.line	486; meter_logger.c	if (IR_PIN) {		
	BTFSS	_PORTBbits, 0
	BRA	_00269_DS_
; removed redundant BANKSEL
;	.line	488; meter_logger.c	rs232_proto.data >>= 1;
	RRNCF	(_rs232_proto + 2), W, B
	ANDLW	0x7f
	MOVWF	r0x00
	MOVF	r0x00, W
; removed redundant BANKSEL
	MOVWF	(_rs232_proto + 2), B
	BRA	_00303_DS_
_00269_DS_:
	BANKSEL	(_rs232_proto + 2)
;	.line	500; meter_logger.c	rs232_proto.data >>= 1;
	RRNCF	(_rs232_proto + 2), W, B
	ANDLW	0x7f
	MOVWF	r0x00
	MOVF	r0x00, W
; removed redundant BANKSEL
	MOVWF	(_rs232_proto + 2), B
;	.line	501; meter_logger.c	rs232_proto.data |= 0x80;
	MOVLW	0x80
; removed redundant BANKSEL
	IORWF	(_rs232_proto + 2), W, B
	MOVWF	r0x00
	MOVF	r0x00, W
; removed redundant BANKSEL
	MOVWF	(_rs232_proto + 2), B
	BRA	_00303_DS_
_00272_DS_:
;	.line	523; meter_logger.c	rs232_proto.state = STOP_BIT_WAIT;
	MOVLW	0x07
	BANKSEL	_rs232_proto
	MOVWF	_rs232_proto, B
;	.line	525; meter_logger.c	break;
	BRA	_00303_DS_
_00274_DS_:
	BANKSEL	(_rs232_proto + 2)
;	.line	531; meter_logger.c	fifo_put(rs232_proto.data);
	MOVF	(_rs232_proto + 2), W, B
	MOVWF	POSTDEC1
	CALL	_fifo_put
	MOVF	POSTINC1, F
	BANKSEL	(_rs232_proto + 2)
;	.line	532; meter_logger.c	rs232_proto.data = 0;
	CLRF	(_rs232_proto + 2), B
; removed redundant BANKSEL
;	.line	533; meter_logger.c	rs232_proto.data_len = 0;
	CLRF	(_rs232_proto + 3), B
;	.line	534; meter_logger.c	rs232_proto.state = START_BIT_WAIT;
	MOVLW	0x02
; removed redundant BANKSEL
	MOVWF	_rs232_proto, B
;	.line	535; meter_logger.c	T0CONbits.TMR0ON = 0;
	BCF	_T0CONbits, 7
;	.line	536; meter_logger.c	INTCONbits.INT0IF = 0;		// dont enter ext int now
	BCF	_INTCONbits, 1
;	.line	537; meter_logger.c	INTCONbits.INT0IE = 1;		// enable ext int again
	BSF	_INTCONbits, 4
;	.line	540; meter_logger.c	break;
	BRA	_00303_DS_
_00276_DS_:
;	.line	542; meter_logger.c	switch (fsk_proto.state) {
	MOVFF	_fsk_proto, r0x00
	MOVF	r0x00, W
	XORLW	0x04
	BZ	_00277_DS_
	MOVF	r0x00, W
	XORLW	0x07
	BZ	_00285_DS_
	BRA	_00303_DS_
_00277_DS_:
;	.line	544; meter_logger.c	fsk_proto.data_len++;						
	MOVFF	(_fsk_proto + 13), r0x00
	INCF	r0x00, F
	MOVF	r0x00, W
	BANKSEL	(_fsk_proto + 13)
	MOVWF	(_fsk_proto + 13), B
;	.line	545; meter_logger.c	if (fsk_proto.data_len <= 8) {
	MOVLW	0x09
; removed redundant BANKSEL
	SUBWF	(_fsk_proto + 13), W, B
	BC	_00283_DS_
;	.line	546; meter_logger.c	if ((fsk_proto.diff > 340) && (fsk_proto.diff < 476)) {
	MOVLW	0x01
; removed redundant BANKSEL
	SUBWF	(_fsk_proto + 2), W, B
	BNZ	_00550_DS_
	MOVLW	0x55
; removed redundant BANKSEL
	SUBWF	(_fsk_proto + 1), W, B
_00550_DS_:
	BNC	_00279_DS_
	MOVLW	0x01
	BANKSEL	(_fsk_proto + 2)
	SUBWF	(_fsk_proto + 2), W, B
	BNZ	_00551_DS_
	MOVLW	0xdc
; removed redundant BANKSEL
	SUBWF	(_fsk_proto + 1), W, B
_00551_DS_:
	BC	_00279_DS_
	BANKSEL	(_fsk_proto + 12)
;	.line	549; meter_logger.c	fsk_proto.data >>= 1;
	RRNCF	(_fsk_proto + 12), W, B
	ANDLW	0x7f
	MOVWF	r0x00
	MOVF	r0x00, W
; removed redundant BANKSEL
	MOVWF	(_fsk_proto + 12), B
	BRA	_00303_DS_
_00279_DS_:
	BANKSEL	(_fsk_proto + 12)
;	.line	554; meter_logger.c	fsk_proto.data >>= 1;
	RRNCF	(_fsk_proto + 12), W, B
	ANDLW	0x7f
	MOVWF	r0x00
	MOVF	r0x00, W
; removed redundant BANKSEL
	MOVWF	(_fsk_proto + 12), B
;	.line	555; meter_logger.c	fsk_proto.data |= 0x80;
	MOVLW	0x80
; removed redundant BANKSEL
	IORWF	(_fsk_proto + 12), W, B
	MOVWF	r0x00
	MOVF	r0x00, W
; removed redundant BANKSEL
	MOVWF	(_fsk_proto + 12), B
	BRA	_00303_DS_
_00283_DS_:
;	.line	563; meter_logger.c	fsk_proto.state = STOP_BIT_WAIT;
	MOVLW	0x07
	BANKSEL	_fsk_proto
	MOVWF	_fsk_proto, B
;	.line	565; meter_logger.c	break;
	BRA	_00303_DS_
_00285_DS_:
	BANKSEL	(_fsk_proto + 12)
;	.line	568; meter_logger.c	fifo_put(fsk_proto.data);
	MOVF	(_fsk_proto + 12), W, B
	MOVWF	POSTDEC1
	CALL	_fifo_put
	MOVF	POSTINC1, F
	BANKSEL	(_fsk_proto + 12)
;	.line	569; meter_logger.c	fsk_proto.data = 0;
	CLRF	(_fsk_proto + 12), B
;	.line	570; meter_logger.c	fsk_proto.state = START_BIT_WAIT;
	MOVLW	0x02
; removed redundant BANKSEL
	MOVWF	_fsk_proto, B
;	.line	572; meter_logger.c	INTCONbits.TMR0IE = 0;						
	BCF	_INTCONbits, 5
;	.line	575; meter_logger.c	break;
	BRA	_00303_DS_
_00287_DS_:
;	.line	577; meter_logger.c	switch (fsk_proto.state) {
	MOVFF	_fsk_proto, r0x00
	MOVF	r0x00, W
	MOVWF	r0x01
	MOVF	r0x01, W
	BZ	_00288_DS_
	MOVF	r0x00, W
	XORLW	0x01
	BZ	_00291_DS_
	MOVF	r0x00, W
	XORLW	0x03
	BZ	_00292_DS_
	MOVF	r0x00, W
	XORLW	0x05
	BNZ	_00560_DS_
	BRA	_00300_DS_
_00560_DS_:
	MOVF	r0x00, W
	XORLW	0x09
	BNZ	_00562_DS_
	BRA	_00301_DS_
_00562_DS_:
	BRA	_00303_DS_
_00288_DS_:
	BANKSEL	(_fsk_proto + 13)
;	.line	580; meter_logger.c	if (fsk_proto.data_len == 8) {
	MOVF	(_fsk_proto + 13), W, B
	XORLW	0x08
	BNZ	_00290_DS_
;	.line	581; meter_logger.c	fsk_proto.state = IDLE;
	MOVLW	0x01
	BANKSEL	_fsk_proto
	MOVWF	_fsk_proto, B
_00290_DS_:
;	.line	584; meter_logger.c	DEBUG_PIN = 0;
	BCF	_PORTBbits, 2
;	.line	586; meter_logger.c	break;
	BRA	_00303_DS_
_00291_DS_:
;	.line	588; meter_logger.c	send_fsk_low();
	CALL	_send_fsk_low
;	.line	589; meter_logger.c	fsk_proto.state = START_BIT_SENT;
	MOVLW	0x03
	BANKSEL	_fsk_proto
	MOVWF	_fsk_proto, B
;	.line	591; meter_logger.c	DEBUG_PIN = 1;
	BSF	_PORTBbits, 2
;	.line	593; meter_logger.c	break;
	BRA	_00303_DS_
_00292_DS_:
;	.line	595; meter_logger.c	if (fsk_proto.data_len--) {
	MOVFF	(_fsk_proto + 13), r0x00
	DECF	r0x00, W
	MOVWF	r0x01
	MOVF	r0x01, W
	BANKSEL	(_fsk_proto + 13)
	MOVWF	(_fsk_proto + 13), B
	MOVF	r0x00, W
	BZ	_00297_DS_
;	.line	596; meter_logger.c	if (fsk_proto.data & (0x80 >> fsk_proto.data_len)) {
	MOVLW	0x80
	MOVWF	r0x00
; removed redundant BANKSEL
	MOVF	(_fsk_proto + 13), W, B
	BZ	_00565_DS_
	NEGF	WREG
	BCF	STATUS, 0
_00566_DS_:
	RRCF	r0x00, F
	ADDLW	0x01
	BNC	_00566_DS_
_00565_DS_:
	BANKSEL	(_fsk_proto + 12)
	MOVF	(_fsk_proto + 12), W, B
	ANDWF	r0x00, F
	MOVF	r0x00, W
	BZ	_00294_DS_
;	.line	597; meter_logger.c	send_fsk_high();
	CALL	_send_fsk_high
;	.line	599; meter_logger.c	DEBUG_PIN = 0;
	BCF	_PORTBbits, 2
	BRA	_00297_DS_
_00294_DS_:
;	.line	603; meter_logger.c	send_fsk_low();
	CALL	_send_fsk_low
;	.line	605; meter_logger.c	DEBUG_PIN = 1;
	BSF	_PORTBbits, 2
_00297_DS_:
	BANKSEL	(_fsk_proto + 13)
;	.line	609; meter_logger.c	if (fsk_proto.data_len == 0) {
	MOVF	(_fsk_proto + 13), W, B
	BNZ	_00303_DS_
;	.line	610; meter_logger.c	fsk_proto.state = DATA_SENT;
	MOVLW	0x05
; removed redundant BANKSEL
	MOVWF	_fsk_proto, B
;	.line	612; meter_logger.c	break;
	BRA	_00303_DS_
_00300_DS_:
;	.line	614; meter_logger.c	send_fsk_high();
	CALL	_send_fsk_high
;	.line	615; meter_logger.c	fsk_proto.state = STOP_BIT_SENT;
	MOVLW	0x09
	BANKSEL	_fsk_proto
	MOVWF	_fsk_proto, B
;	.line	617; meter_logger.c	DEBUG_PIN = 0;
	BCF	_PORTBbits, 2
;	.line	619; meter_logger.c	break;
	BRA	_00303_DS_
_00301_DS_:
;	.line	621; meter_logger.c	send_fsk_high();
	CALL	_send_fsk_high
	BANKSEL	_fsk_proto
;	.line	622; meter_logger.c	fsk_proto.state = INIT_STATE;
	CLRF	_fsk_proto, B
;	.line	624; meter_logger.c	DEBUG_PIN = 0;
	BCF	_PORTBbits, 2
_00303_DS_:
;	.line	631; meter_logger.c	INTCONbits.TMR0IF = 0;
	BCF	_INTCONbits, 2
_00305_DS_:
;	.line	634; meter_logger.c	if (PIR2bits.CMIF && PIE2bits.CMIE) {
	BTFSS	_PIR2bits, 6
	BRA	_00324_DS_
	BTFSS	_PIE2bits, 6
	BRA	_00324_DS_
;	.line	636; meter_logger.c	if (CMCONbits.C1OUT) {		// rising edge
	BTFSS	_CMCONbits, 6
	BRA	_00319_DS_
;	.line	637; meter_logger.c	timer_0 = (unsigned int)(TMR0L) | ((unsigned int)(TMR0H) << 8);
	MOVFF	_TMR0L, r0x00
	CLRF	r0x01
	MOVFF	_TMR0H, r0x02
	CLRF	r0x03
	MOVF	r0x02, W
	MOVWF	r0x05
	CLRF	r0x04
	MOVF	r0x04, W
	IORWF	r0x00, W
	BANKSEL	_timer_0
	MOVWF	_timer_0, B
	MOVF	r0x05, W
	IORWF	r0x01, W
; removed redundant BANKSEL
	MOVWF	(_timer_0 + 1), B
;	.line	642; meter_logger.c	DEBUG_PIN = 1;
	BSF	_PORTBbits, 2
	BANKSEL	_last_timer_0
;	.line	644; meter_logger.c	fsk_proto.diff = timer_0 - last_timer_0;
	MOVF	_last_timer_0, W, B
	BANKSEL	_timer_0
	SUBWF	_timer_0, W, B
	MOVWF	r0x00
	BANKSEL	(_last_timer_0 + 1)
	MOVF	(_last_timer_0 + 1), W, B
	BANKSEL	(_timer_0 + 1)
	SUBWFB	(_timer_0 + 1), W, B
	MOVWF	r0x01
	MOVF	r0x00, W
	BANKSEL	(_fsk_proto + 1)
	MOVWF	(_fsk_proto + 1), B
	MOVF	r0x01, W
; removed redundant BANKSEL
	MOVWF	(_fsk_proto + 2), B
;	.line	645; meter_logger.c	last_timer_0 = timer_0;
	MOVFF	_timer_0, _last_timer_0
	MOVFF	(_timer_0 + 1), (_last_timer_0 + 1)
;	.line	647; meter_logger.c	if ((fsk_proto.diff > 340) && (fsk_proto.diff < 476)) {
	MOVLW	0x01
; removed redundant BANKSEL
	SUBWF	(_fsk_proto + 2), W, B
	BNZ	_00568_DS_
	MOVLW	0x55
; removed redundant BANKSEL
	SUBWF	(_fsk_proto + 1), W, B
_00568_DS_:
	BNC	_00315_DS_
	MOVLW	0x01
	BANKSEL	(_fsk_proto + 2)
	SUBWF	(_fsk_proto + 2), W, B
	BNZ	_00569_DS_
	MOVLW	0xdc
; removed redundant BANKSEL
	SUBWF	(_fsk_proto + 1), W, B
_00569_DS_:
	BC	_00315_DS_
	BANKSEL	(_fsk_proto + 1)
;	.line	648; meter_logger.c	fsk_proto.low_count += fsk_proto.diff;
	MOVF	(_fsk_proto + 1), W, B
; removed redundant BANKSEL
	ADDWF	(_fsk_proto + 5), W, B
	MOVWF	r0x00
; removed redundant BANKSEL
	MOVF	(_fsk_proto + 2), W, B
; removed redundant BANKSEL
	ADDWFC	(_fsk_proto + 6), W, B
	MOVWF	r0x01
	MOVF	r0x00, W
; removed redundant BANKSEL
	MOVWF	(_fsk_proto + 5), B
	MOVF	r0x01, W
; removed redundant BANKSEL
	MOVWF	(_fsk_proto + 6), B
; removed redundant BANKSEL
;	.line	649; meter_logger.c	if (fsk_proto.state == START_BIT_WAIT) {
	MOVF	_fsk_proto, W, B
	XORLW	0x02
	BNZ	_00320_DS_
;	.line	650; meter_logger.c	if (fsk_proto.low_count >= 800) {								// start bit received
	MOVLW	0x03
	BANKSEL	(_fsk_proto + 6)
	SUBWF	(_fsk_proto + 6), W, B
	BNZ	_00572_DS_
	MOVLW	0x20
; removed redundant BANKSEL
	SUBWF	(_fsk_proto + 5), W, B
_00572_DS_:
	BNC	_00320_DS_
	BANKSEL	(_timer0_reload + 1)
;	.line	652; meter_logger.c	TMR0H = (unsigned char)(timer0_reload >> 8);
	MOVF	(_timer0_reload + 1), W, B
	MOVWF	r0x00
	CLRF	r0x01
	MOVF	r0x00, W
	MOVWF	_TMR0H
; removed redundant BANKSEL
;	.line	653; meter_logger.c	TMR0L = (unsigned char)timer0_reload;
	MOVF	_timer0_reload, W, B
	MOVWF	_TMR0L
	BANKSEL	(_fsk_proto + 5)
;	.line	654; meter_logger.c	fsk_proto.low_count = 0;
	CLRF	(_fsk_proto + 5), B
; removed redundant BANKSEL
	CLRF	(_fsk_proto + 6), B
; removed redundant BANKSEL
;	.line	655; meter_logger.c	fsk_proto.high_count = 0;
	CLRF	(_fsk_proto + 7), B
; removed redundant BANKSEL
	CLRF	(_fsk_proto + 8), B
; removed redundant BANKSEL
;	.line	657; meter_logger.c	fsk_proto.data_len = 0;
	CLRF	(_fsk_proto + 13), B
; removed redundant BANKSEL
;	.line	658; meter_logger.c	fsk_proto.data = 0;
	CLRF	(_fsk_proto + 12), B
;	.line	659; meter_logger.c	fsk_proto.state = DATA_WAIT;
	MOVLW	0x04
; removed redundant BANKSEL
	MOVWF	_fsk_proto, B
;	.line	660; meter_logger.c	INTCONbits.TMR0IF = 0;		// clear flag so it dont enter isr now
	BCF	_INTCONbits, 2
;	.line	661; meter_logger.c	INTCONbits.TMR0IE = 1;		// Enable TMR0 Interrupt
	BSF	_INTCONbits, 5
	BRA	_00320_DS_
_00315_DS_:
	BANKSEL	_fsk_proto
;	.line	667; meter_logger.c	if (fsk_proto.state == START_BIT_WAIT) {
	MOVF	_fsk_proto, W, B
	XORLW	0x02
	BNZ	_00312_DS_
_00574_DS_:
	BANKSEL	(_fsk_proto + 5)
;	.line	668; meter_logger.c	fsk_proto.low_count = 0;
	CLRF	(_fsk_proto + 5), B
; removed redundant BANKSEL
	CLRF	(_fsk_proto + 6), B
; removed redundant BANKSEL
;	.line	669; meter_logger.c	fsk_proto.high_count = 0;
	CLRF	(_fsk_proto + 7), B
; removed redundant BANKSEL
	CLRF	(_fsk_proto + 8), B
	BRA	_00320_DS_
_00312_DS_:
	BANKSEL	(_fsk_proto + 1)
;	.line	672; meter_logger.c	fsk_proto.high_count += fsk_proto.diff;
	MOVF	(_fsk_proto + 1), W, B
; removed redundant BANKSEL
	ADDWF	(_fsk_proto + 7), W, B
	MOVWF	r0x00
; removed redundant BANKSEL
	MOVF	(_fsk_proto + 2), W, B
; removed redundant BANKSEL
	ADDWFC	(_fsk_proto + 8), W, B
	MOVWF	r0x01
	MOVF	r0x00, W
; removed redundant BANKSEL
	MOVWF	(_fsk_proto + 7), B
	MOVF	r0x01, W
; removed redundant BANKSEL
	MOVWF	(_fsk_proto + 8), B
	BRA	_00320_DS_
_00319_DS_:
;	.line	678; meter_logger.c	DEBUG_PIN = 0;
	BCF	_PORTBbits, 2
_00320_DS_:
;	.line	682; meter_logger.c	PIR2bits.CMIF = 0;
	BCF	_PIR2bits, 6
_00324_DS_:
	MOVFF	PREINC1, r0x05
	MOVFF	PREINC1, r0x04
	MOVFF	PREINC1, r0x03
	MOVFF	PREINC1, r0x02
	MOVFF	PREINC1, r0x01
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	MOVFF	PREINC1, PCLATU
	MOVFF	PREINC1, PCLATH
	MOVFF	PREINC1, FSR0H
	MOVFF	PREINC1, FSR0L
	MOVFF	PREINC1, PRODH
	MOVFF	PREINC1, PRODL
	MOVF	PREINC1, W
	MOVFF	PREINC1, BSR
	MOVFF	PREINC1, STATUS
	RETFIE	

; ; Starting pCode block
___str_0:
	DB	0x0a, 0x0d, 0x4d, 0x65, 0x74, 0x65, 0x72, 0x4c, 0x6f, 0x67, 0x67, 0x65
	DB	0x72, 0x2e, 0x2e, 0x2e, 0x20, 0x73, 0x65, 0x72, 0x69, 0x61, 0x6c, 0x20
	DB	0x77, 0x6f, 0x72, 0x6b, 0x69, 0x6e, 0x67, 0x0a, 0x0d, 0x00
; ; Starting pCode block
___str_1:
	DB	0x0a, 0x0d, 0x70, 0x72, 0x65, 0x73, 0x73, 0x20, 0x70, 0x72, 0x69, 0x6e
	DB	0x74, 0x20, 0x6f, 0x6e, 0x20, 0x74, 0x65, 0x73, 0x74, 0x6f, 0x0a, 0x0d
	DB	0x00
; ; Starting pCode block
___str_2:
	DB	0x0a, 0x0d, 0x64, 0x6f, 0x6e, 0x65, 0x20, 0x72, 0x65, 0x63, 0x65, 0x69
	DB	0x76, 0x69, 0x6e, 0x67, 0x20, 0x2d, 0x20, 0x73, 0x65, 0x6e, 0x64, 0x69
	DB	0x6e, 0x67, 0x20, 0x76, 0x69, 0x61, 0x20, 0x73, 0x65, 0x72, 0x69, 0x61
	DB	0x6c, 0x2f, 0x66, 0x73, 0x6b, 0x0a, 0x0d, 0x00
; ; Starting pCode block
___str_3:
	DB	0x0a, 0x0d, 0x77, 0x61, 0x69, 0x74, 0x69, 0x6e, 0x67, 0x20, 0x66, 0x6f
	DB	0x72, 0x20, 0x6e, 0x65, 0x77, 0x20, 0x63, 0x6f, 0x6d, 0x6d, 0x61, 0x6e
	DB	0x64, 0x0a, 0x0d, 0x00
; ; Starting pCode block
___str_4:
	DB	0x0a, 0x0d, 0x65, 0x63, 0x68, 0x6f, 0x20, 0x74, 0x65, 0x73, 0x74, 0x20
	DB	0x2d, 0x20, 0x73, 0x65, 0x6e, 0x64, 0x20, 0x73, 0x6f, 0x6d, 0x65, 0x20
	DB	0x64, 0x61, 0x74, 0x61, 0x0a, 0x0d, 0x00


; Statistics:
; code size:	12420 (0x3084) bytes ( 9.48%)
;           	 6210 (0x1842) words
; udata size:	 1200 (0x04b0) bytes (66.96%)
; access size:	   16 (0x0010) bytes


	end
