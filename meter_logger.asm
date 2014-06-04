;--------------------------------------------------------
; File Created by SDCC : free open source ANSI-C Compiler
; Version 3.4.0 #8981 (Jun  3 2014) (Mac OS X x86_64)
; This file was generated Tue Jun  3 23:31:55 2014
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
	global	_buffer
	global	_sleep_ms
	global	_init_system
	global	_my_usart_open
	global	_reverse
	global	_valid_err_corr
	global	_testo_ir_enable
	global	_testo_ir_disable
	global	_rs232_tx_enable
	global	_rs232_tx_disable
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
	global	_c
	global	_fifo_head
	global	_fifo_tail
	global	_fifo_buffer_0
	global	_fifo_buffer_1
	global	_fifo_buffer_2
	global	_fifo_buffer_3
	global	_codec_type
	global	_led_flash
	global	_testo_ir_proto
	global	_rs232_ir_proto
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
_buffer	res	64

udata_meter_logger_1	udata
_timer_1_ms	res	4

udata_meter_logger_2	udata
_fifo_head	res	2

udata_meter_logger_3	udata
_fifo_tail	res	2

udata_meter_logger_4	udata
_main_cmd_1_78	res	1

udata_meter_logger_5	udata
_main_sub_cmd_1_78	res	1

udata_meter_logger_6	udata
_timer_0	res	2

udata_meter_logger_7	udata
_timer0_reload	res	2

udata_meter_logger_8	udata
_testo_ir_proto	res	6

udata_meter_logger_9	udata
_rs232_ir_proto	res	6

udata_meter_logger_10	udata
_fsk_proto	res	16

udata_meter_logger_11	udata
_last_timer_0	res	2

udata_meter_logger_12	udata
_codec_type	res	1

udata_meter_logger_13	udata
_c	res	1

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
;	.line	105; meter_logger.c	OSCCONbits.SCS = 0x10;
	MOVF	_OSCCONbits, W
	ANDLW	0xfc
	MOVWF	_OSCCONbits
;	.line	107; meter_logger.c	OSCCONbits.IRCF = 0x7;	// 8 MHz
	MOVF	_OSCCONbits, W
	ANDLW	0x8f
	IORLW	0x70
	MOVWF	_OSCCONbits
	BANKSEL	_timer_1_ms
;	.line	110; meter_logger.c	timer_1_ms = 0;
	CLRF	_timer_1_ms, B
; removed redundant BANKSEL
	CLRF	(_timer_1_ms + 1), B
; removed redundant BANKSEL
	CLRF	(_timer_1_ms + 2), B
; removed redundant BANKSEL
	CLRF	(_timer_1_ms + 3), B
	BANKSEL	_fifo_head
;	.line	112; meter_logger.c	fifo_head = 0;
	CLRF	_fifo_head, B
; removed redundant BANKSEL
	CLRF	(_fifo_head + 1), B
	BANKSEL	_fifo_tail
;	.line	113; meter_logger.c	fifo_tail = 0;
	CLRF	_fifo_tail, B
; removed redundant BANKSEL
	CLRF	(_fifo_tail + 1), B
;	.line	115; meter_logger.c	init_system();
	CALL	_init_system
;	.line	122; meter_logger.c	fsk_rx_enable();
	CALL	_fsk_rx_enable
_00130_DS_:
;	.line	124; meter_logger.c	if (fifo_get(&cmd)) {
	MOVLW	HIGH(_main_cmd_1_78)
	MOVWF	r0x01
	MOVLW	LOW(_main_cmd_1_78)
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
	BZ	_00130_DS_
	BANKSEL	_main_cmd_1_78
;	.line	125; meter_logger.c	switch (cmd) {
	MOVF	_main_cmd_1_78, W, B
	BZ	_00105_DS_
_00186_DS_:
	BANKSEL	_main_cmd_1_78
	MOVF	_main_cmd_1_78, W, B
	XORLW	0x01
	BNZ	_00188_DS_
	BRA	_00119_DS_
_00188_DS_:
	BANKSEL	_main_cmd_1_78
	MOVF	_main_cmd_1_78, W, B
	XORLW	0xff
	BNZ	_00190_DS_
	BRA	_00112_DS_
_00190_DS_:
	BRA	_00130_DS_
_00105_DS_:
;	.line	127; meter_logger.c	fsk_rx_disable();
	CALL	_fsk_rx_disable
;	.line	128; meter_logger.c	usart_puts("press print on testo\n");
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
;	.line	129; meter_logger.c	testo_ir_enable();
	CALL	_testo_ir_enable
;	.line	131; meter_logger.c	last_fifo_size = 0;
	CLRF	r0x00
	CLRF	r0x01
;	.line	132; meter_logger.c	sleep_ms(10000);						// 10 seconds to start printing
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
;	.line	133; meter_logger.c	fifo_size = fifo_in_use();
	CALL	_fifo_in_use
	MOVWF	r0x02
	MOVFF	PRODL, r0x03
_00106_DS_:
;	.line	134; meter_logger.c	while (fifo_size > last_fifo_size) {	// and wait while we are still receiving data
	MOVF	r0x03, W
	SUBWF	r0x01, W
	BNZ	_00191_DS_
	MOVF	r0x02, W
	SUBWF	r0x00, W
_00191_DS_:
	BC	_00108_DS_
;	.line	135; meter_logger.c	last_fifo_size = fifo_size;
	MOVFF	r0x02, r0x00
	MOVFF	r0x03, r0x01
;	.line	136; meter_logger.c	sleep_ms(200);						// return data when no data for 200 ms
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
;	.line	137; meter_logger.c	fifo_size = fifo_in_use();
	CALL	_fifo_in_use
	MOVWF	r0x02
	MOVFF	PRODL, r0x03
	BRA	_00106_DS_
_00108_DS_:
;	.line	139; meter_logger.c	testo_ir_disable();
	CALL	_testo_ir_disable
;	.line	140; meter_logger.c	usart_puts("done receiving - sending via serial/fsk\n");
	MOVLW	UPPER(___str_1)
	MOVWF	r0x06
	MOVLW	HIGH(___str_1)
	MOVWF	r0x05
	MOVLW	LOW(___str_1)
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
;	.line	142; meter_logger.c	fsk_tx_enable();
	CALL	_fsk_tx_enable
_00109_DS_:
;	.line	144; meter_logger.c	while (fifo_get(&cmd)) {	// and print them to serial
	MOVLW	HIGH(_main_cmd_1_78)
	MOVWF	r0x05
	MOVLW	LOW(_main_cmd_1_78)
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
	BANKSEL	_main_cmd_1_78
;	.line	151; meter_logger.c	fsk_tx_byte(cmd);
	MOVF	_main_cmd_1_78, W, B
	MOVWF	POSTDEC1
	CALL	_fsk_tx_byte
	MOVF	POSTINC1, F
;	.line	152; meter_logger.c	sleep_ms(FSK_TX_SLEEP_AFTER);
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x04
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVLW	0x04
	ADDWF	FSR1L, F
	BRA	_00109_DS_
_00111_DS_:
;	.line	156; meter_logger.c	fsk_tx_disable();
	CALL	_fsk_tx_disable
;	.line	158; meter_logger.c	usart_puts("waiting for new command\n");
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
;	.line	159; meter_logger.c	fsk_rx_enable();
	CALL	_fsk_rx_enable
;	.line	160; meter_logger.c	break;
	BRA	_00130_DS_
_00112_DS_:
;	.line	163; meter_logger.c	fsk_rx_disable();
	CALL	_fsk_rx_disable
;	.line	164; meter_logger.c	usart_puts("echo test - send some data\n");
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
	BNZ	_00192_DS_
	MOVF	r0x02, W
	SUBWF	r0x00, W
_00192_DS_:
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
_00116_DS_:
;	.line	179; meter_logger.c	while (fifo_get(&sub_cmd)) {
	MOVLW	HIGH(_main_sub_cmd_1_78)
	MOVWF	r0x05
	MOVLW	LOW(_main_sub_cmd_1_78)
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
	BANKSEL	_main_sub_cmd_1_78
;	.line	186; meter_logger.c	fsk_tx_byte(sub_cmd);
	MOVF	_main_sub_cmd_1_78, W, B
	MOVWF	POSTDEC1
	CALL	_fsk_tx_byte
	MOVF	POSTINC1, F
;	.line	187; meter_logger.c	sleep_ms(FSK_TX_SLEEP_AFTER);
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x04
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVLW	0x04
	ADDWF	FSR1L, F
	BRA	_00116_DS_
_00118_DS_:
;	.line	191; meter_logger.c	fsk_tx_disable();
	CALL	_fsk_tx_disable
;	.line	193; meter_logger.c	usart_puts("\nwaiting for new command\n");
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
;	.line	194; meter_logger.c	fsk_rx_enable();
	CALL	_fsk_rx_enable
;	.line	195; meter_logger.c	break;
	BRA	_00130_DS_
_00119_DS_:
;	.line	198; meter_logger.c	fsk_rx_disable();
	CALL	_fsk_rx_disable
;	.line	199; meter_logger.c	usart_puts("kamstrup - send some data\n");
	MOVLW	UPPER(___str_5)
	MOVWF	r0x06
	MOVLW	HIGH(___str_5)
	MOVWF	r0x05
	MOVLW	LOW(___str_5)
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
;	.line	200; meter_logger.c	fsk_rx_enable();
	CALL	_fsk_rx_enable
;	.line	202; meter_logger.c	last_fifo_size = 0;
	CLRF	r0x00
	CLRF	r0x01
;	.line	203; meter_logger.c	sleep_ms(1000);							// 1 second
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
;	.line	204; meter_logger.c	fifo_size = fifo_in_use();
	CALL	_fifo_in_use
	MOVWF	r0x02
	MOVFF	PRODL, r0x03
_00120_DS_:
;	.line	205; meter_logger.c	while (fifo_size > last_fifo_size) {	// and wait while we are still receiving data
	MOVF	r0x03, W
	SUBWF	r0x01, W
	BNZ	_00193_DS_
	MOVF	r0x02, W
	SUBWF	r0x00, W
_00193_DS_:
	BC	_00122_DS_
;	.line	206; meter_logger.c	last_fifo_size = fifo_size;
	MOVFF	r0x02, r0x00
	MOVFF	r0x03, r0x01
;	.line	207; meter_logger.c	sleep_ms(500);						// return data when no data for 500 ms
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
;	.line	208; meter_logger.c	fifo_size = fifo_in_use();
	CALL	_fifo_in_use
	MOVWF	r0x02
	MOVFF	PRODL, r0x03
	BRA	_00120_DS_
_00122_DS_:
;	.line	210; meter_logger.c	fsk_rx_disable();
	CALL	_fsk_rx_disable
;	.line	212; meter_logger.c	fsk_tx_enable();
	CALL	_fsk_tx_enable
_00123_DS_:
;	.line	214; meter_logger.c	while (fifo_get(&sub_cmd)) {
	MOVLW	HIGH(_main_sub_cmd_1_78)
	MOVWF	r0x01
	MOVLW	LOW(_main_sub_cmd_1_78)
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
	BZ	_00125_DS_
	BANKSEL	_main_sub_cmd_1_78
;	.line	221; meter_logger.c	fsk_tx_byte(sub_cmd);
	MOVF	_main_sub_cmd_1_78, W, B
	MOVWF	POSTDEC1
	CALL	_fsk_tx_byte
	MOVF	POSTINC1, F
;	.line	222; meter_logger.c	sleep_ms(FSK_TX_SLEEP_AFTER);
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x04
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVLW	0x04
	ADDWF	FSR1L, F
	BRA	_00123_DS_
_00125_DS_:
;	.line	226; meter_logger.c	fsk_tx_disable();
	CALL	_fsk_tx_disable
;	.line	228; meter_logger.c	usart_puts("\nwaiting for new command\n");
	MOVLW	UPPER(___str_4)
	MOVWF	r0x02
	MOVLW	HIGH(___str_4)
	MOVWF	r0x01
	MOVLW	LOW(___str_4)
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
;	.line	229; meter_logger.c	fsk_rx_enable();
	CALL	_fsk_rx_enable
;	.line	231; meter_logger.c	}
	BRA	_00130_DS_
	RETURN	

; ; Starting pCode block
S_meter_logger___debug2	code
__debug2:
;	.line	4062; meter_logger.c	void _debug2() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
;	.line	4063; meter_logger.c	DEBUG2_PIN = 0x1;
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
	
;	.line	4111; meter_logger.c	DEBUG2_PIN = 0x0;
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
S_meter_logger___debug	code
__debug:
;	.line	3962; meter_logger.c	void _debug() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
;	.line	3963; meter_logger.c	DEBUG_PIN = 0x1;
	BSF	_PORTBbits, 1
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	
;	.line	4011; meter_logger.c	DEBUG_PIN = 0x0;
	BCF	_PORTBbits, 1
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
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
;	.line	3957; meter_logger.c	void flash_led(unsigned char ms) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
;	.line	3958; meter_logger.c	led_flash.timer = ms;
	MOVF	r0x00, W
	BANKSEL	(_led_flash + 1)
	MOVWF	(_led_flash + 1), B
; removed redundant BANKSEL
;	.line	3959; meter_logger.c	led_flash.state = LED_FLASH_RUN;
	CLRF	_led_flash, B
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__fifo_snoop	code
_fifo_snoop:
;	.line	3934; meter_logger.c	unsigned char fifo_snoop(unsigned char *c, unsigned int pos) {
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
;	.line	3935; meter_logger.c	if (fifo_in_use() > (pos)) {
	CALL	_fifo_in_use
	MOVWF	r0x05
	MOVFF	PRODL, r0x06
	MOVF	r0x06, W
	SUBWF	r0x04, W
	BNZ	_00828_DS_
	MOVF	r0x05, W
	SUBWF	r0x03, W
_00828_DS_:
	BTFSC	STATUS, 0
	BRA	_00818_DS_
	BANKSEL	(_fifo_tail + 1)
;	.line	3936; meter_logger.c	switch (fifo_tail/QUEUE_SIZE) {
	MOVF	(_fifo_tail + 1), W, B
	MOVWF	r0x05
	CLRF	r0x06
	MOVLW	0x00
	SUBWF	r0x06, W
	BNZ	_00829_DS_
	MOVLW	0x04
	SUBWF	r0x05, W
_00829_DS_:
	BTFSC	STATUS, 0
	BRA	_00816_DS_
	CLRF	PCLATH
	CLRF	PCLATU
	RLCF	r0x05, W
	RLCF	PCLATH, F
	RLCF	WREG, W
	RLCF	PCLATH, F
	ANDLW	0xfc
	ADDLW	LOW(_00830_DS_)
	MOVWF	POSTDEC1
	MOVLW	HIGH(_00830_DS_)
	ADDWFC	PCLATH, F
	MOVLW	UPPER(_00830_DS_)
	ADDWFC	PCLATU, F
	MOVF	PREINC1, W
	MOVWF	PCL
_00830_DS_:
	GOTO	_00812_DS_
	GOTO	_00813_DS_
	GOTO	_00814_DS_
	GOTO	_00815_DS_
_00812_DS_:
;	.line	3938; meter_logger.c	*c = fifo_buffer_0[(fifo_tail + pos) % QUEUE_SIZE];
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
;	.line	3939; meter_logger.c	break;
	BRA	_00816_DS_
_00813_DS_:
;	.line	3941; meter_logger.c	*c = fifo_buffer_1[(fifo_tail + pos) % QUEUE_SIZE];
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
;	.line	3942; meter_logger.c	break;
	BRA	_00816_DS_
_00814_DS_:
;	.line	3944; meter_logger.c	*c = fifo_buffer_2[(fifo_tail + pos) % QUEUE_SIZE];
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
;	.line	3945; meter_logger.c	break;
	BRA	_00816_DS_
_00815_DS_:
	BANKSEL	_fifo_tail
;	.line	3947; meter_logger.c	*c = fifo_buffer_3[(fifo_tail + pos) % QUEUE_SIZE];
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
_00816_DS_:
;	.line	3950; meter_logger.c	return 1;
	MOVLW	0x01
	BRA	_00820_DS_
_00818_DS_:
;	.line	3953; meter_logger.c	return 0;
	CLRF	WREG
_00820_DS_:
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
;	.line	3906; meter_logger.c	unsigned char fifo_get(unsigned char *c) {
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
;	.line	3907; meter_logger.c	if (fifo_in_use() != 0) {
	CALL	_fifo_in_use
	MOVWF	r0x03
	MOVFF	PRODL, r0x04
	MOVF	r0x03, W
	IORWF	r0x04, W
	BTFSC	STATUS, 2
	BRA	_00787_DS_
	BANKSEL	(_fifo_tail + 1)
;	.line	3908; meter_logger.c	switch (fifo_tail/QUEUE_SIZE) {
	MOVF	(_fifo_tail + 1), W, B
	MOVWF	r0x03
	CLRF	r0x04
	MOVLW	0x00
	SUBWF	r0x04, W
	BNZ	_00800_DS_
	MOVLW	0x04
	SUBWF	r0x03, W
_00800_DS_:
	BTFSC	STATUS, 0
	BRA	_00783_DS_
	CLRF	PCLATH
	CLRF	PCLATU
	RLCF	r0x03, W
	RLCF	PCLATH, F
	RLCF	WREG, W
	RLCF	PCLATH, F
	ANDLW	0xfc
	ADDLW	LOW(_00801_DS_)
	MOVWF	POSTDEC1
	MOVLW	HIGH(_00801_DS_)
	ADDWFC	PCLATH, F
	MOVLW	UPPER(_00801_DS_)
	ADDWFC	PCLATU, F
	MOVF	PREINC1, W
	MOVWF	PCL
_00801_DS_:
	GOTO	_00779_DS_
	GOTO	_00780_DS_
	GOTO	_00781_DS_
	GOTO	_00782_DS_
_00779_DS_:
	BANKSEL	_fifo_tail
;	.line	3910; meter_logger.c	*c = fifo_buffer_0[fifo_tail % QUEUE_SIZE];
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
;	.line	3911; meter_logger.c	break;
	BRA	_00783_DS_
_00780_DS_:
	BANKSEL	_fifo_tail
;	.line	3913; meter_logger.c	*c = fifo_buffer_1[fifo_tail % QUEUE_SIZE];
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
;	.line	3914; meter_logger.c	break;
	BRA	_00783_DS_
_00781_DS_:
	BANKSEL	_fifo_tail
;	.line	3916; meter_logger.c	*c = fifo_buffer_2[fifo_tail % QUEUE_SIZE];
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
;	.line	3917; meter_logger.c	break;
	BRA	_00783_DS_
_00782_DS_:
	BANKSEL	_fifo_tail
;	.line	3919; meter_logger.c	*c = fifo_buffer_3[fifo_tail % QUEUE_SIZE];
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
_00783_DS_:
	BANKSEL	_fifo_tail
;	.line	3922; meter_logger.c	fifo_tail++;
	INCFSZ	_fifo_tail, F, B
	BRA	_10845_DS_
; removed redundant BANKSEL
	INCF	(_fifo_tail + 1), F, B
_10845_DS_:
	BANKSEL	_fifo_tail
;	.line	3924; meter_logger.c	if (fifo_tail == QUEUE_SIZE_COMBINED) {
	MOVF	_fifo_tail, W, B
	BNZ	_00806_DS_
; removed redundant BANKSEL
	MOVF	(_fifo_tail + 1), W, B
	XORLW	0x04
	BZ	_00807_DS_
_00806_DS_:
	BRA	_00785_DS_
_00807_DS_:
	BANKSEL	_fifo_tail
;	.line	3925; meter_logger.c	fifo_tail = 0;
	CLRF	_fifo_tail, B
; removed redundant BANKSEL
	CLRF	(_fifo_tail + 1), B
_00785_DS_:
;	.line	3927; meter_logger.c	return 1;
	MOVLW	0x01
	BRA	_00789_DS_
_00787_DS_:
;	.line	3930; meter_logger.c	return 0;
	CLRF	WREG
_00789_DS_:
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
;	.line	3878; meter_logger.c	unsigned char fifo_put(unsigned char c) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVFF	r0x01, POSTDEC1
	MOVFF	r0x02, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
;	.line	3879; meter_logger.c	if (fifo_in_use() != QUEUE_SIZE_COMBINED) {
	CALL	_fifo_in_use
	MOVWF	r0x01
	MOVFF	PRODL, r0x02
	MOVF	r0x01, W
	BNZ	_00766_DS_
	MOVF	r0x02, W
	XORLW	0x04
	BNZ	_00766_DS_
	BRA	_00752_DS_
_00766_DS_:
	BANKSEL	(_fifo_head + 1)
;	.line	3880; meter_logger.c	switch (fifo_head/QUEUE_SIZE) {
	MOVF	(_fifo_head + 1), W, B
	MOVWF	r0x01
	CLRF	r0x02
	MOVLW	0x00
	SUBWF	r0x02, W
	BNZ	_00767_DS_
	MOVLW	0x04
	SUBWF	r0x01, W
_00767_DS_:
	BTFSC	STATUS, 0
	BRA	_00748_DS_
	CLRF	PCLATH
	CLRF	PCLATU
	RLCF	r0x01, W
	RLCF	PCLATH, F
	RLCF	WREG, W
	RLCF	PCLATH, F
	ANDLW	0xfc
	ADDLW	LOW(_00768_DS_)
	MOVWF	POSTDEC1
	MOVLW	HIGH(_00768_DS_)
	ADDWFC	PCLATH, F
	MOVLW	UPPER(_00768_DS_)
	ADDWFC	PCLATU, F
	MOVF	PREINC1, W
	MOVWF	PCL
_00768_DS_:
	GOTO	_00744_DS_
	GOTO	_00745_DS_
	GOTO	_00746_DS_
	GOTO	_00747_DS_
_00744_DS_:
	BANKSEL	_fifo_head
;	.line	3882; meter_logger.c	fifo_buffer_0[fifo_head % QUEUE_SIZE] = c;
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
;	.line	3883; meter_logger.c	break;
	BRA	_00748_DS_
_00745_DS_:
	BANKSEL	_fifo_head
;	.line	3885; meter_logger.c	fifo_buffer_1[fifo_head % QUEUE_SIZE] = c;
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
;	.line	3886; meter_logger.c	break;
	BRA	_00748_DS_
_00746_DS_:
	BANKSEL	_fifo_head
;	.line	3888; meter_logger.c	fifo_buffer_2[fifo_head % QUEUE_SIZE] = c;
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
;	.line	3889; meter_logger.c	break;
	BRA	_00748_DS_
_00747_DS_:
	BANKSEL	_fifo_head
;	.line	3891; meter_logger.c	fifo_buffer_3[fifo_head % QUEUE_SIZE] = c;
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
_00748_DS_:
	BANKSEL	_fifo_head
;	.line	3894; meter_logger.c	fifo_head++;
	INCFSZ	_fifo_head, F, B
	BRA	_20846_DS_
; removed redundant BANKSEL
	INCF	(_fifo_head + 1), F, B
_20846_DS_:
	BANKSEL	_fifo_head
;	.line	3896; meter_logger.c	if (fifo_head == QUEUE_SIZE_COMBINED) {
	MOVF	_fifo_head, W, B
	BNZ	_00773_DS_
; removed redundant BANKSEL
	MOVF	(_fifo_head + 1), W, B
	XORLW	0x04
	BZ	_00774_DS_
_00773_DS_:
	BRA	_00750_DS_
_00774_DS_:
	BANKSEL	_fifo_head
;	.line	3897; meter_logger.c	fifo_head = 0;
	CLRF	_fifo_head, B
; removed redundant BANKSEL
	CLRF	(_fifo_head + 1), B
_00750_DS_:
;	.line	3899; meter_logger.c	return 1;
	MOVLW	0x01
	BRA	_00754_DS_
_00752_DS_:
;	.line	3902; meter_logger.c	return 0;
	CLRF	WREG
_00754_DS_:
	MOVFF	PREINC1, r0x02
	MOVFF	PREINC1, r0x01
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__fifo_in_use	code
_fifo_in_use:
;	.line	3874; meter_logger.c	unsigned int fifo_in_use() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVFF	r0x01, POSTDEC1
	BANKSEL	_fifo_tail
;	.line	3875; meter_logger.c	return fifo_head - fifo_tail;
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
;	.line	3869; meter_logger.c	void fsk_tx_byte(unsigned char c) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
;	.line	3870; meter_logger.c	fsk_proto.data = c;
	MOVF	r0x00, W
	BANKSEL	(_fsk_proto + 12)
	MOVWF	(_fsk_proto + 12), B
;	.line	3871; meter_logger.c	fsk_proto.data_len = 8;
	MOVLW	0x08
; removed redundant BANKSEL
	MOVWF	(_fsk_proto + 13), B
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__send_fsk_low	code
_send_fsk_low:
;	.line	2416; meter_logger.c	void send_fsk_low(void) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
;	.line	2417; meter_logger.c	PWM_PIN = 1;
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
	
;	.line	2624; meter_logger.c	PWM_PIN = 0;
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
	
;	.line	2831; meter_logger.c	PWM_PIN = 1;
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
	
;	.line	3038; meter_logger.c	PWM_PIN = 0;
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
	
;	.line	3245; meter_logger.c	PWM_PIN = 1;
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
	
;	.line	3452; meter_logger.c	PWM_PIN = 0;
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
	
;	.line	3659; meter_logger.c	PWM_PIN = 1;
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
	
;	.line	3866; meter_logger.c	PWM_PIN = 0;
	BCF	_PORTCbits, 1
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__send_fsk_high	code
_send_fsk_high:
;	.line	894; meter_logger.c	void send_fsk_high(void) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
;	.line	895; meter_logger.c	PWM_PIN = 1;
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
	
;	.line	1033; meter_logger.c	PWM_PIN = 0;
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
	
;	.line	1171; meter_logger.c	PWM_PIN = 1;
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
	
;	.line	1309; meter_logger.c	PWM_PIN = 0;
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
	
;	.line	1447; meter_logger.c	PWM_PIN = 1;
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
	
;	.line	1585; meter_logger.c	PWM_PIN = 0;
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
	
;	.line	1723; meter_logger.c	PWM_PIN = 1;
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
	
;	.line	1861; meter_logger.c	PWM_PIN = 0;
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
	
;	.line	1999; meter_logger.c	PWM_PIN = 1;
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
	
;	.line	2137; meter_logger.c	PWM_PIN = 0;
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
	
;	.line	2275; meter_logger.c	PWM_PIN = 1;
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
	
;	.line	2413; meter_logger.c	PWM_PIN = 0;
	BCF	_PORTCbits, 1
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__fsk_rx_disable	code
_fsk_rx_disable:
;	.line	889; meter_logger.c	void fsk_rx_disable() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
;	.line	890; meter_logger.c	PIE2bits.CMIE = 0;		// Disable comparator interrupt
	BCF	_PIE2bits, 6
	BANKSEL	_codec_type
;	.line	891; meter_logger.c	codec_type = NONE;
	CLRF	_codec_type, B
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__fsk_rx_enable	code
_fsk_rx_enable:
;	.line	853; meter_logger.c	void fsk_rx_enable() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
;	.line	854; meter_logger.c	fsk_proto.state = START_BIT_WAIT;
	MOVLW	0x02
	BANKSEL	_fsk_proto
	MOVWF	_fsk_proto, B
; removed redundant BANKSEL
;	.line	855; meter_logger.c	fsk_proto.start_bit_time = 0;
	CLRF	(_fsk_proto + 10), B
; removed redundant BANKSEL
	CLRF	(_fsk_proto + 11), B
;	.line	857; meter_logger.c	timer0_reload = TIMER0_FSK;
	MOVLW	0x9f
	BANKSEL	_timer0_reload
	MOVWF	_timer0_reload, B
	MOVLW	0xf9
; removed redundant BANKSEL
	MOVWF	(_timer0_reload + 1), B
;	.line	859; meter_logger.c	codec_type = FSK_RX;
	MOVLW	0x04
	BANKSEL	_codec_type
	MOVWF	_codec_type, B
;	.line	862; meter_logger.c	T0CONbits.TMR0ON = 1;
	BSF	_T0CONbits, 7
;	.line	863; meter_logger.c	T0CONbits.T0PS0 = 0;
	BCF	_T0CONbits, 0
;	.line	864; meter_logger.c	T0CONbits.T0PS1 = 0;
	BCF	_T0CONbits, 1
;	.line	865; meter_logger.c	T0CONbits.T0PS2 = 0;		// prescaler 1:2
	BCF	_T0CONbits, 2
;	.line	866; meter_logger.c	T0CONbits.T08BIT = 0;		// use timer0 16-bit counter
	BCF	_T0CONbits, 6
;	.line	867; meter_logger.c	T0CONbits.T0CS = 0;			// internal clock source
	BCF	_T0CONbits, 5
;	.line	868; meter_logger.c	T0CONbits.PSA = 1;			// disable timer0 prescaler
	BSF	_T0CONbits, 3
;	.line	869; meter_logger.c	INTCON2bits.TMR0IP = 1;		// high priority
	BSF	_INTCON2bits, 2
;	.line	870; meter_logger.c	INTCONbits.TMR0IE = 0;		// Dont enable TMR0 Interrupt
	BCF	_INTCONbits, 5
;	.line	873; meter_logger.c	CVRCONbits.CVREF = 0xf;	// 0V
	BSF	_CVRCONbits, 4
;	.line	875; meter_logger.c	CVRCONbits.CVRSS = 0;	// VDD – VSS
	BCF	_CVRCONbits, 4
;	.line	876; meter_logger.c	CVRCONbits.CVRR = 0;	// high range, 0.25 CVRSRC to 0.75 CVRSRC, with CVRSRC/32 step size
	BCF	_CVRCONbits, 5
;	.line	877; meter_logger.c	CVRCONbits.CVR = 9;		// 2,65625 V
	MOVF	_CVRCONbits, W
	ANDLW	0xf0
	IORLW	0x09
	MOVWF	_CVRCONbits
;	.line	878; meter_logger.c	CVRCONbits.CVROE = 0;	// Comparator VREF Output disabled, CVREF voltage is disconnected from the RA2/AN2/VREF-/CVREF pin
	BCF	_CVRCONbits, 6
;	.line	879; meter_logger.c	CVRCONbits.CVREN = 1;	// Comparator Voltage Reference Enable bit
	BSF	_CVRCONbits, 7
;	.line	881; meter_logger.c	CMCONbits.CM = 0x6;		// four inputs multiplexed to two comparators
	MOVF	_CMCONbits, W
	ANDLW	0xf8
	IORLW	0x06
	MOVWF	_CMCONbits
;	.line	882; meter_logger.c	CMCONbits.CIS = 0;		// multiplexed to RA0/AN0 and RA1/AN1
	BCF	_CMCONbits, 3
;	.line	883; meter_logger.c	CMCONbits.C1INV = 1;	// inverted output, C1 VIN+ < C1 VIN-
	BSF	_CMCONbits, 4
;	.line	885; meter_logger.c	IPR2bits.CMIP = 1;		// high priority
	BSF	_IPR2bits, 6
;	.line	886; meter_logger.c	PIE2bits.CMIE = 1;		// Enable comparator interrupt
	BSF	_PIE2bits, 6
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__fsk_tx_disable	code
_fsk_tx_disable:
;	.line	849; meter_logger.c	void fsk_tx_disable() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
;	.line	850; meter_logger.c	T2CONbits.TMR2ON = 0;	// timer 2 off
	BCF	_T2CONbits, 2
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__fsk_tx_enable	code
_fsk_tx_enable:
;	.line	831; meter_logger.c	void fsk_tx_enable() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
;	.line	832; meter_logger.c	timer0_reload = TIMER0_FSK;
	MOVLW	0x9f
	BANKSEL	_timer0_reload
	MOVWF	_timer0_reload, B
	MOVLW	0xf9
; removed redundant BANKSEL
	MOVWF	(_timer0_reload + 1), B
	BANKSEL	_fsk_proto
;	.line	834; meter_logger.c	fsk_proto.state = INIT_STATE;
	CLRF	_fsk_proto, B
;	.line	835; meter_logger.c	codec_type = FSK_TX;
	MOVLW	0x05
	BANKSEL	_codec_type
	MOVWF	_codec_type, B
;	.line	838; meter_logger.c	T0CONbits.TMR0ON = 1;
	BSF	_T0CONbits, 7
;	.line	839; meter_logger.c	T0CONbits.T0PS0 = 0;
	BCF	_T0CONbits, 0
;	.line	840; meter_logger.c	T0CONbits.T0PS1 = 0;
	BCF	_T0CONbits, 1
;	.line	841; meter_logger.c	T0CONbits.T0PS2 = 0;		// prescaler 1:2
	BCF	_T0CONbits, 2
;	.line	842; meter_logger.c	T0CONbits.T08BIT = 0;		// use timer0 16-bit counter
	BCF	_T0CONbits, 6
;	.line	843; meter_logger.c	T0CONbits.T0CS = 0;			// internal clock source
	BCF	_T0CONbits, 5
;	.line	844; meter_logger.c	T0CONbits.PSA = 1;			// disable timer0 prescaler
	BSF	_T0CONbits, 3
;	.line	845; meter_logger.c	INTCON2bits.TMR0IP = 1;		// high priority
	BSF	_INTCON2bits, 2
;	.line	846; meter_logger.c	INTCONbits.TMR0IE = 1;		// Enable TMR0 Interrupt
	BSF	_INTCONbits, 5
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__rs232_tx_disable	code
_rs232_tx_disable:
;	.line	827; meter_logger.c	void rs232_tx_disable() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
;	.line	829; meter_logger.c	}
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__rs232_tx_enable	code
_rs232_tx_enable:
;	.line	801; meter_logger.c	void rs232_tx_enable() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	BANKSEL	_rs232_ir_proto
;	.line	802; meter_logger.c	rs232_ir_proto.state = INIT_STATE;
	CLRF	_rs232_ir_proto, B
;	.line	805; meter_logger.c	timer0_reload = TIMER0_RS232_2400;
	MOVLW	0xf2
	BANKSEL	_timer0_reload
	MOVWF	_timer0_reload, B
	MOVLW	0xfc
; removed redundant BANKSEL
	MOVWF	(_timer0_reload + 1), B
;	.line	807; meter_logger.c	PWM_PIN = 1;
	BSF	_PORTCbits, 1
;	.line	809; meter_logger.c	codec_type = RS232_TX;
	MOVLW	0x03
	BANKSEL	_codec_type
	MOVWF	_codec_type, B
;	.line	812; meter_logger.c	T0CONbits.TMR0ON = 0;
	BCF	_T0CONbits, 7
;	.line	813; meter_logger.c	T0CONbits.T0PS0 = 0;
	BCF	_T0CONbits, 0
;	.line	814; meter_logger.c	T0CONbits.T0PS1 = 0;
	BCF	_T0CONbits, 1
;	.line	815; meter_logger.c	T0CONbits.T0PS2 = 0;		// prescaler 1:2
	BCF	_T0CONbits, 2
;	.line	816; meter_logger.c	T0CONbits.T08BIT = 0;		// use timer0 16-bit counter
	BCF	_T0CONbits, 6
;	.line	817; meter_logger.c	T0CONbits.T0CS = 0;			// internal clock source
	BCF	_T0CONbits, 5
;	.line	818; meter_logger.c	T0CONbits.PSA = 1;			// disable timer0 prescaler
	BSF	_T0CONbits, 3
;	.line	819; meter_logger.c	INTCON2bits.TMR0IP = 1;		// high priority
	BSF	_INTCON2bits, 2
;	.line	820; meter_logger.c	INTCONbits.TMR0IE = 1;		// Enable TMR0 Interrupt
	BSF	_INTCONbits, 5
;	.line	821; meter_logger.c	INTCONbits.TMR0IF = 0;
	BCF	_INTCONbits, 2
;	.line	823; meter_logger.c	INTCONbits.INT0IE = 0;		// disable ext int while sending with software uart
	BCF	_INTCONbits, 4
;	.line	824; meter_logger.c	T0CONbits.TMR0ON = 1;		// start timer 0
	BSF	_T0CONbits, 7
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__testo_ir_disable	code
_testo_ir_disable:
;	.line	796; meter_logger.c	void testo_ir_disable() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	BANKSEL	_codec_type
;	.line	797; meter_logger.c	codec_type = NONE;
	CLRF	_codec_type, B
;	.line	798; meter_logger.c	INTCONbits.INT0IE = 0;		// disable ext int
	BCF	_INTCONbits, 4
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__testo_ir_enable	code
_testo_ir_enable:
;	.line	772; meter_logger.c	void testo_ir_enable() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	BANKSEL	_testo_ir_proto
;	.line	773; meter_logger.c	testo_ir_proto.state = INIT_STATE;
	CLRF	_testo_ir_proto, B
; removed redundant BANKSEL
;	.line	774; meter_logger.c	testo_ir_proto.start_bit_len = 0;
	CLRF	(_testo_ir_proto + 2), B
;	.line	776; meter_logger.c	timer0_reload = TIMER0_TESTO;
	MOVLW	0x23
	BANKSEL	_timer0_reload
	MOVWF	_timer0_reload, B
	MOVLW	0xf3
; removed redundant BANKSEL
	MOVWF	(_timer0_reload + 1), B
;	.line	778; meter_logger.c	codec_type = TESTO;
	MOVLW	0x01
	BANKSEL	_codec_type
	MOVWF	_codec_type, B
;	.line	781; meter_logger.c	T0CONbits.TMR0ON = 0;
	BCF	_T0CONbits, 7
;	.line	782; meter_logger.c	T0CONbits.T0PS0 = 0;
	BCF	_T0CONbits, 0
;	.line	783; meter_logger.c	T0CONbits.T0PS1 = 0;
	BCF	_T0CONbits, 1
;	.line	784; meter_logger.c	T0CONbits.T0PS2 = 0;		// prescaler 1:2
	BCF	_T0CONbits, 2
;	.line	785; meter_logger.c	T0CONbits.T08BIT = 0;		// use timer0 16-bit counter
	BCF	_T0CONbits, 6
;	.line	786; meter_logger.c	T0CONbits.T0CS = 0;			// internal clock source
	BCF	_T0CONbits, 5
;	.line	787; meter_logger.c	T0CONbits.PSA = 1;			// disable timer0 prescaler
	BSF	_T0CONbits, 3
;	.line	788; meter_logger.c	INTCON2bits.TMR0IP = 1;		// high priority
	BSF	_INTCON2bits, 2
;	.line	789; meter_logger.c	INTCONbits.TMR0IE = 1;		// Enable TMR0 Interrupt
	BSF	_INTCONbits, 5
;	.line	790; meter_logger.c	INTCONbits.TMR0IF = 0;
	BCF	_INTCONbits, 2
;	.line	792; meter_logger.c	INTCONbits.INT0IE = 1;		// enable ext int
	BSF	_INTCONbits, 4
;	.line	793; meter_logger.c	INTCON2bits.INTEDG0 = 1;	// rising edge
	BSF	_INTCON2bits, 6
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__valid_err_corr	code
_valid_err_corr:
;	.line	717; meter_logger.c	unsigned char valid_err_corr(unsigned int c) {
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
;	.line	724; meter_logger.c	calculated_err_corr_bit = 0;
	CLRF	r0x02
;	.line	725; meter_logger.c	for (i = 0; i < 8; i++) {
	MOVLW	0x78
	ANDWF	r0x00, W
	MOVWF	r0x03
	CLRF	r0x04
	CLRF	r0x05
_00602_DS_:
;	.line	726; meter_logger.c	calculated_err_corr_bit ^= (((c & 0x78) & (1 << i)) != 0);   // 0b01111000
	MOVLW	0x01
	MOVWF	r0x06
	MOVLW	0x00
	MOVWF	r0x07
	MOVF	r0x05, W
	BZ	_00644_DS_
	NEGF	WREG
	BCF	STATUS, 0
_00645_DS_:
	RLCF	r0x06, F
	RLCF	r0x07, F
	ADDLW	0x01
	BNC	_00645_DS_
_00644_DS_:
	MOVF	r0x03, W
	ANDWF	r0x06, F
	MOVF	r0x04, W
	ANDWF	r0x07, F
	MOVF	r0x06, W
	BNZ	_00647_DS_
	MOVF	r0x07, W
	BNZ	_00647_DS_
	CLRF	r0x06
	INCF	r0x06, F
	BRA	_00648_DS_
_00647_DS_:
	CLRF	r0x06
_00648_DS_:
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
;	.line	725; meter_logger.c	for (i = 0; i < 8; i++) {
	INCF	r0x05, F
	MOVLW	0x08
	SUBWF	r0x05, W
	BNC	_00602_DS_
;	.line	729; meter_logger.c	calculated_err_corr = calculated_err_corr << 1;
	RLNCF	r0x02, W
	ANDLW	0xfe
	MOVWF	r0x03
;	.line	732; meter_logger.c	calculated_err_corr_bit = 0;
	CLRF	r0x02
;	.line	733; meter_logger.c	for (i = 0; i < 8; i++) {
	MOVLW	0xe6
	ANDWF	r0x00, W
	MOVWF	r0x04
	CLRF	r0x05
	CLRF	r0x06
_00604_DS_:
;	.line	734; meter_logger.c	calculated_err_corr_bit ^= (((c & 0xe6) & (1 << i)) != 0);   // 0b11100110
	MOVLW	0x01
	MOVWF	r0x07
	MOVLW	0x00
	MOVWF	r0x08
	MOVF	r0x06, W
	BZ	_00652_DS_
	NEGF	WREG
	BCF	STATUS, 0
_00653_DS_:
	RLCF	r0x07, F
	RLCF	r0x08, F
	ADDLW	0x01
	BNC	_00653_DS_
_00652_DS_:
	MOVF	r0x04, W
	ANDWF	r0x07, F
	MOVF	r0x05, W
	ANDWF	r0x08, F
	MOVF	r0x07, W
	BNZ	_00655_DS_
	MOVF	r0x08, W
	BNZ	_00655_DS_
	CLRF	r0x07
	INCF	r0x07, F
	BRA	_00656_DS_
_00655_DS_:
	CLRF	r0x07
_00656_DS_:
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
;	.line	733; meter_logger.c	for (i = 0; i < 8; i++) {
	INCF	r0x06, F
	MOVLW	0x08
	SUBWF	r0x06, W
	BNC	_00604_DS_
;	.line	736; meter_logger.c	calculated_err_corr |= calculated_err_corr_bit;
	MOVF	r0x02, W
	IORWF	r0x03, F
;	.line	737; meter_logger.c	calculated_err_corr = calculated_err_corr << 1;
	BCF	STATUS, 0
	RLCF	r0x03, F
;	.line	740; meter_logger.c	calculated_err_corr_bit = 0;
	CLRF	r0x02
;	.line	741; meter_logger.c	for (i = 0; i < 8; i++) {
	MOVLW	0xd5
	ANDWF	r0x00, W
	MOVWF	r0x04
	CLRF	r0x05
	CLRF	r0x06
_00606_DS_:
;	.line	742; meter_logger.c	calculated_err_corr_bit ^= (((c & 0xd5) & (1 << i)) != 0);   // 0b11010101
	MOVLW	0x01
	MOVWF	r0x07
	MOVLW	0x00
	MOVWF	r0x08
	MOVF	r0x06, W
	BZ	_00661_DS_
	NEGF	WREG
	BCF	STATUS, 0
_00662_DS_:
	RLCF	r0x07, F
	RLCF	r0x08, F
	ADDLW	0x01
	BNC	_00662_DS_
_00661_DS_:
	MOVF	r0x04, W
	ANDWF	r0x07, F
	MOVF	r0x05, W
	ANDWF	r0x08, F
	MOVF	r0x07, W
	BNZ	_00664_DS_
	MOVF	r0x08, W
	BNZ	_00664_DS_
	CLRF	r0x07
	INCF	r0x07, F
	BRA	_00665_DS_
_00664_DS_:
	CLRF	r0x07
_00665_DS_:
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
;	.line	741; meter_logger.c	for (i = 0; i < 8; i++) {
	INCF	r0x06, F
	MOVLW	0x08
	SUBWF	r0x06, W
	BNC	_00606_DS_
;	.line	744; meter_logger.c	calculated_err_corr |= calculated_err_corr_bit;
	MOVF	r0x02, W
	IORWF	r0x03, F
;	.line	745; meter_logger.c	calculated_err_corr = calculated_err_corr << 1;
	BCF	STATUS, 0
	RLCF	r0x03, F
;	.line	748; meter_logger.c	calculated_err_corr_bit = 0;
	CLRF	r0x02
;	.line	749; meter_logger.c	for (i = 0; i < 8; i++) {
	MOVLW	0x8b
	ANDWF	r0x00, W
	MOVWF	r0x04
	CLRF	r0x05
	CLRF	r0x06
_00608_DS_:
;	.line	750; meter_logger.c	calculated_err_corr_bit ^= (((c & 0x8b) & (1 << i)) != 0);   // 0b10001011
	MOVLW	0x01
	MOVWF	r0x07
	MOVLW	0x00
	MOVWF	r0x08
	MOVF	r0x06, W
	BZ	_00670_DS_
	NEGF	WREG
	BCF	STATUS, 0
_00671_DS_:
	RLCF	r0x07, F
	RLCF	r0x08, F
	ADDLW	0x01
	BNC	_00671_DS_
_00670_DS_:
	MOVF	r0x04, W
	ANDWF	r0x07, F
	MOVF	r0x05, W
	ANDWF	r0x08, F
	MOVF	r0x07, W
	BNZ	_00673_DS_
	MOVF	r0x08, W
	BNZ	_00673_DS_
	CLRF	r0x07
	INCF	r0x07, F
	BRA	_00674_DS_
_00673_DS_:
	CLRF	r0x07
_00674_DS_:
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
;	.line	749; meter_logger.c	for (i = 0; i < 8; i++) {
	INCF	r0x06, F
	MOVLW	0x08
	SUBWF	r0x06, W
	BNC	_00608_DS_
;	.line	752; meter_logger.c	calculated_err_corr |= calculated_err_corr_bit;
	MOVF	r0x02, W
	IORWF	r0x03, F
;	.line	763; meter_logger.c	if ((c >> 8) == calculated_err_corr) {
	MOVF	r0x01, W
	MOVWF	r0x00
	CLRF	r0x01
	CLRF	r0x02
	MOVF	r0x00, W
	XORWF	r0x03, W
	BNZ	_00678_DS_
	MOVF	r0x01, W
	XORWF	r0x02, W
	BZ	_00679_DS_
_00678_DS_:
	BRA	_00600_DS_
_00679_DS_:
;	.line	764; meter_logger.c	return 1;
	MOVLW	0x01
	BRA	_00610_DS_
_00600_DS_:
;	.line	767; meter_logger.c	return 0;
	CLRF	WREG
_00610_DS_:
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
;	.line	709; meter_logger.c	unsigned char reverse(unsigned char b) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVFF	r0x01, POSTDEC1
	MOVFF	r0x02, POSTDEC1
	MOVFF	r0x03, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
;	.line	711; meter_logger.c	c  = ((b >>  1) & 0x55) | ((b <<  1) & 0xaa);
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
;	.line	712; meter_logger.c	c |= ((b >>  2) & 0x33) | ((b <<  2) & 0xcc);
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
;	.line	713; meter_logger.c	c |= ((b >>  4) & 0x0f) | ((b <<  4) & 0xf0);
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
;	.line	714; meter_logger.c	return(c);
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
;	.line	679; meter_logger.c	void my_usart_open() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
;	.line	680; meter_logger.c	SPBRG = 103;					// 8MHz => 19230 baud
	MOVLW	0x67
	MOVWF	_SPBRG
;	.line	681; meter_logger.c	TXSTAbits.BRGH = 1;	// (0 = low speed)
	BSF	_TXSTAbits, 2
;	.line	682; meter_logger.c	TXSTAbits.SYNC = 0;	// (0 = asynchronous)
	BCF	_TXSTAbits, 4
;	.line	683; meter_logger.c	BAUDCONbits.BRG16 = 1;
	BSF	_BAUDCONbits, 3
;	.line	686; meter_logger.c	RCSTAbits.SPEN = 1; // (1 = serial port enabled)
	BSF	_RCSTAbits, 7
;	.line	689; meter_logger.c	PIE1bits.TXIE = 0; // (1 = enabled)
	BCF	_PIE1bits, 4
;	.line	690; meter_logger.c	IPR1bits.TXIP = 0; // USART Tx on low priority interrupt
	BCF	_IPR1bits, 4
;	.line	693; meter_logger.c	PIE1bits.RCIE = 1; // (1 = enabled)
	BSF	_PIE1bits, 5
;	.line	694; meter_logger.c	IPR1bits.RCIP = 0; // USART Rx on low priority interrupt
	BCF	_IPR1bits, 5
;	.line	697; meter_logger.c	TXSTAbits.TX9 = 0; // (0 = 8-bit transmit)
	BCF	_TXSTAbits, 6
;	.line	700; meter_logger.c	RCSTAbits.RX9 = 0; // (0 = 8-bit reception)
	BCF	_RCSTAbits, 6
;	.line	703; meter_logger.c	RCSTAbits.CREN = 1; // (1 = Enables receiver)
	BSF	_RCSTAbits, 4
;	.line	706; meter_logger.c	TXSTAbits.TXEN = 1; // (1 = transmit enabled)
	BSF	_TXSTAbits, 5
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__init_system	code
_init_system:
;	.line	586; meter_logger.c	void init_system() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
;	.line	588; meter_logger.c	TRIS_IR_PIN = INPUT_STATE;		// as input
	BSF	_TRISBbits, 0
;	.line	589; meter_logger.c	TRIS_LED_PIN = OUTPUT_STATE;	// as output
	BCF	_TRISBbits, 4
;	.line	590; meter_logger.c	LED_PIN = 0;					// and clear
	BCF	_PORTBbits, 4
;	.line	592; meter_logger.c	TRIS_DEBUG_PIN = OUTPUT_STATE;	// as output
	BCF	_TRISBbits, 1
;	.line	593; meter_logger.c	DEBUG_PIN = 0;					// and clear
	BCF	_PORTBbits, 1
;	.line	594; meter_logger.c	TRIS_DEBUG2_PIN = OUTPUT_STATE;	// as output
	BCF	_TRISBbits, 2
;	.line	595; meter_logger.c	DEBUG2_PIN = 0;					// and clear
	BCF	_PORTBbits, 2
;	.line	596; meter_logger.c	TRIS_DEBUG3_PIN = OUTPUT_STATE;	// as output
	BCF	_TRISBbits, 3
;	.line	597; meter_logger.c	DEBUG3_PIN = 0;					// and clear
	BCF	_PORTBbits, 3
;	.line	599; meter_logger.c	TRIS_COMP1 = INPUT_STATE;		// as input
	BSF	_TRISAbits, 0
;	.line	600; meter_logger.c	TRIS_COMP2 = INPUT_STATE;		// as input
	BSF	_TRISAbits, 1
;	.line	604; meter_logger.c	TRIS_PWM_PIN = OUTPUT_STATE;	// enable output from pwm module at init
	BCF	_TRISCbits, 1
;	.line	605; meter_logger.c	PWM_PIN = 0;					// and clear
	BCF	_PORTCbits, 1
;	.line	608; meter_logger.c	TRIS_RX_PIN = INPUT_STATE;		// as input
	BSF	_TRISCbits, 7
;	.line	609; meter_logger.c	TRIS_TX_PIN = OUTPUT_STATE;		// as input
	BCF	_TRISCbits, 6
;	.line	614; meter_logger.c	T1CONbits.TMR1ON = 1;
	BSF	_T1CONbits, 0
;	.line	615; meter_logger.c	T1CONbits.RD16 = 1;
	BSF	_T1CONbits, 7
;	.line	616; meter_logger.c	T1CONbits.TMR1CS = 0;   // internal clock source
	BCF	_T1CONbits, 1
;	.line	617; meter_logger.c	T1CONbits.T1OSCEN = 0;  // dont put t1 on pin
	BCF	_T1CONbits, 3
;	.line	618; meter_logger.c	T1CONbits.T1CKPS0 = 0;
	BCF	_T1CONbits, 4
;	.line	619; meter_logger.c	T1CONbits.T1CKPS1 = 0;
	BCF	_T1CONbits, 5
;	.line	620; meter_logger.c	IPR1bits.TMR1IP = 0;	// low priority
	BCF	_IPR1bits, 0
;	.line	621; meter_logger.c	PIE1bits.TMR1IE = 1;	// Ensure that TMR1 Interrupt is enabled
	BSF	_PIE1bits, 0
;	.line	622; meter_logger.c	PIR1bits.TMR1IF = 1;	// Force Instant entry to Timer 1 Interrupt
	BSF	_PIR1bits, 0
;	.line	653; meter_logger.c	RCONbits.IPEN = 1;
	BSF	_RCONbits, 7
;	.line	655; meter_logger.c	INTCONbits.INT0IE = 0;		// disable ext int, enabled when ir demodulator is started
	BCF	_INTCONbits, 4
;	.line	656; meter_logger.c	INTCON2bits.INTEDG0 = 1;	// rising edge
	BSF	_INTCON2bits, 6
;	.line	658; meter_logger.c	INTCONbits.PEIE = 1;
	BSF	_INTCONbits, 6
;	.line	659; meter_logger.c	INTCONbits.GIE = 1;	/* Enable Global interrupts   */	
	BSF	_INTCONbits, 7
;	.line	664; meter_logger.c	IPR1bits.RCIP = 0;
	BCF	_IPR1bits, 5
;	.line	665; meter_logger.c	IPR1bits.TXIP = 0;
	BCF	_IPR1bits, 4
;	.line	676; meter_logger.c	my_usart_open();
	CALL	_my_usart_open
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__sleep_ms	code
_sleep_ms:
;	.line	576; meter_logger.c	void sleep_ms(unsigned long ms) {
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
;	.line	578; meter_logger.c	start_timer_1_ms = timer_1_ms;	
	MOVFF	_timer_1_ms, r0x04
	MOVFF	(_timer_1_ms + 1), r0x05
	MOVFF	(_timer_1_ms + 2), r0x06
	MOVFF	(_timer_1_ms + 3), r0x07
_00562_DS_:
;	.line	581; meter_logger.c	while ( (((signed long)(timer_1_ms - start_timer_1_ms) < 0) ? (-1 * (timer_1_ms - start_timer_1_ms)) : (timer_1_ms - start_timer_1_ms)) < ms) {
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
	BNC	_00567_DS_
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
	BRA	_00568_DS_
_00567_DS_:
	MOVFF	r0x08, r0x0c
	MOVFF	r0x09, r0x0d
	MOVFF	r0x0a, r0x0e
	MOVFF	r0x0b, r0x0f
_00568_DS_:
	MOVF	r0x03, W
	SUBWF	r0x0f, W
	BNZ	_00575_DS_
	MOVF	r0x02, W
	SUBWF	r0x0e, W
	BNZ	_00575_DS_
	MOVF	r0x01, W
	SUBWF	r0x0d, W
	BNZ	_00575_DS_
	MOVF	r0x00, W
	SUBWF	r0x0c, W
_00575_DS_:
	BTFSS	STATUS, 0
	BRA	_00562_DS_
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
;	.line	544; meter_logger.c	static void isr_low_prio(void) __interrupt 2 {
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
;	.line	547; meter_logger.c	if (PIR1bits.TMR1IF) {
	BTFSS	_PIR1bits, 0
	BRA	_00533_DS_
;	.line	548; meter_logger.c	TMR1H = (unsigned char)(TIMER1_RELOAD >> 8);    // 262,158ms @ 8MHz
	MOVLW	0xf8
	MOVWF	_TMR1H
;	.line	549; meter_logger.c	TMR1L = (unsigned char)TIMER1_RELOAD;
	MOVLW	0x53
	MOVWF	_TMR1L
;	.line	551; meter_logger.c	switch (led_flash.state) {
	MOVFF	_led_flash, r0x00
	MOVF	r0x00, W
	MOVWF	r0x01
	MOVF	r0x01, W
	BZ	_00527_DS_
	MOVF	r0x00, W
	XORLW	0x01
	BZ	_00528_DS_
	BRA	_00531_DS_
_00527_DS_:
;	.line	553; meter_logger.c	LED_PIN = 1;
	BSF	_PORTBbits, 4
;	.line	554; meter_logger.c	led_flash.state = LED_FLASH_RUNNING;
	MOVLW	0x01
	BANKSEL	_led_flash
	MOVWF	_led_flash, B
;	.line	555; meter_logger.c	break;
	BRA	_00531_DS_
_00528_DS_:
;	.line	557; meter_logger.c	if (led_flash.timer-- == 0) {
	MOVFF	(_led_flash + 1), r0x00
	DECF	r0x00, W
	MOVWF	r0x01
	MOVF	r0x01, W
	BANKSEL	(_led_flash + 1)
	MOVWF	(_led_flash + 1), B
	MOVF	r0x00, W
	BNZ	_00531_DS_
;	.line	558; meter_logger.c	LED_PIN = 0;
	BCF	_PORTBbits, 4
;	.line	559; meter_logger.c	led_flash.state = LED_FLASH_STOPPED;
	MOVLW	0x02
; removed redundant BANKSEL
	MOVWF	_led_flash, B
_00531_DS_:
	BANKSEL	_timer_1_ms
;	.line	563; meter_logger.c	timer_1_ms++;
	INCF	_timer_1_ms, F, B
	BNC	_00557_DS_
; removed redundant BANKSEL
	INCF	(_timer_1_ms + 1), F, B
	BNC	_00557_DS_
; removed redundant BANKSEL
	INCFSZ	(_timer_1_ms + 2), F, B
	BRA	_30847_DS_
; removed redundant BANKSEL
	INCF	(_timer_1_ms + 3), F, B
_30847_DS_:
_00557_DS_:
;	.line	564; meter_logger.c	PIR1bits.TMR1IF = 0;    /* Clear the Timer Flag  */
	BCF	_PIR1bits, 0
_00533_DS_:
;	.line	568; meter_logger.c	if (usart_drdy()) {
	CALL	_usart_drdy
	MOVWF	r0x00
	MOVF	r0x00, W
	BZ	_00536_DS_
;	.line	570; meter_logger.c	c = usart_getc();
	CALL	_usart_getc
	MOVWF	r0x00
;	.line	571; meter_logger.c	usart_putc(c);
	MOVF	r0x00, W
	CALL	_usart_putc
_00536_DS_:
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
;	.line	236; meter_logger.c	static void isr_high_prio(void) __interrupt 1 {
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
;	.line	238; meter_logger.c	if (INTCONbits.INT0IF && INTCONbits.INT0IE) {
	BTFSS	_INTCONbits, 1
	BRA	_00235_DS_
	BTFSS	_INTCONbits, 4
	BRA	_00235_DS_
;	.line	239; meter_logger.c	timer_0 = (unsigned int)(TMR0L) | ((unsigned int)(TMR0H) << 8);
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
;	.line	240; meter_logger.c	TMR0H = (unsigned char)(timer0_reload >> 8);
	MOVF	(_timer0_reload + 1), W, B
	MOVWF	r0x00
	CLRF	r0x01
	MOVF	r0x00, W
	MOVWF	_TMR0H
; removed redundant BANKSEL
;	.line	241; meter_logger.c	TMR0L = (unsigned char)timer0_reload;
	MOVF	_timer0_reload, W, B
	MOVWF	_TMR0L
	BANKSEL	_codec_type
;	.line	243; meter_logger.c	switch (codec_type) {
	MOVF	_codec_type, W, B
	XORLW	0x01
	BZ	_00453_DS_
	BRA	_00233_DS_
_00453_DS_:
;	.line	245; meter_logger.c	flash_led(100);
	MOVLW	0x64
	MOVWF	POSTDEC1
	CALL	_flash_led
	MOVF	POSTINC1, F
;	.line	246; meter_logger.c	switch (testo_ir_proto.state) {
	MOVFF	_testo_ir_proto, r0x00
	MOVF	r0x00, W
	MOVWF	r0x01
	MOVF	r0x01, W
	BZ	_00199_DS_
	MOVF	r0x00, W
	XORLW	0x02
	BZ	_00200_DS_
	MOVF	r0x00, W
	XORLW	0x04
	BNZ	_00459_DS_
	BRA	_00208_DS_
_00459_DS_:
	BRA	_00233_DS_
_00199_DS_:
;	.line	248; meter_logger.c	T0CONbits.TMR0ON = 1;		// Start TMR0
	BSF	_T0CONbits, 7
;	.line	249; meter_logger.c	testo_ir_proto.start_bit_len = 1;
	MOVLW	0x01
	BANKSEL	(_testo_ir_proto + 2)
	MOVWF	(_testo_ir_proto + 2), B
;	.line	250; meter_logger.c	testo_ir_proto.state = START_BIT_WAIT;
	MOVLW	0x02
; removed redundant BANKSEL
	MOVWF	_testo_ir_proto, B
;	.line	251; meter_logger.c	break;
	BRA	_00233_DS_
_00200_DS_:
	BANKSEL	_timer0_reload
;	.line	253; meter_logger.c	if ((TICK + timer0_reload - TICK_ADJ < timer_0) && (timer_0 < TICK + timer0_reload + TICK_ADJ)) {
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
	BNZ	_00460_DS_
; removed redundant BANKSEL
	MOVF	_timer_0, W, B
	SUBWF	r0x00, W
_00460_DS_:
	BC	_00205_DS_
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
	BNZ	_00461_DS_
	MOVF	r0x00, W
; removed redundant BANKSEL
	SUBWF	_timer_0, W, B
_00461_DS_:
	BC	_00205_DS_
;	.line	254; meter_logger.c	if (testo_ir_proto.start_bit_len < 2) {
	MOVLW	0x02
	BANKSEL	(_testo_ir_proto + 2)
	SUBWF	(_testo_ir_proto + 2), W, B
	BC	_00202_DS_
;	.line	255; meter_logger.c	testo_ir_proto.start_bit_len++;
	MOVFF	(_testo_ir_proto + 2), r0x00
	INCF	r0x00, F
	MOVF	r0x00, W
; removed redundant BANKSEL
	MOVWF	(_testo_ir_proto + 2), B
	BRA	_00233_DS_
_00202_DS_:
	BANKSEL	(_testo_ir_proto + 3)
;	.line	259; meter_logger.c	testo_ir_proto.data = 0;
	CLRF	(_testo_ir_proto + 3), B
; removed redundant BANKSEL
	CLRF	(_testo_ir_proto + 4), B
; removed redundant BANKSEL
;	.line	260; meter_logger.c	testo_ir_proto.data_len = 0;
	CLRF	(_testo_ir_proto + 5), B
;	.line	261; meter_logger.c	testo_ir_proto.state = DATA_WAIT;
	MOVLW	0x04
; removed redundant BANKSEL
	MOVWF	_testo_ir_proto, B
	BRA	_00233_DS_
_00205_DS_:
;	.line	266; meter_logger.c	testo_ir_proto.start_bit_len = 1;
	MOVLW	0x01
	BANKSEL	(_testo_ir_proto + 2)
	MOVWF	(_testo_ir_proto + 2), B
;	.line	267; meter_logger.c	testo_ir_proto.state = START_BIT_WAIT;
	MOVLW	0x02
; removed redundant BANKSEL
	MOVWF	_testo_ir_proto, B
;	.line	269; meter_logger.c	break;
	BRA	_00233_DS_
_00208_DS_:
;	.line	271; meter_logger.c	if (testo_ir_proto.data_len <= 12) {
	MOVLW	0x0d
	BANKSEL	(_testo_ir_proto + 5)
	SUBWF	(_testo_ir_proto + 5), W, B
	BTFSC	STATUS, 0
	BRA	_00233_DS_
	BANKSEL	_timer0_reload
;	.line	272; meter_logger.c	if (((TICK + timer0_reload - TICK_ADJ < timer_0) && (timer_0 < TICK + timer0_reload + TICK_ADJ)) || ((3 * TICK + timer0_reload - TICK_ADJ < timer_0) && (timer_0 < 3 * TICK + timer0_reload + TICK_ADJ))) {
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
	BNZ	_00464_DS_
; removed redundant BANKSEL
	MOVF	_timer_0, W, B
	SUBWF	r0x00, W
_00464_DS_:
	BC	_00224_DS_
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
	BNZ	_00465_DS_
	MOVF	r0x00, W
; removed redundant BANKSEL
	SUBWF	_timer_0, W, B
_00465_DS_:
	BNC	_00219_DS_
_00224_DS_:
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
	BNZ	_00466_DS_
; removed redundant BANKSEL
	MOVF	_timer_0, W, B
	SUBWF	r0x00, W
_00466_DS_:
	BC	_00220_DS_
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
	BNZ	_00467_DS_
	MOVF	r0x00, W
; removed redundant BANKSEL
	SUBWF	_timer_0, W, B
_00467_DS_:
	BC	_00220_DS_
_00219_DS_:
	BANKSEL	(_testo_ir_proto + 3)
;	.line	274; meter_logger.c	if ((testo_ir_proto.data & 1) != 0) {
	BTFSS	(_testo_ir_proto + 3), 0
	BRA	_00210_DS_
; removed redundant BANKSEL
;	.line	276; meter_logger.c	testo_ir_proto.data <<= 1;		// bitshift once to left
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
	BRA	_00211_DS_
_00210_DS_:
	BANKSEL	(_testo_ir_proto + 3)
;	.line	281; meter_logger.c	testo_ir_proto.data <<= 1;		// bitshift once to left
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
;	.line	282; meter_logger.c	testo_ir_proto.data |= 1;	// and set bit 0
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
_00211_DS_:
;	.line	284; meter_logger.c	testo_ir_proto.data_len++;
	MOVFF	(_testo_ir_proto + 5), r0x00
	INCF	r0x00, F
	MOVF	r0x00, W
	BANKSEL	(_testo_ir_proto + 5)
	MOVWF	(_testo_ir_proto + 5), B
	BRA	_00221_DS_
_00220_DS_:
	BANKSEL	_timer0_reload
;	.line	286; meter_logger.c	else if ((2 * TICK + timer0_reload - TICK_ADJ < timer_0) && (timer_0 < 2 * TICK + timer0_reload + TICK_ADJ)) {
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
	BNZ	_00470_DS_
; removed redundant BANKSEL
	MOVF	_timer_0, W, B
	SUBWF	r0x00, W
_00470_DS_:
	BC	_00216_DS_
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
	BNZ	_00471_DS_
	MOVF	r0x00, W
; removed redundant BANKSEL
	SUBWF	_timer_0, W, B
_00471_DS_:
	BC	_00216_DS_
	BANKSEL	(_testo_ir_proto + 3)
;	.line	288; meter_logger.c	if ((testo_ir_proto.data & 1) != 0) {
	BTFSS	(_testo_ir_proto + 3), 0
	BRA	_00213_DS_
; removed redundant BANKSEL
;	.line	290; meter_logger.c	testo_ir_proto.data <<= 1;		// bitshift once to left
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
;	.line	291; meter_logger.c	testo_ir_proto.data |= 1;	// and set bit 0
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
	BRA	_00214_DS_
_00213_DS_:
	BANKSEL	(_testo_ir_proto + 3)
;	.line	295; meter_logger.c	testo_ir_proto.data <<= 1;		// bitshift once to left
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
_00214_DS_:
;	.line	298; meter_logger.c	testo_ir_proto.data_len++;
	MOVFF	(_testo_ir_proto + 5), r0x00
	INCF	r0x00, F
	MOVF	r0x00, W
	BANKSEL	(_testo_ir_proto + 5)
	MOVWF	(_testo_ir_proto + 5), B
	BRA	_00221_DS_
_00216_DS_:
;	.line	303; meter_logger.c	testo_ir_proto.start_bit_len = 1;
	MOVLW	0x01
	BANKSEL	(_testo_ir_proto + 2)
	MOVWF	(_testo_ir_proto + 2), B
;	.line	304; meter_logger.c	testo_ir_proto.state = START_BIT_WAIT;
	MOVLW	0x02
; removed redundant BANKSEL
	MOVWF	_testo_ir_proto, B
_00221_DS_:
	BANKSEL	(_testo_ir_proto + 5)
;	.line	306; meter_logger.c	if (testo_ir_proto.data_len == 12) {
	MOVF	(_testo_ir_proto + 5), W, B
	XORLW	0x0c
	BNZ	_00233_DS_
_00475_DS_:
	BANKSEL	(_testo_ir_proto + 4)
;	.line	309; meter_logger.c	if (valid_err_corr(testo_ir_proto.data & 0xffff)) {
	MOVF	(_testo_ir_proto + 4), W, B
	MOVWF	POSTDEC1
; removed redundant BANKSEL
	MOVF	(_testo_ir_proto + 3), W, B
	MOVWF	POSTDEC1
	CALL	_valid_err_corr
	MOVWF	r0x00
	MOVF	POSTINC1, F
	MOVF	POSTINC1, F
	MOVF	r0x00, W
	BZ	_00226_DS_
	BANKSEL	(_testo_ir_proto + 3)
;	.line	311; meter_logger.c	fifo_put(testo_ir_proto.data & 0xff);
	MOVF	(_testo_ir_proto + 3), W, B
	MOVWF	r0x00
	CLRF	r0x01
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	_fifo_put
	MOVF	POSTINC1, F
;	.line	312; meter_logger.c	LED_PIN = 1;
	BSF	_PORTBbits, 4
_00226_DS_:
	BANKSEL	_testo_ir_proto
;	.line	314; meter_logger.c	testo_ir_proto.state = INIT_STATE;
	CLRF	_testo_ir_proto, B
_00233_DS_:
;	.line	324; meter_logger.c	INTCONbits.INT0IF = 0;	/* Clear Interrupt Flag */
	BCF	_INTCONbits, 1
_00235_DS_:
;	.line	328; meter_logger.c	if (INTCONbits.TMR0IF && INTCONbits.TMR0IE) {
	BTFSS	_INTCONbits, 2
	BRA	_00277_DS_
	BTFSS	_INTCONbits, 5
	BRA	_00277_DS_
	BANKSEL	(_timer0_reload + 1)
;	.line	330; meter_logger.c	TMR0H = (unsigned char)(timer0_reload >> 8);
	MOVF	(_timer0_reload + 1), W, B
	MOVWF	r0x00
	CLRF	r0x01
	MOVF	r0x00, W
	MOVWF	_TMR0H
; removed redundant BANKSEL
;	.line	331; meter_logger.c	TMR0L = (unsigned char)timer0_reload;
	MOVF	_timer0_reload, W, B
	MOVWF	_TMR0L
	BANKSEL	_codec_type
;	.line	333; meter_logger.c	switch (codec_type) {
	MOVF	_codec_type, W, B
	XORLW	0x01
	BZ	_00237_DS_
_00478_DS_:
	BANKSEL	_codec_type
	MOVF	_codec_type, W, B
	XORLW	0x03
	BZ	_00238_DS_
_00480_DS_:
	BANKSEL	_codec_type
	MOVF	_codec_type, W, B
	XORLW	0x04
	BNZ	_00482_DS_
	BRA	_00248_DS_
_00482_DS_:
	BANKSEL	_codec_type
	MOVF	_codec_type, W, B
	XORLW	0x05
	BNZ	_00484_DS_
	BRA	_00259_DS_
_00484_DS_:
	BRA	_00275_DS_
_00237_DS_:
;	.line	335; meter_logger.c	T0CONbits.TMR0ON = 0;			// Stop TMR0
	BCF	_T0CONbits, 7
	BANKSEL	_testo_ir_proto
;	.line	336; meter_logger.c	testo_ir_proto.state = INIT_STATE;
	CLRF	_testo_ir_proto, B
	sleep 
;	.line	338; meter_logger.c	break;
	BRA	_00275_DS_
_00238_DS_:
;	.line	340; meter_logger.c	switch (rs232_ir_proto.state) {
	MOVFF	_rs232_ir_proto, r0x00
	MOVF	r0x00, W
	MOVWF	r0x01
	MOVF	r0x01, W
	BZ	_00239_DS_
	MOVF	r0x00, W
	XORLW	0x03
	BZ	_00242_DS_
	MOVF	r0x00, W
	XORLW	0x08
	BNZ	_00490_DS_
	BRA	_00246_DS_
_00490_DS_:
	BRA	_00275_DS_
_00239_DS_:
;	.line	342; meter_logger.c	if (fifo_get(&c)) {
	MOVLW	HIGH(_c)
	MOVWF	r0x01
	MOVLW	LOW(_c)
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
	BTFSC	STATUS, 2
	BRA	_00275_DS_
;	.line	343; meter_logger.c	PWM_PIN = 0;
	BCF	_PORTCbits, 1
;	.line	344; meter_logger.c	rs232_ir_proto.state = START_BIT_SENT;
	MOVLW	0x03
	BANKSEL	_rs232_ir_proto
	MOVWF	_rs232_ir_proto, B
;	.line	345; meter_logger.c	rs232_ir_proto.data_len = 8;
	MOVLW	0x08
; removed redundant BANKSEL
	MOVWF	(_rs232_ir_proto + 3), B
	BANKSEL	_c
;	.line	346; meter_logger.c	rs232_ir_proto.data = c;
	MOVF	_c, W, B
	BANKSEL	(_rs232_ir_proto + 2)
	MOVWF	(_rs232_ir_proto + 2), B
;	.line	348; meter_logger.c	break;
	BRA	_00275_DS_
_00242_DS_:
;	.line	350; meter_logger.c	if (rs232_ir_proto.data_len >= 1) {
	MOVLW	0x01
	BANKSEL	(_rs232_ir_proto + 3)
	SUBWF	(_rs232_ir_proto + 3), W, B
	BNC	_00244_DS_
;	.line	351; meter_logger.c	PWM_PIN = (rs232_ir_proto.data & 1) != 0;
	MOVLW	0x01
; removed redundant BANKSEL
	ANDWF	(_rs232_ir_proto + 2), W, B
	MOVWF	r0x00
	MOVF	r0x00, W
	ANDLW	0x01
	RLNCF	WREG, W
	MOVWF	PRODH
	MOVF	_PORTCbits, W
	ANDLW	0xfd
	IORWF	PRODH, W
	MOVWF	_PORTCbits
; removed redundant BANKSEL
;	.line	352; meter_logger.c	rs232_ir_proto.data = rs232_ir_proto.data >> 1;
	RRNCF	(_rs232_ir_proto + 2), W, B
	ANDLW	0x7f
	MOVWF	r0x00
	MOVF	r0x00, W
; removed redundant BANKSEL
	MOVWF	(_rs232_ir_proto + 2), B
;	.line	353; meter_logger.c	rs232_ir_proto.data_len--;
	MOVFF	(_rs232_ir_proto + 3), r0x00
	DECF	r0x00, F
	MOVF	r0x00, W
; removed redundant BANKSEL
	MOVWF	(_rs232_ir_proto + 3), B
	BRA	_00275_DS_
_00244_DS_:
;	.line	356; meter_logger.c	PWM_PIN = 1;							
	BSF	_PORTCbits, 1
;	.line	357; meter_logger.c	rs232_ir_proto.state = STOP_BIT_SENT;
	MOVLW	0x08
	BANKSEL	_rs232_ir_proto
	MOVWF	_rs232_ir_proto, B
;	.line	359; meter_logger.c	break;
	BRA	_00275_DS_
_00246_DS_:
;	.line	361; meter_logger.c	PWM_PIN = 1;
	BSF	_PORTCbits, 1
	BANKSEL	_rs232_ir_proto
;	.line	362; meter_logger.c	rs232_ir_proto.state = INIT_STATE;
	CLRF	_rs232_ir_proto, B
;	.line	365; meter_logger.c	break;
	BRA	_00275_DS_
_00248_DS_:
;	.line	367; meter_logger.c	switch (fsk_proto.state) {
	MOVFF	_fsk_proto, r0x00
	MOVF	r0x00, W
	XORLW	0x04
	BZ	_00249_DS_
	MOVF	r0x00, W
	XORLW	0x07
	BZ	_00257_DS_
	BRA	_00275_DS_
_00249_DS_:
;	.line	369; meter_logger.c	fsk_proto.data_len++;						
	MOVFF	(_fsk_proto + 13), r0x00
	INCF	r0x00, F
	MOVF	r0x00, W
	BANKSEL	(_fsk_proto + 13)
	MOVWF	(_fsk_proto + 13), B
;	.line	370; meter_logger.c	if (fsk_proto.data_len <= 8) {
	MOVLW	0x09
; removed redundant BANKSEL
	SUBWF	(_fsk_proto + 13), W, B
	BC	_00255_DS_
;	.line	371; meter_logger.c	if ((fsk_proto.diff > 340) && (fsk_proto.diff < 476)) {
	MOVLW	0x01
; removed redundant BANKSEL
	SUBWF	(_fsk_proto + 2), W, B
	BNZ	_00498_DS_
	MOVLW	0x55
; removed redundant BANKSEL
	SUBWF	(_fsk_proto + 1), W, B
_00498_DS_:
	BNC	_00251_DS_
	MOVLW	0x01
	BANKSEL	(_fsk_proto + 2)
	SUBWF	(_fsk_proto + 2), W, B
	BNZ	_00499_DS_
	MOVLW	0xdc
; removed redundant BANKSEL
	SUBWF	(_fsk_proto + 1), W, B
_00499_DS_:
	BC	_00251_DS_
	BANKSEL	(_fsk_proto + 12)
;	.line	374; meter_logger.c	fsk_proto.data >>= 1;
	RRNCF	(_fsk_proto + 12), W, B
	ANDLW	0x7f
	MOVWF	r0x00
	MOVF	r0x00, W
; removed redundant BANKSEL
	MOVWF	(_fsk_proto + 12), B
	BRA	_00275_DS_
_00251_DS_:
	BANKSEL	(_fsk_proto + 12)
;	.line	386; meter_logger.c	fsk_proto.data >>= 1;
	RRNCF	(_fsk_proto + 12), W, B
	ANDLW	0x7f
	MOVWF	r0x00
	MOVF	r0x00, W
; removed redundant BANKSEL
	MOVWF	(_fsk_proto + 12), B
;	.line	387; meter_logger.c	fsk_proto.data |= 0x80;
	MOVLW	0x80
; removed redundant BANKSEL
	IORWF	(_fsk_proto + 12), W, B
	MOVWF	r0x00
	MOVF	r0x00, W
; removed redundant BANKSEL
	MOVWF	(_fsk_proto + 12), B
	BRA	_00275_DS_
_00255_DS_:
;	.line	402; meter_logger.c	fsk_proto.state = STOP_BIT_WAIT;
	MOVLW	0x07
	BANKSEL	_fsk_proto
	MOVWF	_fsk_proto, B
;	.line	404; meter_logger.c	break;
	BRA	_00275_DS_
_00257_DS_:
	BANKSEL	(_fsk_proto + 12)
;	.line	407; meter_logger.c	fifo_put(fsk_proto.data);
	MOVF	(_fsk_proto + 12), W, B
	MOVWF	POSTDEC1
	CALL	_fifo_put
	MOVF	POSTINC1, F
	BANKSEL	(_fsk_proto + 12)
;	.line	408; meter_logger.c	fsk_proto.data = 0;
	CLRF	(_fsk_proto + 12), B
;	.line	409; meter_logger.c	fsk_proto.state = START_BIT_WAIT;
	MOVLW	0x02
; removed redundant BANKSEL
	MOVWF	_fsk_proto, B
;	.line	411; meter_logger.c	INTCONbits.TMR0IE = 0;						
	BCF	_INTCONbits, 5
;	.line	414; meter_logger.c	break;
	BRA	_00275_DS_
_00259_DS_:
;	.line	416; meter_logger.c	switch (fsk_proto.state) {
	MOVFF	_fsk_proto, r0x00
	MOVF	r0x00, W
	MOVWF	r0x01
	MOVF	r0x01, W
	BZ	_00260_DS_
	MOVF	r0x00, W
	XORLW	0x01
	BZ	_00263_DS_
	MOVF	r0x00, W
	XORLW	0x03
	BZ	_00264_DS_
	MOVF	r0x00, W
	XORLW	0x05
	BNZ	_00508_DS_
	BRA	_00272_DS_
_00508_DS_:
	MOVF	r0x00, W
	XORLW	0x08
	BNZ	_00510_DS_
	BRA	_00273_DS_
_00510_DS_:
	BRA	_00275_DS_
_00260_DS_:
;	.line	418; meter_logger.c	send_fsk_high();
	CALL	_send_fsk_high
	BANKSEL	(_fsk_proto + 13)
;	.line	419; meter_logger.c	if (fsk_proto.data_len == 8) {//fifo_get(&fsk_proto.data)) {
	MOVF	(_fsk_proto + 13), W, B
	XORLW	0x08
	BNZ	_00262_DS_
;	.line	421; meter_logger.c	fsk_proto.state = IDLE;
	MOVLW	0x01
	BANKSEL	_fsk_proto
	MOVWF	_fsk_proto, B
_00262_DS_:
;	.line	424; meter_logger.c	DEBUG_PIN = 0;
	BCF	_PORTBbits, 1
;	.line	426; meter_logger.c	break;
	BRA	_00275_DS_
_00263_DS_:
;	.line	428; meter_logger.c	send_fsk_low();
	CALL	_send_fsk_low
;	.line	429; meter_logger.c	fsk_proto.state = START_BIT_SENT;
	MOVLW	0x03
	BANKSEL	_fsk_proto
	MOVWF	_fsk_proto, B
;	.line	431; meter_logger.c	DEBUG_PIN = 1;
	BSF	_PORTBbits, 1
;	.line	433; meter_logger.c	break;
	BRA	_00275_DS_
_00264_DS_:
;	.line	435; meter_logger.c	if (fsk_proto.data_len--) {
	MOVFF	(_fsk_proto + 13), r0x00
	DECF	r0x00, W
	MOVWF	r0x01
	MOVF	r0x01, W
	BANKSEL	(_fsk_proto + 13)
	MOVWF	(_fsk_proto + 13), B
	MOVF	r0x00, W
	BZ	_00269_DS_
;	.line	436; meter_logger.c	if (fsk_proto.data & (0x80 >> fsk_proto.data_len)) {
	MOVLW	0x80
	MOVWF	r0x00
; removed redundant BANKSEL
	MOVF	(_fsk_proto + 13), W, B
	BZ	_00513_DS_
	NEGF	WREG
	BCF	STATUS, 0
_00514_DS_:
	RRCF	r0x00, F
	ADDLW	0x01
	BNC	_00514_DS_
_00513_DS_:
	BANKSEL	(_fsk_proto + 12)
	MOVF	(_fsk_proto + 12), W, B
	ANDWF	r0x00, F
	MOVF	r0x00, W
	BZ	_00266_DS_
;	.line	444; meter_logger.c	send_fsk_high();
	CALL	_send_fsk_high
;	.line	446; meter_logger.c	DEBUG_PIN = 0;
	BCF	_PORTBbits, 1
	BRA	_00269_DS_
_00266_DS_:
;	.line	457; meter_logger.c	send_fsk_low();
	CALL	_send_fsk_low
;	.line	459; meter_logger.c	DEBUG_PIN = 1;
	BSF	_PORTBbits, 1
_00269_DS_:
	BANKSEL	(_fsk_proto + 13)
;	.line	463; meter_logger.c	if (fsk_proto.data_len == 0) {
	MOVF	(_fsk_proto + 13), W, B
	BNZ	_00275_DS_
;	.line	464; meter_logger.c	fsk_proto.state = DATA_SENT;
	MOVLW	0x05
; removed redundant BANKSEL
	MOVWF	_fsk_proto, B
;	.line	466; meter_logger.c	break;
	BRA	_00275_DS_
_00272_DS_:
;	.line	468; meter_logger.c	send_fsk_high();
	CALL	_send_fsk_high
;	.line	469; meter_logger.c	fsk_proto.state = STOP_BIT_SENT;
	MOVLW	0x08
	BANKSEL	_fsk_proto
	MOVWF	_fsk_proto, B
;	.line	471; meter_logger.c	DEBUG_PIN = 0;
	BCF	_PORTBbits, 1
;	.line	473; meter_logger.c	break;
	BRA	_00275_DS_
_00273_DS_:
;	.line	475; meter_logger.c	send_fsk_high();
	CALL	_send_fsk_high
	BANKSEL	_fsk_proto
;	.line	476; meter_logger.c	fsk_proto.state = INIT_STATE;
	CLRF	_fsk_proto, B
;	.line	478; meter_logger.c	DEBUG_PIN = 0;
	BCF	_PORTBbits, 1
_00275_DS_:
;	.line	485; meter_logger.c	INTCONbits.TMR0IF = 0;
	BCF	_INTCONbits, 2
_00277_DS_:
;	.line	488; meter_logger.c	if (PIR2bits.CMIF && PIE2bits.CMIE) {
	BTFSS	_PIR2bits, 6
	BRA	_00296_DS_
	BTFSS	_PIE2bits, 6
	BRA	_00296_DS_
;	.line	490; meter_logger.c	if (CMCONbits.C1OUT) {		// rising edge
	BTFSS	_CMCONbits, 6
	BRA	_00291_DS_
;	.line	491; meter_logger.c	timer_0 = (unsigned int)(TMR0L) | ((unsigned int)(TMR0H) << 8);
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
;	.line	496; meter_logger.c	DEBUG_PIN = 1;
	BSF	_PORTBbits, 1
	BANKSEL	_last_timer_0
;	.line	498; meter_logger.c	fsk_proto.diff = timer_0 - last_timer_0;
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
;	.line	499; meter_logger.c	last_timer_0 = timer_0;
	MOVFF	_timer_0, _last_timer_0
	MOVFF	(_timer_0 + 1), (_last_timer_0 + 1)
;	.line	501; meter_logger.c	if ((fsk_proto.diff > 340) && (fsk_proto.diff < 476)) {
	MOVLW	0x01
; removed redundant BANKSEL
	SUBWF	(_fsk_proto + 2), W, B
	BNZ	_00516_DS_
	MOVLW	0x55
; removed redundant BANKSEL
	SUBWF	(_fsk_proto + 1), W, B
_00516_DS_:
	BNC	_00287_DS_
	MOVLW	0x01
	BANKSEL	(_fsk_proto + 2)
	SUBWF	(_fsk_proto + 2), W, B
	BNZ	_00517_DS_
	MOVLW	0xdc
; removed redundant BANKSEL
	SUBWF	(_fsk_proto + 1), W, B
_00517_DS_:
	BC	_00287_DS_
	BANKSEL	(_fsk_proto + 1)
;	.line	502; meter_logger.c	fsk_proto.low_count += fsk_proto.diff;
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
;	.line	503; meter_logger.c	if (fsk_proto.state == START_BIT_WAIT) {
	MOVF	_fsk_proto, W, B
	XORLW	0x02
	BNZ	_00292_DS_
;	.line	504; meter_logger.c	if (fsk_proto.low_count >= 800) {								// start bit received
	MOVLW	0x03
	BANKSEL	(_fsk_proto + 6)
	SUBWF	(_fsk_proto + 6), W, B
	BNZ	_00520_DS_
	MOVLW	0x20
; removed redundant BANKSEL
	SUBWF	(_fsk_proto + 5), W, B
_00520_DS_:
	BNC	_00292_DS_
	BANKSEL	(_timer0_reload + 1)
;	.line	506; meter_logger.c	TMR0H = (unsigned char)(timer0_reload >> 8);
	MOVF	(_timer0_reload + 1), W, B
	MOVWF	r0x00
	CLRF	r0x01
	MOVF	r0x00, W
	MOVWF	_TMR0H
; removed redundant BANKSEL
;	.line	507; meter_logger.c	TMR0L = (unsigned char)timer0_reload;
	MOVF	_timer0_reload, W, B
	MOVWF	_TMR0L
	BANKSEL	(_fsk_proto + 5)
;	.line	508; meter_logger.c	fsk_proto.low_count = 0;
	CLRF	(_fsk_proto + 5), B
; removed redundant BANKSEL
	CLRF	(_fsk_proto + 6), B
; removed redundant BANKSEL
;	.line	509; meter_logger.c	fsk_proto.high_count = 0;
	CLRF	(_fsk_proto + 7), B
; removed redundant BANKSEL
	CLRF	(_fsk_proto + 8), B
; removed redundant BANKSEL
;	.line	511; meter_logger.c	fsk_proto.data_len = 0;
	CLRF	(_fsk_proto + 13), B
; removed redundant BANKSEL
;	.line	512; meter_logger.c	fsk_proto.data = 0;
	CLRF	(_fsk_proto + 12), B
;	.line	513; meter_logger.c	fsk_proto.state = DATA_WAIT;
	MOVLW	0x04
; removed redundant BANKSEL
	MOVWF	_fsk_proto, B
;	.line	514; meter_logger.c	INTCONbits.TMR0IF = 0;		// clear flag so it dont enter isr now
	BCF	_INTCONbits, 2
;	.line	515; meter_logger.c	INTCONbits.TMR0IE = 1;		// Enable TMR0 Interrupt
	BSF	_INTCONbits, 5
	BRA	_00292_DS_
_00287_DS_:
	BANKSEL	_fsk_proto
;	.line	521; meter_logger.c	if (fsk_proto.state == START_BIT_WAIT) {
	MOVF	_fsk_proto, W, B
	XORLW	0x02
	BNZ	_00284_DS_
_00522_DS_:
	BANKSEL	(_fsk_proto + 5)
;	.line	522; meter_logger.c	fsk_proto.low_count = 0;
	CLRF	(_fsk_proto + 5), B
; removed redundant BANKSEL
	CLRF	(_fsk_proto + 6), B
; removed redundant BANKSEL
;	.line	523; meter_logger.c	fsk_proto.high_count = 0;
	CLRF	(_fsk_proto + 7), B
; removed redundant BANKSEL
	CLRF	(_fsk_proto + 8), B
	BRA	_00292_DS_
_00284_DS_:
	BANKSEL	(_fsk_proto + 1)
;	.line	526; meter_logger.c	fsk_proto.high_count += fsk_proto.diff;
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
	BRA	_00292_DS_
_00291_DS_:
;	.line	532; meter_logger.c	DEBUG_PIN = 0;
	BCF	_PORTBbits, 1
_00292_DS_:
;	.line	536; meter_logger.c	PIR2bits.CMIF = 0;
	BCF	_PIR2bits, 6
_00296_DS_:
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
	DB	0x70, 0x72, 0x65, 0x73, 0x73, 0x20, 0x70, 0x72, 0x69, 0x6e, 0x74, 0x20
	DB	0x6f, 0x6e, 0x20, 0x74, 0x65, 0x73, 0x74, 0x6f, 0x0a, 0x00
; ; Starting pCode block
___str_1:
	DB	0x64, 0x6f, 0x6e, 0x65, 0x20, 0x72, 0x65, 0x63, 0x65, 0x69, 0x76, 0x69
	DB	0x6e, 0x67, 0x20, 0x2d, 0x20, 0x73, 0x65, 0x6e, 0x64, 0x69, 0x6e, 0x67
	DB	0x20, 0x76, 0x69, 0x61, 0x20, 0x73, 0x65, 0x72, 0x69, 0x61, 0x6c, 0x2f
	DB	0x66, 0x73, 0x6b, 0x0a, 0x00
; ; Starting pCode block
___str_2:
	DB	0x77, 0x61, 0x69, 0x74, 0x69, 0x6e, 0x67, 0x20, 0x66, 0x6f, 0x72, 0x20
	DB	0x6e, 0x65, 0x77, 0x20, 0x63, 0x6f, 0x6d, 0x6d, 0x61, 0x6e, 0x64, 0x0a
	DB	0x00
; ; Starting pCode block
___str_3:
	DB	0x65, 0x63, 0x68, 0x6f, 0x20, 0x74, 0x65, 0x73, 0x74, 0x20, 0x2d, 0x20
	DB	0x73, 0x65, 0x6e, 0x64, 0x20, 0x73, 0x6f, 0x6d, 0x65, 0x20, 0x64, 0x61
	DB	0x74, 0x61, 0x0a, 0x00
; ; Starting pCode block
___str_4:
	DB	0x0a, 0x77, 0x61, 0x69, 0x74, 0x69, 0x6e, 0x67, 0x20, 0x66, 0x6f, 0x72
	DB	0x20, 0x6e, 0x65, 0x77, 0x20, 0x63, 0x6f, 0x6d, 0x6d, 0x61, 0x6e, 0x64
	DB	0x0a, 0x00
; ; Starting pCode block
___str_5:
	DB	0x6b, 0x61, 0x6d, 0x73, 0x74, 0x72, 0x75, 0x70, 0x20, 0x2d, 0x20, 0x73
	DB	0x65, 0x6e, 0x64, 0x20, 0x73, 0x6f, 0x6d, 0x65, 0x20, 0x64, 0x61, 0x74
	DB	0x61, 0x0a, 0x00


; Statistics:
; code size:	11686 (0x2da6) bytes ( 8.92%)
;           	 5843 (0x16d3) words
; udata size:	 1136 (0x0470) bytes (63.39%)
; access size:	   16 (0x0010) bytes


	end
