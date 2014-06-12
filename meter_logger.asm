;--------------------------------------------------------
; File Created by SDCC : free open source ANSI-C Compiler
; Version 3.4.0 #8981 (Jun  6 2014) (Mac OS X x86_64)
; This file was generated Thu Jun 12 17:03:36 2014
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
	global	_debug_buffer
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
	extern	_sprintf
	extern	_usart_drdy
	extern	_usart_getc
	extern	_usart_putc
	extern	_usart_puts

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

udata_meter_logger_0	udata
_c	res	1

udata_meter_logger_1	udata
_timer_1_ms	res	2

udata_meter_logger_2	udata
_fifo_head	res	2

udata_meter_logger_3	udata
_fifo_tail	res	2

udata_meter_logger_4	udata
_main_cmd_1_79	res	1

udata_meter_logger_5	udata
_debug_buffer	res	128

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
;	.line	121; meter_logger.c	usart_puts("\n\rMeterLogger... serial working\n\r");
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
;	.line	123; meter_logger.c	sleep_ms(100);
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x64
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVF	POSTINC1, F
	MOVF	POSTINC1, F
;	.line	125; meter_logger.c	fsk_rx_enable();
	CALL	_fsk_rx_enable
_00143_DS_:
;	.line	127; meter_logger.c	if (fifo_get(&cmd)) {
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
	BZ	_00143_DS_
;	.line	128; meter_logger.c	switch (cmd) {
	MOVLW	0xfd
	BANKSEL	_main_cmd_1_79
	SUBWF	_main_cmd_1_79, W, B
	BNC	_00143_DS_
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
	ADDLW	LOW(_00248_DS_)
	MOVWF	POSTDEC1
	MOVLW	HIGH(_00248_DS_)
	ADDWFC	PCLATH, F
	MOVLW	UPPER(_00248_DS_)
	ADDWFC	PCLATU, F
	MOVF	PREINC1, W
	MOVWF	PCL
_00248_DS_:
	GOTO	_00121_DS_
	GOTO	_00105_DS_
	GOTO	_00113_DS_
_00105_DS_:
;	.line	130; meter_logger.c	fsk_rx_disable();
	CALL	_fsk_rx_disable
;	.line	131; meter_logger.c	usart_puts("\n\rpress print on testo\n\r");
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
;	.line	132; meter_logger.c	testo_ir_enable();
	CALL	_testo_ir_enable
;	.line	134; meter_logger.c	last_fifo_size = 0;
	CLRF	r0x00
	CLRF	r0x01
;	.line	135; meter_logger.c	sleep_ms(10000);						// 10 seconds to start printing
	MOVLW	0x27
	MOVWF	POSTDEC1
	MOVLW	0x10
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVF	POSTINC1, F
	MOVF	POSTINC1, F
;	.line	136; meter_logger.c	fifo_size = fifo_in_use();
	CALL	_fifo_in_use
	MOVWF	r0x02
	MOVFF	PRODL, r0x03
_00106_DS_:
;	.line	137; meter_logger.c	while (fifo_size > last_fifo_size) {	// and wait while we are still receiving data
	MOVF	r0x03, W
	SUBWF	r0x01, W
	BNZ	_00249_DS_
	MOVF	r0x02, W
	SUBWF	r0x00, W
_00249_DS_:
	BC	_00108_DS_
;	.line	138; meter_logger.c	last_fifo_size = fifo_size;
	MOVFF	r0x02, r0x00
	MOVFF	r0x03, r0x01
;	.line	139; meter_logger.c	sleep_ms(200);						// return data when no data for 200 ms
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0xc8
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVF	POSTINC1, F
	MOVF	POSTINC1, F
;	.line	140; meter_logger.c	fifo_size = fifo_in_use();
	CALL	_fifo_in_use
	MOVWF	r0x02
	MOVFF	PRODL, r0x03
	BRA	_00106_DS_
_00108_DS_:
;	.line	142; meter_logger.c	testo_ir_disable();
	CALL	_testo_ir_disable
;	.line	145; meter_logger.c	usart_puts("\n\rdone receiving - sending via serial/fsk\n\r");
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
;	.line	148; meter_logger.c	for (i = 0; i < fifo_in_use(); i++) {
	CLRF	r0x04
	CLRF	r0x05
_00146_DS_:
	CALL	_fifo_in_use
	MOVWF	r0x06
	MOVFF	PRODL, r0x07
	MOVF	r0x07, W
	SUBWF	r0x05, W
	BNZ	_00250_DS_
	MOVF	r0x06, W
	SUBWF	r0x04, W
_00250_DS_:
	BTFSC	STATUS, 0
	BRA	_00109_DS_
;	.line	150; meter_logger.c	fifo_get(&sub_cmd);
	MOVLW	HIGH(_main_sub_cmd_1_79)
	MOVWF	r0x07
	MOVLW	LOW(_main_sub_cmd_1_79)
	MOVWF	r0x06
	MOVLW	0x80
	MOVWF	r0x08
	MOVF	r0x08, W
	MOVWF	POSTDEC1
	MOVF	r0x07, W
	MOVWF	POSTDEC1
	MOVF	r0x06, W
	MOVWF	POSTDEC1
	CALL	_fifo_get
	MOVLW	0x03
	ADDWF	FSR1L, F
;	.line	152; meter_logger.c	sprintf(debug_buffer, "%d ", sub_cmd);
	MOVFF	_main_sub_cmd_1_79, r0x06
	CLRF	r0x07
	MOVLW	UPPER(___str_3)
	MOVWF	r0x0a
	MOVLW	HIGH(___str_3)
	MOVWF	r0x09
	MOVLW	LOW(___str_3)
	MOVWF	r0x08
	MOVLW	HIGH(_debug_buffer)
	MOVWF	r0x0c
	MOVLW	LOW(_debug_buffer)
	MOVWF	r0x0b
	MOVLW	0x80
	MOVWF	r0x0d
	MOVF	r0x07, W
	MOVWF	POSTDEC1
	MOVF	r0x06, W
	MOVWF	POSTDEC1
	MOVF	r0x0a, W
	MOVWF	POSTDEC1
	MOVF	r0x09, W
	MOVWF	POSTDEC1
	MOVF	r0x08, W
	MOVWF	POSTDEC1
	MOVF	r0x0d, W
	MOVWF	POSTDEC1
	MOVF	r0x0c, W
	MOVWF	POSTDEC1
	MOVF	r0x0b, W
	MOVWF	POSTDEC1
	CALL	_sprintf
	MOVLW	0x08
	ADDWF	FSR1L, F
;	.line	153; meter_logger.c	usart_puts(debug_buffer);
	MOVLW	HIGH(_debug_buffer)
	MOVWF	r0x07
	MOVLW	LOW(_debug_buffer)
	MOVWF	r0x06
	MOVLW	0x80
	MOVWF	r0x08
	MOVF	r0x08, W
	MOVWF	POSTDEC1
	MOVF	r0x07, W
	MOVWF	POSTDEC1
	MOVF	r0x06, W
	MOVWF	POSTDEC1
	CALL	_usart_puts
	MOVLW	0x03
	ADDWF	FSR1L, F
	BANKSEL	_main_sub_cmd_1_79
;	.line	155; meter_logger.c	fifo_put(sub_cmd);
	MOVF	_main_sub_cmd_1_79, W, B
	MOVWF	POSTDEC1
	CALL	_fifo_put
	MOVF	POSTINC1, F
;	.line	148; meter_logger.c	for (i = 0; i < fifo_in_use(); i++) {
	INFSNZ	r0x04, F
	INCF	r0x05, F
	BRA	_00146_DS_
_00109_DS_:
;	.line	158; meter_logger.c	fsk_tx_enable();
	CALL	_fsk_tx_enable
;	.line	159; meter_logger.c	sleep_ms(FSK_TX_SLEEP);
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x0c
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVF	POSTINC1, F
	MOVF	POSTINC1, F
_00110_DS_:
;	.line	160; meter_logger.c	while (fifo_get(&cmd)) {	// and print them to serial
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
	BZ	_00112_DS_
	BANKSEL	_main_cmd_1_79
;	.line	161; meter_logger.c	fsk_tx_byte(cmd);
	MOVF	_main_cmd_1_79, W, B
	MOVWF	POSTDEC1
	CALL	_fsk_tx_byte
	MOVF	POSTINC1, F
;	.line	162; meter_logger.c	sleep_ms(FSK_TX_SLEEP);
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x0c
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVF	POSTINC1, F
	MOVF	POSTINC1, F
	BRA	_00110_DS_
_00112_DS_:
;	.line	164; meter_logger.c	fsk_tx_disable();
	CALL	_fsk_tx_disable
;	.line	166; meter_logger.c	usart_puts("\n\rwaiting for new command\n\r");
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
;	.line	168; meter_logger.c	fsk_rx_enable();
	CALL	_fsk_rx_enable
;	.line	169; meter_logger.c	break;
	BRA	_00143_DS_
_00113_DS_:
;	.line	173; meter_logger.c	fsk_rx_disable();
	CALL	_fsk_rx_disable
;	.line	174; meter_logger.c	usart_puts("\n\recho test - send some data\n\r");
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
;	.line	175; meter_logger.c	fsk_rx_enable();
	CALL	_fsk_rx_enable
;	.line	179; meter_logger.c	last_fifo_size = 0;
	CLRF	r0x00
	CLRF	r0x01
;	.line	180; meter_logger.c	sleep_ms(1000);							// 1 second
	MOVLW	0x03
	MOVWF	POSTDEC1
	MOVLW	0xe8
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVF	POSTINC1, F
	MOVF	POSTINC1, F
;	.line	181; meter_logger.c	fifo_size = fifo_in_use();
	CALL	_fifo_in_use
	MOVWF	r0x02
	MOVFF	PRODL, r0x03
_00114_DS_:
;	.line	182; meter_logger.c	while (fifo_size > last_fifo_size) {	// and wait while we are still receiving data
	MOVF	r0x03, W
	SUBWF	r0x01, W
	BNZ	_00251_DS_
	MOVF	r0x02, W
	SUBWF	r0x00, W
_00251_DS_:
	BC	_00116_DS_
;	.line	183; meter_logger.c	last_fifo_size = fifo_size;
	MOVFF	r0x02, r0x00
	MOVFF	r0x03, r0x01
;	.line	184; meter_logger.c	sleep_ms(500);						// return data when no data for 500 ms
	MOVLW	0x01
	MOVWF	POSTDEC1
	MOVLW	0xf4
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVF	POSTINC1, F
	MOVF	POSTINC1, F
;	.line	185; meter_logger.c	fifo_size = fifo_in_use();
	CALL	_fifo_in_use
	MOVWF	r0x02
	MOVFF	PRODL, r0x03
	BRA	_00114_DS_
_00116_DS_:
;	.line	187; meter_logger.c	fsk_rx_disable();
	CALL	_fsk_rx_disable
;	.line	190; meter_logger.c	for (i = 0; i < fifo_in_use(); i++) {
	CLRF	r0x04
	CLRF	r0x05
_00149_DS_:
	CALL	_fifo_in_use
	MOVWF	r0x06
	MOVFF	PRODL, r0x07
	MOVF	r0x07, W
	SUBWF	r0x05, W
	BNZ	_00252_DS_
	MOVF	r0x06, W
	SUBWF	r0x04, W
_00252_DS_:
	BTFSC	STATUS, 0
	BRA	_00117_DS_
;	.line	192; meter_logger.c	fifo_get(&sub_cmd);
	MOVLW	HIGH(_main_sub_cmd_1_79)
	MOVWF	r0x07
	MOVLW	LOW(_main_sub_cmd_1_79)
	MOVWF	r0x06
	MOVLW	0x80
	MOVWF	r0x08
	MOVF	r0x08, W
	MOVWF	POSTDEC1
	MOVF	r0x07, W
	MOVWF	POSTDEC1
	MOVF	r0x06, W
	MOVWF	POSTDEC1
	CALL	_fifo_get
	MOVLW	0x03
	ADDWF	FSR1L, F
;	.line	194; meter_logger.c	sprintf(debug_buffer, "%d ", sub_cmd);
	MOVFF	_main_sub_cmd_1_79, r0x06
	CLRF	r0x07
	MOVLW	UPPER(___str_3)
	MOVWF	r0x0a
	MOVLW	HIGH(___str_3)
	MOVWF	r0x09
	MOVLW	LOW(___str_3)
	MOVWF	r0x08
	MOVLW	HIGH(_debug_buffer)
	MOVWF	r0x0c
	MOVLW	LOW(_debug_buffer)
	MOVWF	r0x0b
	MOVLW	0x80
	MOVWF	r0x0d
	MOVF	r0x07, W
	MOVWF	POSTDEC1
	MOVF	r0x06, W
	MOVWF	POSTDEC1
	MOVF	r0x0a, W
	MOVWF	POSTDEC1
	MOVF	r0x09, W
	MOVWF	POSTDEC1
	MOVF	r0x08, W
	MOVWF	POSTDEC1
	MOVF	r0x0d, W
	MOVWF	POSTDEC1
	MOVF	r0x0c, W
	MOVWF	POSTDEC1
	MOVF	r0x0b, W
	MOVWF	POSTDEC1
	CALL	_sprintf
	MOVLW	0x08
	ADDWF	FSR1L, F
;	.line	195; meter_logger.c	usart_puts(debug_buffer);
	MOVLW	HIGH(_debug_buffer)
	MOVWF	r0x07
	MOVLW	LOW(_debug_buffer)
	MOVWF	r0x06
	MOVLW	0x80
	MOVWF	r0x08
	MOVF	r0x08, W
	MOVWF	POSTDEC1
	MOVF	r0x07, W
	MOVWF	POSTDEC1
	MOVF	r0x06, W
	MOVWF	POSTDEC1
	CALL	_usart_puts
	MOVLW	0x03
	ADDWF	FSR1L, F
	BANKSEL	_main_sub_cmd_1_79
;	.line	197; meter_logger.c	fifo_put(sub_cmd);
	MOVF	_main_sub_cmd_1_79, W, B
	MOVWF	POSTDEC1
	CALL	_fifo_put
	MOVF	POSTINC1, F
;	.line	190; meter_logger.c	for (i = 0; i < fifo_in_use(); i++) {
	INFSNZ	r0x04, F
	INCF	r0x05, F
	BRA	_00149_DS_
_00117_DS_:
;	.line	202; meter_logger.c	fsk_tx_enable();
	CALL	_fsk_tx_enable
;	.line	203; meter_logger.c	sleep_ms(FSK_TX_SLEEP);
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x0c
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVF	POSTINC1, F
	MOVF	POSTINC1, F
_00118_DS_:
;	.line	204; meter_logger.c	while (fifo_get(&sub_cmd)) {
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
	BZ	_00120_DS_
	BANKSEL	_main_sub_cmd_1_79
;	.line	205; meter_logger.c	fsk_tx_byte(sub_cmd);
	MOVF	_main_sub_cmd_1_79, W, B
	MOVWF	POSTDEC1
	CALL	_fsk_tx_byte
	MOVF	POSTINC1, F
;	.line	206; meter_logger.c	sleep_ms(FSK_TX_SLEEP);
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x0c
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVF	POSTINC1, F
	MOVF	POSTINC1, F
	BRA	_00118_DS_
_00120_DS_:
;	.line	208; meter_logger.c	fsk_tx_disable();
	CALL	_fsk_tx_disable
;	.line	211; meter_logger.c	usart_puts("\n\rwaiting for new command\n\r");
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
;	.line	213; meter_logger.c	fsk_rx_enable();
	CALL	_fsk_rx_enable
;	.line	214; meter_logger.c	break;
	BRA	_00143_DS_
_00121_DS_:
;	.line	217; meter_logger.c	fsk_rx_disable();
	CALL	_fsk_rx_disable
;	.line	219; meter_logger.c	usart_puts("\n\rkamstrup - send kmp frame data\n\r");
	MOVLW	UPPER(___str_6)
	MOVWF	r0x06
	MOVLW	HIGH(___str_6)
	MOVWF	r0x05
	MOVLW	LOW(___str_6)
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
;	.line	223; meter_logger.c	fsk_rx_enable();
	CALL	_fsk_rx_enable
;	.line	224; meter_logger.c	last_fifo_size = 0;
	CLRF	r0x00
	CLRF	r0x01
;	.line	225; meter_logger.c	sleep_ms(400);							// sleep 400 ms to let some data come in
	MOVLW	0x01
	MOVWF	POSTDEC1
	MOVLW	0x90
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVF	POSTINC1, F
	MOVF	POSTINC1, F
;	.line	226; meter_logger.c	fifo_size = fifo_in_use();
	CALL	_fifo_in_use
	MOVWF	r0x02
	MOVFF	PRODL, r0x03
_00122_DS_:
;	.line	227; meter_logger.c	while (fifo_size > last_fifo_size) {	// and wait while we are still receiving data
	MOVF	r0x03, W
	SUBWF	r0x01, W
	BNZ	_00253_DS_
	MOVF	r0x02, W
	SUBWF	r0x00, W
_00253_DS_:
	BC	_00124_DS_
;	.line	228; meter_logger.c	last_fifo_size = fifo_size;
	MOVFF	r0x02, r0x00
	MOVFF	r0x03, r0x01
;	.line	229; meter_logger.c	sleep_ms(200);						// return data when no data for 100 ms
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0xc8
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVF	POSTINC1, F
	MOVF	POSTINC1, F
;	.line	230; meter_logger.c	fifo_size = fifo_in_use();
	CALL	_fifo_in_use
	MOVWF	r0x02
	MOVFF	PRODL, r0x03
	BRA	_00122_DS_
_00124_DS_:
;	.line	232; meter_logger.c	fsk_rx_disable();
	CALL	_fsk_rx_disable
;	.line	235; meter_logger.c	usart_puts("\n\rkamstrup - kmp frame received:\n\r");
	MOVLW	UPPER(___str_7)
	MOVWF	r0x06
	MOVLW	HIGH(___str_7)
	MOVWF	r0x05
	MOVLW	LOW(___str_7)
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
;	.line	238; meter_logger.c	for (i = 0; i < fifo_in_use(); i++) {
	CLRF	r0x04
	CLRF	r0x05
_00152_DS_:
	CALL	_fifo_in_use
	MOVWF	r0x06
	MOVFF	PRODL, r0x07
	MOVF	r0x07, W
	SUBWF	r0x05, W
	BNZ	_00254_DS_
	MOVF	r0x06, W
	SUBWF	r0x04, W
_00254_DS_:
	BTFSC	STATUS, 0
	BRA	_00125_DS_
;	.line	240; meter_logger.c	fifo_get(&sub_cmd);
	MOVLW	HIGH(_main_sub_cmd_1_79)
	MOVWF	r0x07
	MOVLW	LOW(_main_sub_cmd_1_79)
	MOVWF	r0x06
	MOVLW	0x80
	MOVWF	r0x08
	MOVF	r0x08, W
	MOVWF	POSTDEC1
	MOVF	r0x07, W
	MOVWF	POSTDEC1
	MOVF	r0x06, W
	MOVWF	POSTDEC1
	CALL	_fifo_get
	MOVLW	0x03
	ADDWF	FSR1L, F
;	.line	242; meter_logger.c	sprintf(debug_buffer, "%d ", sub_cmd);
	MOVFF	_main_sub_cmd_1_79, r0x06
	CLRF	r0x07
	MOVLW	UPPER(___str_3)
	MOVWF	r0x0a
	MOVLW	HIGH(___str_3)
	MOVWF	r0x09
	MOVLW	LOW(___str_3)
	MOVWF	r0x08
	MOVLW	HIGH(_debug_buffer)
	MOVWF	r0x0c
	MOVLW	LOW(_debug_buffer)
	MOVWF	r0x0b
	MOVLW	0x80
	MOVWF	r0x0d
	MOVF	r0x07, W
	MOVWF	POSTDEC1
	MOVF	r0x06, W
	MOVWF	POSTDEC1
	MOVF	r0x0a, W
	MOVWF	POSTDEC1
	MOVF	r0x09, W
	MOVWF	POSTDEC1
	MOVF	r0x08, W
	MOVWF	POSTDEC1
	MOVF	r0x0d, W
	MOVWF	POSTDEC1
	MOVF	r0x0c, W
	MOVWF	POSTDEC1
	MOVF	r0x0b, W
	MOVWF	POSTDEC1
	CALL	_sprintf
	MOVLW	0x08
	ADDWF	FSR1L, F
;	.line	243; meter_logger.c	usart_puts(debug_buffer);
	MOVLW	HIGH(_debug_buffer)
	MOVWF	r0x07
	MOVLW	LOW(_debug_buffer)
	MOVWF	r0x06
	MOVLW	0x80
	MOVWF	r0x08
	MOVF	r0x08, W
	MOVWF	POSTDEC1
	MOVF	r0x07, W
	MOVWF	POSTDEC1
	MOVF	r0x06, W
	MOVWF	POSTDEC1
	CALL	_usart_puts
	MOVLW	0x03
	ADDWF	FSR1L, F
	BANKSEL	_main_sub_cmd_1_79
;	.line	245; meter_logger.c	fifo_put(sub_cmd);
	MOVF	_main_sub_cmd_1_79, W, B
	MOVWF	POSTDEC1
	CALL	_fifo_put
	MOVF	POSTINC1, F
;	.line	238; meter_logger.c	for (i = 0; i < fifo_in_use(); i++) {
	INFSNZ	r0x04, F
	INCF	r0x05, F
	BRA	_00152_DS_
_00125_DS_:
;	.line	249; meter_logger.c	rs232_tx_enable();
	CALL	_rs232_tx_enable
_00126_DS_:
;	.line	250; meter_logger.c	while (fifo_get(&sub_cmd)) {
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
	BZ	_00128_DS_
	BANKSEL	_main_sub_cmd_1_79
;	.line	251; meter_logger.c	rs232_tx_byte(sub_cmd);
	MOVF	_main_sub_cmd_1_79, W, B
	MOVWF	POSTDEC1
	CALL	_rs232_tx_byte
	MOVF	POSTINC1, F
;	.line	252; meter_logger.c	sleep_ms(RS232_TX_SLEEP);
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x0c
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVF	POSTINC1, F
	MOVF	POSTINC1, F
	BRA	_00126_DS_
_00128_DS_:
;	.line	254; meter_logger.c	rs232_tx_disable();
	CALL	_rs232_tx_disable
;	.line	260; meter_logger.c	rs232_rx_enable();
	CALL	_rs232_rx_enable
;	.line	261; meter_logger.c	last_fifo_size = 0;
	CLRF	r0x00
	CLRF	r0x01
;	.line	262; meter_logger.c	sleep_ms(400);							// sleep 200 ms to let some data come in
	MOVLW	0x01
	MOVWF	POSTDEC1
	MOVLW	0x90
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVF	POSTINC1, F
	MOVF	POSTINC1, F
;	.line	263; meter_logger.c	fifo_size = fifo_in_use();
	CALL	_fifo_in_use
	MOVWF	r0x02
	MOVFF	PRODL, r0x03
_00129_DS_:
;	.line	265; meter_logger.c	while (fifo_size > last_fifo_size) {	// and wait while we are still receiving data
	MOVF	r0x03, W
	SUBWF	r0x01, W
	BNZ	_00255_DS_
	MOVF	r0x02, W
	SUBWF	r0x00, W
_00255_DS_:
	BC	_00131_DS_
;	.line	266; meter_logger.c	last_fifo_size = fifo_size;
	MOVFF	r0x02, r0x00
	MOVFF	r0x03, r0x01
;	.line	267; meter_logger.c	sleep_ms(200);						// return data when no data for 100 ms
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0xc8
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVF	POSTINC1, F
	MOVF	POSTINC1, F
;	.line	268; meter_logger.c	fifo_size = fifo_in_use();
	CALL	_fifo_in_use
	MOVWF	r0x02
	MOVFF	PRODL, r0x03
	BRA	_00129_DS_
_00131_DS_:
;	.line	271; meter_logger.c	rs232_rx_disable();
	CALL	_rs232_rx_disable
;	.line	275; meter_logger.c	usart_puts("\n\rkamstrup - kmp reply received:\n\r");
	MOVLW	UPPER(___str_8)
	MOVWF	r0x02
	MOVLW	HIGH(___str_8)
	MOVWF	r0x01
	MOVLW	LOW(___str_8)
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
;	.line	278; meter_logger.c	for (i = 0; i < fifo_in_use(); i++) {
	CLRF	r0x00
	CLRF	r0x01
_00155_DS_:
	CALL	_fifo_in_use
	MOVWF	r0x02
	MOVFF	PRODL, r0x03
	MOVF	r0x03, W
	SUBWF	r0x01, W
	BNZ	_00256_DS_
	MOVF	r0x02, W
	SUBWF	r0x00, W
_00256_DS_:
	BTFSC	STATUS, 0
	BRA	_00132_DS_
;	.line	280; meter_logger.c	fifo_get(&sub_cmd);
	MOVLW	HIGH(_main_sub_cmd_1_79)
	MOVWF	r0x03
	MOVLW	LOW(_main_sub_cmd_1_79)
	MOVWF	r0x02
	MOVLW	0x80
	MOVWF	r0x04
	MOVF	r0x04, W
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	CALL	_fifo_get
	MOVLW	0x03
	ADDWF	FSR1L, F
;	.line	282; meter_logger.c	sprintf(debug_buffer, "%d ", sub_cmd);
	MOVFF	_main_sub_cmd_1_79, r0x02
	CLRF	r0x03
	MOVLW	UPPER(___str_3)
	MOVWF	r0x06
	MOVLW	HIGH(___str_3)
	MOVWF	r0x05
	MOVLW	LOW(___str_3)
	MOVWF	r0x04
	MOVLW	HIGH(_debug_buffer)
	MOVWF	r0x08
	MOVLW	LOW(_debug_buffer)
	MOVWF	r0x07
	MOVLW	0x80
	MOVWF	r0x09
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x06, W
	MOVWF	POSTDEC1
	MOVF	r0x05, W
	MOVWF	POSTDEC1
	MOVF	r0x04, W
	MOVWF	POSTDEC1
	MOVF	r0x09, W
	MOVWF	POSTDEC1
	MOVF	r0x08, W
	MOVWF	POSTDEC1
	MOVF	r0x07, W
	MOVWF	POSTDEC1
	CALL	_sprintf
	MOVLW	0x08
	ADDWF	FSR1L, F
;	.line	283; meter_logger.c	usart_puts(debug_buffer);
	MOVLW	HIGH(_debug_buffer)
	MOVWF	r0x03
	MOVLW	LOW(_debug_buffer)
	MOVWF	r0x02
	MOVLW	0x80
	MOVWF	r0x04
	MOVF	r0x04, W
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	CALL	_usart_puts
	MOVLW	0x03
	ADDWF	FSR1L, F
	BANKSEL	_main_sub_cmd_1_79
;	.line	285; meter_logger.c	fifo_put(sub_cmd);
	MOVF	_main_sub_cmd_1_79, W, B
	MOVWF	POSTDEC1
	CALL	_fifo_put
	MOVF	POSTINC1, F
;	.line	278; meter_logger.c	for (i = 0; i < fifo_in_use(); i++) {
	INFSNZ	r0x00, F
	INCF	r0x01, F
	BRA	_00155_DS_
_00132_DS_:
;	.line	288; meter_logger.c	if (fifo_in_use()) {
	CALL	_fifo_in_use
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVF	r0x00, W
	IORWF	r0x01, W
	BZ	_00137_DS_
;	.line	290; meter_logger.c	DEBUG2_PIN = 1;
	BSF	_PORTBbits, 3
	nop
	nop
	
;	.line	295; meter_logger.c	DEBUG2_PIN = 0;
	BCF	_PORTBbits, 3
;	.line	297; meter_logger.c	fsk_tx_enable();
	CALL	_fsk_tx_enable
;	.line	299; meter_logger.c	sleep_ms(FSK_TX_SLEEP);
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x0c
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVF	POSTINC1, F
	MOVF	POSTINC1, F
_00133_DS_:
;	.line	300; meter_logger.c	while (fifo_get(&sub_cmd)) {
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
	BZ	_00135_DS_
	BANKSEL	_main_sub_cmd_1_79
;	.line	305; meter_logger.c	fsk_tx_byte(sub_cmd);
	MOVF	_main_sub_cmd_1_79, W, B
	MOVWF	POSTDEC1
	CALL	_fsk_tx_byte
	MOVF	POSTINC1, F
;	.line	306; meter_logger.c	sleep_ms(FSK_TX_SLEEP);
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x0c
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVF	POSTINC1, F
	MOVF	POSTINC1, F
	BRA	_00133_DS_
_00135_DS_:
;	.line	310; meter_logger.c	fsk_tx_disable();
	CALL	_fsk_tx_disable
	BRA	_00138_DS_
_00137_DS_:
;	.line	315; meter_logger.c	DEBUG2_PIN = 1;
	BSF	_PORTBbits, 3
	nop
	nop
	
;	.line	320; meter_logger.c	DEBUG2_PIN = 0;
	BCF	_PORTBbits, 3
	nop
	nop
	
;	.line	325; meter_logger.c	DEBUG2_PIN = 1;
	BSF	_PORTBbits, 3
	nop
	nop
	
;	.line	330; meter_logger.c	DEBUG2_PIN = 0;
	BCF	_PORTBbits, 3
;	.line	332; meter_logger.c	sprintf(debug_buffer, "\n\rno reply from meter\n\r");
	MOVLW	UPPER(___str_9)
	MOVWF	r0x02
	MOVLW	HIGH(___str_9)
	MOVWF	r0x01
	MOVLW	LOW(___str_9)
	MOVWF	r0x00
	MOVLW	HIGH(_debug_buffer)
	MOVWF	r0x04
	MOVLW	LOW(_debug_buffer)
	MOVWF	r0x03
	MOVLW	0x80
	MOVWF	r0x05
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	MOVF	r0x05, W
	MOVWF	POSTDEC1
	MOVF	r0x04, W
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	CALL	_sprintf
	MOVLW	0x06
	ADDWF	FSR1L, F
;	.line	333; meter_logger.c	usart_puts(debug_buffer);
	MOVLW	HIGH(_debug_buffer)
	MOVWF	r0x01
	MOVLW	LOW(_debug_buffer)
	MOVWF	r0x00
	MOVLW	0x80
	MOVWF	r0x02
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	_usart_puts
	MOVLW	0x03
	ADDWF	FSR1L, F
;	.line	335; meter_logger.c	fsk_tx_enable();
	CALL	_fsk_tx_enable
;	.line	336; meter_logger.c	sleep_ms(FSK_TX_SLEEP);
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x0c
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVF	POSTINC1, F
	MOVF	POSTINC1, F
;	.line	337; meter_logger.c	fsk_tx_byte(0x0d);
	MOVLW	0x0d
	MOVWF	POSTDEC1
	CALL	_fsk_tx_byte
	MOVF	POSTINC1, F
;	.line	338; meter_logger.c	sleep_ms(FSK_TX_SLEEP);
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x0c
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVF	POSTINC1, F
	MOVF	POSTINC1, F
;	.line	339; meter_logger.c	fsk_tx_disable();
	CALL	_fsk_tx_disable
_00138_DS_:
;	.line	342; meter_logger.c	usart_puts("\n\rwaiting for new command\n\r");
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
;	.line	344; meter_logger.c	fsk_rx_enable();
	CALL	_fsk_rx_enable
;	.line	346; meter_logger.c	}
	GOTO	_00143_DS_
	RETURN	

; ; Starting pCode block
S_meter_logger___debug2	code
__debug2:
;	.line	4285; meter_logger.c	void _debug2() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
;	.line	4286; meter_logger.c	DEBUG2_PIN = 0x1;
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
	
;	.line	4334; meter_logger.c	DEBUG2_PIN = 0x0;
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
;	.line	4185; meter_logger.c	void _debug() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
;	.line	4186; meter_logger.c	DEBUG_PIN = 0x1;
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
	
;	.line	4234; meter_logger.c	DEBUG_PIN = 0x0;
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
;	.line	4180; meter_logger.c	void flash_led(unsigned char ms) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
;	.line	4181; meter_logger.c	led_flash.timer = ms;
	MOVF	r0x00, W
	BANKSEL	(_led_flash + 1)
	MOVWF	(_led_flash + 1), B
; removed redundant BANKSEL
;	.line	4182; meter_logger.c	led_flash.state = LED_FLASH_RUN;
	CLRF	_led_flash, B
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__fifo_snoop	code
_fifo_snoop:
;	.line	4157; meter_logger.c	unsigned char fifo_snoop(unsigned char *c, unsigned int pos) {
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
;	.line	4158; meter_logger.c	if (fifo_in_use() > (pos)) {
	CALL	_fifo_in_use
	MOVWF	r0x05
	MOVFF	PRODL, r0x06
	MOVF	r0x06, W
	SUBWF	r0x04, W
	BNZ	_00944_DS_
	MOVF	r0x05, W
	SUBWF	r0x03, W
_00944_DS_:
	BTFSC	STATUS, 0
	BRA	_00934_DS_
	BANKSEL	(_fifo_tail + 1)
;	.line	4159; meter_logger.c	switch (fifo_tail/QUEUE_SIZE) {
	MOVF	(_fifo_tail + 1), W, B
	MOVWF	r0x05
	CLRF	r0x06
	MOVLW	0x00
	SUBWF	r0x06, W
	BNZ	_00945_DS_
	MOVLW	0x04
	SUBWF	r0x05, W
_00945_DS_:
	BTFSC	STATUS, 0
	BRA	_00932_DS_
	CLRF	PCLATH
	CLRF	PCLATU
	RLCF	r0x05, W
	RLCF	PCLATH, F
	RLCF	WREG, W
	RLCF	PCLATH, F
	ANDLW	0xfc
	ADDLW	LOW(_00946_DS_)
	MOVWF	POSTDEC1
	MOVLW	HIGH(_00946_DS_)
	ADDWFC	PCLATH, F
	MOVLW	UPPER(_00946_DS_)
	ADDWFC	PCLATU, F
	MOVF	PREINC1, W
	MOVWF	PCL
_00946_DS_:
	GOTO	_00928_DS_
	GOTO	_00929_DS_
	GOTO	_00930_DS_
	GOTO	_00931_DS_
_00928_DS_:
;	.line	4161; meter_logger.c	*c = fifo_buffer_0[(fifo_tail + pos) % QUEUE_SIZE];
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
;	.line	4162; meter_logger.c	break;
	BRA	_00932_DS_
_00929_DS_:
;	.line	4164; meter_logger.c	*c = fifo_buffer_1[(fifo_tail + pos) % QUEUE_SIZE];
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
;	.line	4165; meter_logger.c	break;
	BRA	_00932_DS_
_00930_DS_:
;	.line	4167; meter_logger.c	*c = fifo_buffer_2[(fifo_tail + pos) % QUEUE_SIZE];
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
;	.line	4168; meter_logger.c	break;
	BRA	_00932_DS_
_00931_DS_:
	BANKSEL	_fifo_tail
;	.line	4170; meter_logger.c	*c = fifo_buffer_3[(fifo_tail + pos) % QUEUE_SIZE];
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
_00932_DS_:
;	.line	4173; meter_logger.c	return 1;
	MOVLW	0x01
	BRA	_00936_DS_
_00934_DS_:
;	.line	4176; meter_logger.c	return 0;
	CLRF	WREG
_00936_DS_:
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
;	.line	4129; meter_logger.c	unsigned char fifo_get(unsigned char *c) {
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
;	.line	4130; meter_logger.c	if (fifo_in_use() != 0) {
	CALL	_fifo_in_use
	MOVWF	r0x03
	MOVFF	PRODL, r0x04
	MOVF	r0x03, W
	IORWF	r0x04, W
	BTFSC	STATUS, 2
	BRA	_00903_DS_
	BANKSEL	(_fifo_tail + 1)
;	.line	4131; meter_logger.c	switch (fifo_tail/QUEUE_SIZE) {
	MOVF	(_fifo_tail + 1), W, B
	MOVWF	r0x03
	CLRF	r0x04
	MOVLW	0x00
	SUBWF	r0x04, W
	BNZ	_00916_DS_
	MOVLW	0x04
	SUBWF	r0x03, W
_00916_DS_:
	BTFSC	STATUS, 0
	BRA	_00899_DS_
	CLRF	PCLATH
	CLRF	PCLATU
	RLCF	r0x03, W
	RLCF	PCLATH, F
	RLCF	WREG, W
	RLCF	PCLATH, F
	ANDLW	0xfc
	ADDLW	LOW(_00917_DS_)
	MOVWF	POSTDEC1
	MOVLW	HIGH(_00917_DS_)
	ADDWFC	PCLATH, F
	MOVLW	UPPER(_00917_DS_)
	ADDWFC	PCLATU, F
	MOVF	PREINC1, W
	MOVWF	PCL
_00917_DS_:
	GOTO	_00895_DS_
	GOTO	_00896_DS_
	GOTO	_00897_DS_
	GOTO	_00898_DS_
_00895_DS_:
	BANKSEL	_fifo_tail
;	.line	4133; meter_logger.c	*c = fifo_buffer_0[fifo_tail % QUEUE_SIZE];
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
;	.line	4134; meter_logger.c	break;
	BRA	_00899_DS_
_00896_DS_:
	BANKSEL	_fifo_tail
;	.line	4136; meter_logger.c	*c = fifo_buffer_1[fifo_tail % QUEUE_SIZE];
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
;	.line	4137; meter_logger.c	break;
	BRA	_00899_DS_
_00897_DS_:
	BANKSEL	_fifo_tail
;	.line	4139; meter_logger.c	*c = fifo_buffer_2[fifo_tail % QUEUE_SIZE];
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
;	.line	4140; meter_logger.c	break;
	BRA	_00899_DS_
_00898_DS_:
	BANKSEL	_fifo_tail
;	.line	4142; meter_logger.c	*c = fifo_buffer_3[fifo_tail % QUEUE_SIZE];
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
_00899_DS_:
	BANKSEL	_fifo_tail
;	.line	4145; meter_logger.c	fifo_tail++;
	INCFSZ	_fifo_tail, F, B
	BRA	_10961_DS_
; removed redundant BANKSEL
	INCF	(_fifo_tail + 1), F, B
_10961_DS_:
	BANKSEL	_fifo_tail
;	.line	4147; meter_logger.c	if (fifo_tail == QUEUE_SIZE_COMBINED) {
	MOVF	_fifo_tail, W, B
	BNZ	_00922_DS_
; removed redundant BANKSEL
	MOVF	(_fifo_tail + 1), W, B
	XORLW	0x04
	BZ	_00923_DS_
_00922_DS_:
	BRA	_00901_DS_
_00923_DS_:
	BANKSEL	_fifo_tail
;	.line	4148; meter_logger.c	fifo_tail = 0;
	CLRF	_fifo_tail, B
; removed redundant BANKSEL
	CLRF	(_fifo_tail + 1), B
_00901_DS_:
;	.line	4150; meter_logger.c	return 1;
	MOVLW	0x01
	BRA	_00905_DS_
_00903_DS_:
;	.line	4153; meter_logger.c	return 0;
	CLRF	WREG
_00905_DS_:
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
;	.line	4101; meter_logger.c	unsigned char fifo_put(unsigned char c) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVFF	r0x01, POSTDEC1
	MOVFF	r0x02, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
;	.line	4102; meter_logger.c	if (fifo_in_use() != QUEUE_SIZE_COMBINED) {
	CALL	_fifo_in_use
	MOVWF	r0x01
	MOVFF	PRODL, r0x02
	MOVF	r0x01, W
	BNZ	_00882_DS_
	MOVF	r0x02, W
	XORLW	0x04
	BNZ	_00882_DS_
	BRA	_00868_DS_
_00882_DS_:
	BANKSEL	(_fifo_head + 1)
;	.line	4103; meter_logger.c	switch (fifo_head/QUEUE_SIZE) {
	MOVF	(_fifo_head + 1), W, B
	MOVWF	r0x01
	CLRF	r0x02
	MOVLW	0x00
	SUBWF	r0x02, W
	BNZ	_00883_DS_
	MOVLW	0x04
	SUBWF	r0x01, W
_00883_DS_:
	BTFSC	STATUS, 0
	BRA	_00864_DS_
	CLRF	PCLATH
	CLRF	PCLATU
	RLCF	r0x01, W
	RLCF	PCLATH, F
	RLCF	WREG, W
	RLCF	PCLATH, F
	ANDLW	0xfc
	ADDLW	LOW(_00884_DS_)
	MOVWF	POSTDEC1
	MOVLW	HIGH(_00884_DS_)
	ADDWFC	PCLATH, F
	MOVLW	UPPER(_00884_DS_)
	ADDWFC	PCLATU, F
	MOVF	PREINC1, W
	MOVWF	PCL
_00884_DS_:
	GOTO	_00860_DS_
	GOTO	_00861_DS_
	GOTO	_00862_DS_
	GOTO	_00863_DS_
_00860_DS_:
	BANKSEL	_fifo_head
;	.line	4105; meter_logger.c	fifo_buffer_0[fifo_head % QUEUE_SIZE] = c;
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
;	.line	4106; meter_logger.c	break;
	BRA	_00864_DS_
_00861_DS_:
	BANKSEL	_fifo_head
;	.line	4108; meter_logger.c	fifo_buffer_1[fifo_head % QUEUE_SIZE] = c;
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
;	.line	4109; meter_logger.c	break;
	BRA	_00864_DS_
_00862_DS_:
	BANKSEL	_fifo_head
;	.line	4111; meter_logger.c	fifo_buffer_2[fifo_head % QUEUE_SIZE] = c;
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
;	.line	4112; meter_logger.c	break;
	BRA	_00864_DS_
_00863_DS_:
	BANKSEL	_fifo_head
;	.line	4114; meter_logger.c	fifo_buffer_3[fifo_head % QUEUE_SIZE] = c;
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
_00864_DS_:
	BANKSEL	_fifo_head
;	.line	4117; meter_logger.c	fifo_head++;
	INCFSZ	_fifo_head, F, B
	BRA	_20962_DS_
; removed redundant BANKSEL
	INCF	(_fifo_head + 1), F, B
_20962_DS_:
	BANKSEL	_fifo_head
;	.line	4119; meter_logger.c	if (fifo_head == QUEUE_SIZE_COMBINED) {
	MOVF	_fifo_head, W, B
	BNZ	_00889_DS_
; removed redundant BANKSEL
	MOVF	(_fifo_head + 1), W, B
	XORLW	0x04
	BZ	_00890_DS_
_00889_DS_:
	BRA	_00866_DS_
_00890_DS_:
	BANKSEL	_fifo_head
;	.line	4120; meter_logger.c	fifo_head = 0;
	CLRF	_fifo_head, B
; removed redundant BANKSEL
	CLRF	(_fifo_head + 1), B
_00866_DS_:
;	.line	4122; meter_logger.c	return 1;
	MOVLW	0x01
	BRA	_00870_DS_
_00868_DS_:
;	.line	4125; meter_logger.c	return 0;
	CLRF	WREG
_00870_DS_:
	MOVFF	PREINC1, r0x02
	MOVFF	PREINC1, r0x01
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__fifo_in_use	code
_fifo_in_use:
;	.line	4097; meter_logger.c	unsigned int fifo_in_use() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVFF	r0x01, POSTDEC1
	BANKSEL	_fifo_tail
;	.line	4098; meter_logger.c	return fifo_head - fifo_tail;
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
;	.line	4092; meter_logger.c	void fsk_tx_byte(unsigned char c) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
;	.line	4093; meter_logger.c	fsk_proto.data = c;
	MOVF	r0x00, W
	BANKSEL	(_fsk_proto + 12)
	MOVWF	(_fsk_proto + 12), B
;	.line	4094; meter_logger.c	fsk_proto.data_len = 8;
	MOVLW	0x08
; removed redundant BANKSEL
	MOVWF	(_fsk_proto + 13), B
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__send_fsk_low	code
_send_fsk_low:
;	.line	2639; meter_logger.c	void send_fsk_low(void) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
;	.line	2640; meter_logger.c	PWM_PIN = 1;
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
	
;	.line	2847; meter_logger.c	PWM_PIN = 0;
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
	
;	.line	3054; meter_logger.c	PWM_PIN = 1;
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
	
;	.line	3261; meter_logger.c	PWM_PIN = 0;
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
	
;	.line	3468; meter_logger.c	PWM_PIN = 1;
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
	
;	.line	3675; meter_logger.c	PWM_PIN = 0;
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
	
;	.line	3882; meter_logger.c	PWM_PIN = 1;
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
	
;	.line	4089; meter_logger.c	PWM_PIN = 0;
	BCF	_PORTCbits, 1
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__send_fsk_high	code
_send_fsk_high:
;	.line	1117; meter_logger.c	void send_fsk_high(void) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
;	.line	1118; meter_logger.c	PWM_PIN = 1;
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
	
;	.line	1256; meter_logger.c	PWM_PIN = 0;
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
	
;	.line	1394; meter_logger.c	PWM_PIN = 1;
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
	
;	.line	1532; meter_logger.c	PWM_PIN = 0;
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
	
;	.line	1670; meter_logger.c	PWM_PIN = 1;
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
	
;	.line	1808; meter_logger.c	PWM_PIN = 0;
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
	
;	.line	1946; meter_logger.c	PWM_PIN = 1;
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
	
;	.line	2084; meter_logger.c	PWM_PIN = 0;
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
	
;	.line	2222; meter_logger.c	PWM_PIN = 1;
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
	
;	.line	2360; meter_logger.c	PWM_PIN = 0;
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
	
;	.line	2498; meter_logger.c	PWM_PIN = 1;
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
	
;	.line	2636; meter_logger.c	PWM_PIN = 0;
	BCF	_PORTCbits, 1
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__fsk_rx_disable	code
_fsk_rx_disable:
;	.line	1112; meter_logger.c	void fsk_rx_disable() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
;	.line	1113; meter_logger.c	PIE2bits.CMIE = 0;		// Disable comparator interrupt
	BCF	_PIE2bits, 6
	BANKSEL	_codec_type
;	.line	1114; meter_logger.c	codec_type = NONE;
	CLRF	_codec_type, B
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__fsk_rx_enable	code
_fsk_rx_enable:
;	.line	1076; meter_logger.c	void fsk_rx_enable() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
;	.line	1077; meter_logger.c	fsk_proto.state = START_BIT_WAIT;
	MOVLW	0x02
	BANKSEL	_fsk_proto
	MOVWF	_fsk_proto, B
; removed redundant BANKSEL
;	.line	1078; meter_logger.c	fsk_proto.start_bit_time = 0;
	CLRF	(_fsk_proto + 10), B
; removed redundant BANKSEL
	CLRF	(_fsk_proto + 11), B
;	.line	1080; meter_logger.c	timer0_reload = TIMER0_FSK;
	MOVLW	0x9f
	BANKSEL	_timer0_reload
	MOVWF	_timer0_reload, B
	MOVLW	0xf9
; removed redundant BANKSEL
	MOVWF	(_timer0_reload + 1), B
;	.line	1082; meter_logger.c	codec_type = FSK_RX;
	MOVLW	0x04
	BANKSEL	_codec_type
	MOVWF	_codec_type, B
;	.line	1085; meter_logger.c	T0CONbits.TMR0ON = 1;
	BSF	_T0CONbits, 7
;	.line	1086; meter_logger.c	T0CONbits.T0PS0 = 0;
	BCF	_T0CONbits, 0
;	.line	1087; meter_logger.c	T0CONbits.T0PS1 = 0;
	BCF	_T0CONbits, 1
;	.line	1088; meter_logger.c	T0CONbits.T0PS2 = 0;		// prescaler 1:2
	BCF	_T0CONbits, 2
;	.line	1089; meter_logger.c	T0CONbits.T08BIT = 0;		// use timer0 16-bit counter
	BCF	_T0CONbits, 6
;	.line	1090; meter_logger.c	T0CONbits.T0CS = 0;			// internal clock source
	BCF	_T0CONbits, 5
;	.line	1091; meter_logger.c	T0CONbits.PSA = 1;			// disable timer0 prescaler
	BSF	_T0CONbits, 3
;	.line	1092; meter_logger.c	INTCON2bits.TMR0IP = 1;		// high priority
	BSF	_INTCON2bits, 2
;	.line	1093; meter_logger.c	INTCONbits.TMR0IE = 0;		// Dont enable TMR0 Interrupt
	BCF	_INTCONbits, 5
;	.line	1096; meter_logger.c	CVRCONbits.CVREF = 0xf;	// 0V
	BSF	_CVRCONbits, 4
;	.line	1098; meter_logger.c	CVRCONbits.CVRSS = 0;	// VDD – VSS
	BCF	_CVRCONbits, 4
;	.line	1099; meter_logger.c	CVRCONbits.CVRR = 0;	// high range, 0.25 CVRSRC to 0.75 CVRSRC, with CVRSRC/32 step size
	BCF	_CVRCONbits, 5
;	.line	1100; meter_logger.c	CVRCONbits.CVR = 9;		// 2,65625 V
	MOVF	_CVRCONbits, W
	ANDLW	0xf0
	IORLW	0x09
	MOVWF	_CVRCONbits
;	.line	1101; meter_logger.c	CVRCONbits.CVROE = 0;	// Comparator VREF Output disabled, CVREF voltage is disconnected from the RA2/AN2/VREF-/CVREF pin
	BCF	_CVRCONbits, 6
;	.line	1102; meter_logger.c	CVRCONbits.CVREN = 1;	// Comparator Voltage Reference Enable bit
	BSF	_CVRCONbits, 7
;	.line	1104; meter_logger.c	CMCONbits.CM = 0x6;		// four inputs multiplexed to two comparators
	MOVF	_CMCONbits, W
	ANDLW	0xf8
	IORLW	0x06
	MOVWF	_CMCONbits
;	.line	1105; meter_logger.c	CMCONbits.CIS = 0;		// multiplexed to RA0/AN0 and RA1/AN1
	BCF	_CMCONbits, 3
;	.line	1106; meter_logger.c	CMCONbits.C1INV = 1;	// inverted output, C1 VIN+ < C1 VIN-
	BSF	_CMCONbits, 4
;	.line	1108; meter_logger.c	IPR2bits.CMIP = 1;		// high priority
	BSF	_IPR2bits, 6
;	.line	1109; meter_logger.c	PIE2bits.CMIE = 1;		// Enable comparator interrupt
	BSF	_PIE2bits, 6
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__fsk_tx_disable	code
_fsk_tx_disable:
;	.line	1070; meter_logger.c	void fsk_tx_disable() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	BANKSEL	_codec_type
;	.line	1071; meter_logger.c	codec_type = NONE;
	CLRF	_codec_type, B
;	.line	1072; meter_logger.c	T0CONbits.TMR0ON = 0;	// Disable TMR0 
	BCF	_T0CONbits, 7
;	.line	1073; meter_logger.c	PIE2bits.CMIE = 1;		// Disable comparator interrupt
	BSF	_PIE2bits, 6
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__fsk_tx_enable	code
_fsk_tx_enable:
;	.line	1052; meter_logger.c	void fsk_tx_enable() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
;	.line	1053; meter_logger.c	timer0_reload = TIMER0_FSK;
	MOVLW	0x9f
	BANKSEL	_timer0_reload
	MOVWF	_timer0_reload, B
	MOVLW	0xf9
; removed redundant BANKSEL
	MOVWF	(_timer0_reload + 1), B
	BANKSEL	_fsk_proto
;	.line	1055; meter_logger.c	fsk_proto.state = INIT_STATE;
	CLRF	_fsk_proto, B
;	.line	1056; meter_logger.c	codec_type = FSK_TX;
	MOVLW	0x05
	BANKSEL	_codec_type
	MOVWF	_codec_type, B
;	.line	1059; meter_logger.c	T0CONbits.TMR0ON = 1;
	BSF	_T0CONbits, 7
;	.line	1060; meter_logger.c	T0CONbits.T0PS0 = 0;
	BCF	_T0CONbits, 0
;	.line	1061; meter_logger.c	T0CONbits.T0PS1 = 0;
	BCF	_T0CONbits, 1
;	.line	1062; meter_logger.c	T0CONbits.T0PS2 = 0;		// prescaler 1:2
	BCF	_T0CONbits, 2
;	.line	1063; meter_logger.c	T0CONbits.T08BIT = 0;		// use timer0 16-bit counter
	BCF	_T0CONbits, 6
;	.line	1064; meter_logger.c	T0CONbits.T0CS = 0;			// internal clock source
	BCF	_T0CONbits, 5
;	.line	1065; meter_logger.c	T0CONbits.PSA = 1;			// disable timer0 prescaler
	BSF	_T0CONbits, 3
;	.line	1066; meter_logger.c	INTCON2bits.TMR0IP = 1;		// high priority
	BSF	_INTCON2bits, 2
;	.line	1067; meter_logger.c	INTCONbits.TMR0IE = 1;		// Enable TMR0 Interrupt
	BSF	_INTCONbits, 5
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__rs232_tx_byte	code
_rs232_tx_byte:
;	.line	1045; meter_logger.c	void rs232_tx_byte(unsigned char c) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
;	.line	1046; meter_logger.c	rs232_proto.data = c;
	MOVF	r0x00, W
	BANKSEL	(_rs232_proto + 2)
	MOVWF	(_rs232_proto + 2), B
;	.line	1047; meter_logger.c	rs232_proto.data_len = 8;
	MOVLW	0x08
; removed redundant BANKSEL
	MOVWF	(_rs232_proto + 3), B
;	.line	1048; meter_logger.c	T0CONbits.TMR0ON = 1;		// start timer 0
	BSF	_T0CONbits, 7
;	.line	1049; meter_logger.c	INTCONbits.TMR0IF = 1;		// enter timer interrupt handler now
	BSF	_INTCONbits, 2
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__rs232_rx_disable	code
_rs232_rx_disable:
;	.line	1039; meter_logger.c	void rs232_rx_disable() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
;	.line	1040; meter_logger.c	INTCONbits.INT0IE = 0;		// disable ext int
	BCF	_INTCONbits, 4
	BANKSEL	_codec_type
;	.line	1041; meter_logger.c	codec_type = NONE;
	CLRF	_codec_type, B
;	.line	1042; meter_logger.c	T0CONbits.TMR0ON = 0;
	BCF	_T0CONbits, 7
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__rs232_rx_enable	code
_rs232_rx_enable:
;	.line	1015; meter_logger.c	void rs232_rx_enable() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
;	.line	1016; meter_logger.c	rs232_proto.state = START_BIT_WAIT;
	MOVLW	0x02
	BANKSEL	_rs232_proto
	MOVWF	_rs232_proto, B
; removed redundant BANKSEL
;	.line	1017; meter_logger.c	rs232_proto.data_len = 0;
	CLRF	(_rs232_proto + 3), B
;	.line	1019; meter_logger.c	timer0_reload = TIMER0_RS232_1200;
	MOVLW	0xae
	BANKSEL	_timer0_reload
	MOVWF	_timer0_reload, B
	MOVLW	0xf9
; removed redundant BANKSEL
	MOVWF	(_timer0_reload + 1), B
;	.line	1021; meter_logger.c	codec_type = RS232_RX;
	MOVLW	0x02
	BANKSEL	_codec_type
	MOVWF	_codec_type, B
;	.line	1024; meter_logger.c	T0CONbits.TMR0ON = 0;
	BCF	_T0CONbits, 7
;	.line	1025; meter_logger.c	T0CONbits.T0PS0 = 0;
	BCF	_T0CONbits, 0
;	.line	1026; meter_logger.c	T0CONbits.T0PS1 = 0;
	BCF	_T0CONbits, 1
;	.line	1027; meter_logger.c	T0CONbits.T0PS2 = 0;		// prescaler 1:2
	BCF	_T0CONbits, 2
;	.line	1028; meter_logger.c	T0CONbits.T08BIT = 0;		// use timer0 16-bit counter
	BCF	_T0CONbits, 6
;	.line	1029; meter_logger.c	T0CONbits.T0CS = 0;			// internal clock source
	BCF	_T0CONbits, 5
;	.line	1030; meter_logger.c	T0CONbits.PSA = 1;			// disable timer0 prescaler
	BSF	_T0CONbits, 3
;	.line	1031; meter_logger.c	INTCON2bits.TMR0IP = 1;		// high priority
	BSF	_INTCON2bits, 2
;	.line	1032; meter_logger.c	INTCONbits.TMR0IE = 1;		// Enable TMR0 Interrupt
	BSF	_INTCONbits, 5
;	.line	1033; meter_logger.c	INTCONbits.TMR0IF = 0;
	BCF	_INTCONbits, 2
;	.line	1035; meter_logger.c	INTCONbits.INT0IE = 1;		// enable ext int
	BSF	_INTCONbits, 4
;	.line	1036; meter_logger.c	INTCON2bits.INTEDG0 = 1;	// rising edge
	BSF	_INTCON2bits, 6
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__rs232_tx_disable	code
_rs232_tx_disable:
;	.line	1009; meter_logger.c	void rs232_tx_disable() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	BANKSEL	_codec_type
;	.line	1010; meter_logger.c	codec_type = NONE;
	CLRF	_codec_type, B
;	.line	1011; meter_logger.c	IR_LED_PIN = 0;				// no need to set it to inverted idle
	BCF	_PORTBbits, 1
;	.line	1012; meter_logger.c	T0CONbits.TMR0ON = 0;
	BCF	_T0CONbits, 7
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__rs232_tx_enable	code
_rs232_tx_enable:
;	.line	983; meter_logger.c	void rs232_tx_enable() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
;	.line	984; meter_logger.c	timer0_reload = TIMER0_RS232_1200;
	MOVLW	0xae
	BANKSEL	_timer0_reload
	MOVWF	_timer0_reload, B
	MOVLW	0xf9
; removed redundant BANKSEL
	MOVWF	(_timer0_reload + 1), B
	BANKSEL	_rs232_proto
;	.line	986; meter_logger.c	rs232_proto.state = INIT_STATE;
	CLRF	_rs232_proto, B
; removed redundant BANKSEL
;	.line	987; meter_logger.c	rs232_proto.data_len = 0;
	CLRF	(_rs232_proto + 3), B
;	.line	989; meter_logger.c	IR_LED_PIN = 0;				// inverted rs232 output on ir, idle = no ir light
	BCF	_PORTBbits, 1
;	.line	991; meter_logger.c	codec_type = RS232_TX;
	MOVLW	0x03
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
;	.line	1006; meter_logger.c	T0CONbits.TMR0ON = 0;		// timer 0 started in rs232_tx_byte()
	BCF	_T0CONbits, 7
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__testo_ir_disable	code
_testo_ir_disable:
;	.line	978; meter_logger.c	void testo_ir_disable() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	BANKSEL	_codec_type
;	.line	979; meter_logger.c	codec_type = NONE;
	CLRF	_codec_type, B
;	.line	980; meter_logger.c	INTCONbits.INT0IE = 0;		// disable ext int
	BCF	_INTCONbits, 4
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__testo_ir_enable	code
_testo_ir_enable:
;	.line	954; meter_logger.c	void testo_ir_enable() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	BANKSEL	_testo_ir_proto
;	.line	955; meter_logger.c	testo_ir_proto.state = INIT_STATE;
	CLRF	_testo_ir_proto, B
; removed redundant BANKSEL
;	.line	956; meter_logger.c	testo_ir_proto.start_bit_len = 0;
	CLRF	(_testo_ir_proto + 2), B
;	.line	958; meter_logger.c	timer0_reload = TIMER0_TESTO;
	MOVLW	0x23
	BANKSEL	_timer0_reload
	MOVWF	_timer0_reload, B
	MOVLW	0xf3
; removed redundant BANKSEL
	MOVWF	(_timer0_reload + 1), B
;	.line	960; meter_logger.c	codec_type = TESTO;
	MOVLW	0x01
	BANKSEL	_codec_type
	MOVWF	_codec_type, B
;	.line	963; meter_logger.c	T0CONbits.TMR0ON = 0;
	BCF	_T0CONbits, 7
;	.line	964; meter_logger.c	T0CONbits.T0PS0 = 0;
	BCF	_T0CONbits, 0
;	.line	965; meter_logger.c	T0CONbits.T0PS1 = 0;
	BCF	_T0CONbits, 1
;	.line	966; meter_logger.c	T0CONbits.T0PS2 = 0;		// prescaler 1:2
	BCF	_T0CONbits, 2
;	.line	967; meter_logger.c	T0CONbits.T08BIT = 0;		// use timer0 16-bit counter
	BCF	_T0CONbits, 6
;	.line	968; meter_logger.c	T0CONbits.T0CS = 0;			// internal clock source
	BCF	_T0CONbits, 5
;	.line	969; meter_logger.c	T0CONbits.PSA = 1;			// disable timer0 prescaler
	BSF	_T0CONbits, 3
;	.line	970; meter_logger.c	INTCON2bits.TMR0IP = 1;		// high priority
	BSF	_INTCON2bits, 2
;	.line	971; meter_logger.c	INTCONbits.TMR0IE = 1;		// Enable TMR0 Interrupt
	BSF	_INTCONbits, 5
;	.line	972; meter_logger.c	INTCONbits.TMR0IF = 0;
	BCF	_INTCONbits, 2
;	.line	974; meter_logger.c	INTCONbits.INT0IE = 1;		// enable ext int
	BSF	_INTCONbits, 4
;	.line	975; meter_logger.c	INTCON2bits.INTEDG0 = 1;	// rising edge
	BSF	_INTCON2bits, 6
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__testo_valid_err_corr	code
_testo_valid_err_corr:
;	.line	899; meter_logger.c	unsigned char testo_valid_err_corr(unsigned int c) {
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
;	.line	906; meter_logger.c	calculated_err_corr_bit = 0;
	CLRF	r0x02
;	.line	907; meter_logger.c	for (i = 0; i < 8; i++) {
	MOVLW	0x78
	ANDWF	r0x00, W
	MOVWF	r0x03
	CLRF	r0x04
	CLRF	r0x05
_00703_DS_:
;	.line	908; meter_logger.c	calculated_err_corr_bit ^= (((c & 0x78) & (1 << i)) != 0);   // 0b01111000
	MOVLW	0x01
	MOVWF	r0x06
	MOVLW	0x00
	MOVWF	r0x07
	MOVF	r0x05, W
	BZ	_00745_DS_
	NEGF	WREG
	BCF	STATUS, 0
_00746_DS_:
	RLCF	r0x06, F
	RLCF	r0x07, F
	ADDLW	0x01
	BNC	_00746_DS_
_00745_DS_:
	MOVF	r0x03, W
	ANDWF	r0x06, F
	MOVF	r0x04, W
	ANDWF	r0x07, F
	MOVF	r0x06, W
	BNZ	_00748_DS_
	MOVF	r0x07, W
	BNZ	_00748_DS_
	CLRF	r0x06
	INCF	r0x06, F
	BRA	_00749_DS_
_00748_DS_:
	CLRF	r0x06
_00749_DS_:
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
;	.line	907; meter_logger.c	for (i = 0; i < 8; i++) {
	INCF	r0x05, F
	MOVLW	0x08
	SUBWF	r0x05, W
	BNC	_00703_DS_
;	.line	911; meter_logger.c	calculated_err_corr = calculated_err_corr << 1;
	RLNCF	r0x02, W
	ANDLW	0xfe
	MOVWF	r0x03
;	.line	914; meter_logger.c	calculated_err_corr_bit = 0;
	CLRF	r0x02
;	.line	915; meter_logger.c	for (i = 0; i < 8; i++) {
	MOVLW	0xe6
	ANDWF	r0x00, W
	MOVWF	r0x04
	CLRF	r0x05
	CLRF	r0x06
_00705_DS_:
;	.line	916; meter_logger.c	calculated_err_corr_bit ^= (((c & 0xe6) & (1 << i)) != 0);   // 0b11100110
	MOVLW	0x01
	MOVWF	r0x07
	MOVLW	0x00
	MOVWF	r0x08
	MOVF	r0x06, W
	BZ	_00753_DS_
	NEGF	WREG
	BCF	STATUS, 0
_00754_DS_:
	RLCF	r0x07, F
	RLCF	r0x08, F
	ADDLW	0x01
	BNC	_00754_DS_
_00753_DS_:
	MOVF	r0x04, W
	ANDWF	r0x07, F
	MOVF	r0x05, W
	ANDWF	r0x08, F
	MOVF	r0x07, W
	BNZ	_00756_DS_
	MOVF	r0x08, W
	BNZ	_00756_DS_
	CLRF	r0x07
	INCF	r0x07, F
	BRA	_00757_DS_
_00756_DS_:
	CLRF	r0x07
_00757_DS_:
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
;	.line	915; meter_logger.c	for (i = 0; i < 8; i++) {
	INCF	r0x06, F
	MOVLW	0x08
	SUBWF	r0x06, W
	BNC	_00705_DS_
;	.line	918; meter_logger.c	calculated_err_corr |= calculated_err_corr_bit;
	MOVF	r0x02, W
	IORWF	r0x03, F
;	.line	919; meter_logger.c	calculated_err_corr = calculated_err_corr << 1;
	BCF	STATUS, 0
	RLCF	r0x03, F
;	.line	922; meter_logger.c	calculated_err_corr_bit = 0;
	CLRF	r0x02
;	.line	923; meter_logger.c	for (i = 0; i < 8; i++) {
	MOVLW	0xd5
	ANDWF	r0x00, W
	MOVWF	r0x04
	CLRF	r0x05
	CLRF	r0x06
_00707_DS_:
;	.line	924; meter_logger.c	calculated_err_corr_bit ^= (((c & 0xd5) & (1 << i)) != 0);   // 0b11010101
	MOVLW	0x01
	MOVWF	r0x07
	MOVLW	0x00
	MOVWF	r0x08
	MOVF	r0x06, W
	BZ	_00762_DS_
	NEGF	WREG
	BCF	STATUS, 0
_00763_DS_:
	RLCF	r0x07, F
	RLCF	r0x08, F
	ADDLW	0x01
	BNC	_00763_DS_
_00762_DS_:
	MOVF	r0x04, W
	ANDWF	r0x07, F
	MOVF	r0x05, W
	ANDWF	r0x08, F
	MOVF	r0x07, W
	BNZ	_00765_DS_
	MOVF	r0x08, W
	BNZ	_00765_DS_
	CLRF	r0x07
	INCF	r0x07, F
	BRA	_00766_DS_
_00765_DS_:
	CLRF	r0x07
_00766_DS_:
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
;	.line	923; meter_logger.c	for (i = 0; i < 8; i++) {
	INCF	r0x06, F
	MOVLW	0x08
	SUBWF	r0x06, W
	BNC	_00707_DS_
;	.line	926; meter_logger.c	calculated_err_corr |= calculated_err_corr_bit;
	MOVF	r0x02, W
	IORWF	r0x03, F
;	.line	927; meter_logger.c	calculated_err_corr = calculated_err_corr << 1;
	BCF	STATUS, 0
	RLCF	r0x03, F
;	.line	930; meter_logger.c	calculated_err_corr_bit = 0;
	CLRF	r0x02
;	.line	931; meter_logger.c	for (i = 0; i < 8; i++) {
	MOVLW	0x8b
	ANDWF	r0x00, W
	MOVWF	r0x04
	CLRF	r0x05
	CLRF	r0x06
_00709_DS_:
;	.line	932; meter_logger.c	calculated_err_corr_bit ^= (((c & 0x8b) & (1 << i)) != 0);   // 0b10001011
	MOVLW	0x01
	MOVWF	r0x07
	MOVLW	0x00
	MOVWF	r0x08
	MOVF	r0x06, W
	BZ	_00771_DS_
	NEGF	WREG
	BCF	STATUS, 0
_00772_DS_:
	RLCF	r0x07, F
	RLCF	r0x08, F
	ADDLW	0x01
	BNC	_00772_DS_
_00771_DS_:
	MOVF	r0x04, W
	ANDWF	r0x07, F
	MOVF	r0x05, W
	ANDWF	r0x08, F
	MOVF	r0x07, W
	BNZ	_00774_DS_
	MOVF	r0x08, W
	BNZ	_00774_DS_
	CLRF	r0x07
	INCF	r0x07, F
	BRA	_00775_DS_
_00774_DS_:
	CLRF	r0x07
_00775_DS_:
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
;	.line	931; meter_logger.c	for (i = 0; i < 8; i++) {
	INCF	r0x06, F
	MOVLW	0x08
	SUBWF	r0x06, W
	BNC	_00709_DS_
;	.line	934; meter_logger.c	calculated_err_corr |= calculated_err_corr_bit;
	MOVF	r0x02, W
	IORWF	r0x03, F
;	.line	945; meter_logger.c	if ((c >> 8) == calculated_err_corr) {
	MOVF	r0x01, W
	MOVWF	r0x00
	CLRF	r0x01
	CLRF	r0x02
	MOVF	r0x00, W
	XORWF	r0x03, W
	BNZ	_00779_DS_
	MOVF	r0x01, W
	XORWF	r0x02, W
	BZ	_00780_DS_
_00779_DS_:
	BRA	_00701_DS_
_00780_DS_:
;	.line	946; meter_logger.c	return 1;
	MOVLW	0x01
	BRA	_00711_DS_
_00701_DS_:
;	.line	949; meter_logger.c	return 0;
	CLRF	WREG
_00711_DS_:
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
;	.line	891; meter_logger.c	unsigned char reverse(unsigned char b) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVFF	r0x01, POSTDEC1
	MOVFF	r0x02, POSTDEC1
	MOVFF	r0x03, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
;	.line	893; meter_logger.c	c  = ((b >>  1) & 0x55) | ((b <<  1) & 0xaa);
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
;	.line	894; meter_logger.c	c |= ((b >>  2) & 0x33) | ((b <<  2) & 0xcc);
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
;	.line	895; meter_logger.c	c |= ((b >>  4) & 0x0f) | ((b <<  4) & 0xf0);
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
;	.line	896; meter_logger.c	return(c);
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
;	.line	861; meter_logger.c	void my_usart_open() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
;	.line	862; meter_logger.c	SPBRG = 103;					// 8MHz => 19230 baud
	MOVLW	0x67
	MOVWF	_SPBRG
;	.line	863; meter_logger.c	TXSTAbits.BRGH = 1;	// (0 = low speed)
	BSF	_TXSTAbits, 2
;	.line	864; meter_logger.c	TXSTAbits.SYNC = 0;	// (0 = asynchronous)
	BCF	_TXSTAbits, 4
;	.line	865; meter_logger.c	BAUDCONbits.BRG16 = 1;
	BSF	_BAUDCONbits, 3
;	.line	868; meter_logger.c	RCSTAbits.SPEN = 1; // (1 = serial port enabled)
	BSF	_RCSTAbits, 7
;	.line	871; meter_logger.c	PIE1bits.TXIE = 0; // (1 = enabled)
	BCF	_PIE1bits, 4
;	.line	872; meter_logger.c	IPR1bits.TXIP = 0; // USART Tx on low priority interrupt
	BCF	_IPR1bits, 4
;	.line	875; meter_logger.c	PIE1bits.RCIE = 1; // (1 = enabled)
	BSF	_PIE1bits, 5
;	.line	876; meter_logger.c	IPR1bits.RCIP = 0; // USART Rx on low priority interrupt
	BCF	_IPR1bits, 5
;	.line	879; meter_logger.c	TXSTAbits.TX9 = 0; // (0 = 8-bit transmit)
	BCF	_TXSTAbits, 6
;	.line	882; meter_logger.c	RCSTAbits.RX9 = 0; // (0 = 8-bit reception)
	BCF	_RCSTAbits, 6
;	.line	885; meter_logger.c	RCSTAbits.CREN = 1; // (1 = Enables receiver)
	BSF	_RCSTAbits, 4
;	.line	888; meter_logger.c	TXSTAbits.TXEN = 1; // (1 = transmit enabled)
	BSF	_TXSTAbits, 5
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__init_system	code
_init_system:
;	.line	762; meter_logger.c	void init_system() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
;	.line	764; meter_logger.c	TRIS_COMP1 = INPUT_STATE;		// as input
	BSF	_TRISAbits, 0
;	.line	765; meter_logger.c	TRIS_COMP2 = INPUT_STATE;		// as input
	BSF	_TRISAbits, 1
;	.line	767; meter_logger.c	TRIS_IR_PIN = INPUT_STATE;		// as input
	BSF	_TRISBbits, 0
;	.line	769; meter_logger.c	TRIS_LED_PIN = OUTPUT_STATE;	// as output
	BCF	_TRISBbits, 4
;	.line	770; meter_logger.c	LED_PIN = 0;					// and clear
	BCF	_PORTBbits, 4
;	.line	772; meter_logger.c	TRIS_IR_LED_PIN = OUTPUT_STATE;	// as output
	BCF	_TRISBbits, 1
;	.line	773; meter_logger.c	IR_LED_PIN = 0;					// and clear
	BCF	_PORTBbits, 1
;	.line	775; meter_logger.c	TRIS_DEBUG_PIN = OUTPUT_STATE;	// as output
	BCF	_TRISBbits, 2
;	.line	776; meter_logger.c	DEBUG_PIN = 0;					// and clear
	BCF	_PORTBbits, 2
;	.line	778; meter_logger.c	TRIS_DEBUG2_PIN = OUTPUT_STATE;	// as output
	BCF	_TRISBbits, 3
;	.line	779; meter_logger.c	DEBUG2_PIN = 0;					// and clear
	BCF	_PORTBbits, 3
;	.line	781; meter_logger.c	TRIS_DEBUG3_PIN = OUTPUT_STATE;	// as output
	BCF	_TRISBbits, 4
;	.line	782; meter_logger.c	DEBUG3_PIN = 0;					// and clear
	BCF	_PORTBbits, 4
;	.line	786; meter_logger.c	TRIS_PWM_PIN = OUTPUT_STATE;	// enable output from pwm module at init
	BCF	_TRISCbits, 1
;	.line	787; meter_logger.c	PWM_PIN = 0;					// and clear
	BCF	_PORTCbits, 1
;	.line	790; meter_logger.c	TRIS_RX_PIN = INPUT_STATE;		// as input
	BSF	_TRISCbits, 7
;	.line	791; meter_logger.c	TRIS_TX_PIN = OUTPUT_STATE;		// as input
	BCF	_TRISCbits, 6
;	.line	796; meter_logger.c	T1CONbits.TMR1ON = 1;
	BSF	_T1CONbits, 0
;	.line	797; meter_logger.c	T1CONbits.RD16 = 1;
	BSF	_T1CONbits, 7
;	.line	798; meter_logger.c	T1CONbits.TMR1CS = 0;   // internal clock source
	BCF	_T1CONbits, 1
;	.line	799; meter_logger.c	T1CONbits.T1OSCEN = 0;  // dont put t1 on pin
	BCF	_T1CONbits, 3
;	.line	800; meter_logger.c	T1CONbits.T1CKPS0 = 0;
	BCF	_T1CONbits, 4
;	.line	801; meter_logger.c	T1CONbits.T1CKPS1 = 0;
	BCF	_T1CONbits, 5
;	.line	802; meter_logger.c	IPR1bits.TMR1IP = 0;	// low priority
	BCF	_IPR1bits, 0
;	.line	803; meter_logger.c	PIE1bits.TMR1IE = 1;	// Ensure that TMR1 Interrupt is enabled
	BSF	_PIE1bits, 0
;	.line	804; meter_logger.c	PIR1bits.TMR1IF = 1;	// Force Instant entry to Timer 1 Interrupt
	BSF	_PIR1bits, 0
;	.line	835; meter_logger.c	RCONbits.IPEN = 1;
	BSF	_RCONbits, 7
;	.line	837; meter_logger.c	INTCONbits.INT0IE = 0;		// disable ext int, enabled when ir demodulator is started
	BCF	_INTCONbits, 4
;	.line	838; meter_logger.c	INTCON2bits.INTEDG0 = 1;	// rising edge
	BSF	_INTCON2bits, 6
;	.line	840; meter_logger.c	INTCONbits.PEIE = 1;
	BSF	_INTCONbits, 6
;	.line	841; meter_logger.c	INTCONbits.GIE = 1;	/* Enable Global interrupts   */	
	BSF	_INTCONbits, 7
;	.line	846; meter_logger.c	IPR1bits.RCIP = 0;
	BCF	_IPR1bits, 5
;	.line	847; meter_logger.c	IPR1bits.TXIP = 0;
	BCF	_IPR1bits, 4
;	.line	858; meter_logger.c	my_usart_open();
	CALL	_my_usart_open
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__sleep_ms	code
_sleep_ms:
;	.line	742; meter_logger.c	void sleep_ms(unsigned int ms) {
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
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
	MOVLW	0x03
	MOVFF	PLUSW2, r0x01
;	.line	745; meter_logger.c	start_timer_1_ms = timer_1_ms;	
	MOVFF	_timer_1_ms, r0x02
	MOVFF	(_timer_1_ms + 1), r0x03
;	.line	748; meter_logger.c	do {
	MOVF	r0x02, W
	SUBLW	0xff
	MOVWF	r0x04
	MOVLW	0xff
	SUBFWB	r0x03, W
	MOVWF	r0x05
_00664_DS_:
;	.line	749; meter_logger.c	if (start_timer_1_ms <= timer_1_ms) {
	MOVF	r0x03, W
	BANKSEL	(_timer_1_ms + 1)
	SUBWF	(_timer_1_ms + 1), W, B
	BNZ	_00675_DS_
	MOVF	r0x02, W
; removed redundant BANKSEL
	SUBWF	_timer_1_ms, W, B
_00675_DS_:
	BNC	_00662_DS_
;	.line	750; meter_logger.c	diff = timer_1_ms - start_timer_1_ms;
	MOVF	r0x02, W
	BANKSEL	_timer_1_ms
	SUBWF	_timer_1_ms, W, B
	MOVWF	r0x06
	MOVF	r0x03, W
; removed redundant BANKSEL
	SUBWFB	(_timer_1_ms + 1), W, B
	MOVWF	r0x07
	BRA	_00665_DS_
_00662_DS_:
	BANKSEL	_timer_1_ms
;	.line	754; meter_logger.c	diff = 0xffff - start_timer_1_ms + timer_1_ms;
	MOVF	_timer_1_ms, W, B
	ADDWF	r0x04, W
	MOVWF	r0x06
; removed redundant BANKSEL
	MOVF	(_timer_1_ms + 1), W, B
	ADDWFC	r0x05, W
	MOVWF	r0x07
_00665_DS_:
;	.line	756; meter_logger.c	} while (diff < ms);
	MOVF	r0x01, W
	SUBWF	r0x07, W
	BNZ	_00676_DS_
	MOVF	r0x00, W
	SUBWF	r0x06, W
_00676_DS_:
	BNC	_00664_DS_
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
;	.line	710; meter_logger.c	static void isr_low_prio(void) __interrupt 2 {
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
;	.line	713; meter_logger.c	if (PIR1bits.TMR1IF) {
	BTFSS	_PIR1bits, 0
	BRA	_00633_DS_
;	.line	714; meter_logger.c	TMR1H = (unsigned char)(TIMER1_RELOAD >> 8);    // 262,158ms @ 8MHz
	MOVLW	0xf8
	MOVWF	_TMR1H
;	.line	715; meter_logger.c	TMR1L = (unsigned char)TIMER1_RELOAD;
	MOVLW	0x53
	MOVWF	_TMR1L
;	.line	717; meter_logger.c	switch (led_flash.state) {
	MOVFF	_led_flash, r0x00
	MOVF	r0x00, W
	MOVWF	r0x01
	MOVF	r0x01, W
	BZ	_00627_DS_
	MOVF	r0x00, W
	XORLW	0x01
	BZ	_00628_DS_
	BRA	_00631_DS_
_00627_DS_:
;	.line	719; meter_logger.c	LED_PIN = 1;
	BSF	_PORTBbits, 4
;	.line	720; meter_logger.c	led_flash.state = LED_FLASH_RUNNING;
	MOVLW	0x01
	BANKSEL	_led_flash
	MOVWF	_led_flash, B
;	.line	721; meter_logger.c	break;
	BRA	_00631_DS_
_00628_DS_:
;	.line	723; meter_logger.c	if (led_flash.timer-- == 0) {
	MOVFF	(_led_flash + 1), r0x00
	DECF	r0x00, W
	MOVWF	r0x01
	MOVF	r0x01, W
	BANKSEL	(_led_flash + 1)
	MOVWF	(_led_flash + 1), B
	MOVF	r0x00, W
	BNZ	_00631_DS_
;	.line	724; meter_logger.c	LED_PIN = 0;
	BCF	_PORTBbits, 4
;	.line	725; meter_logger.c	led_flash.state = LED_FLASH_STOPPED;
	MOVLW	0x02
; removed redundant BANKSEL
	MOVWF	_led_flash, B
_00631_DS_:
	BANKSEL	_timer_1_ms
;	.line	729; meter_logger.c	timer_1_ms++;
	INCFSZ	_timer_1_ms, F, B
	BRA	_30963_DS_
; removed redundant BANKSEL
	INCF	(_timer_1_ms + 1), F, B
_30963_DS_:
;	.line	730; meter_logger.c	PIR1bits.TMR1IF = 0;    /* Clear the Timer Flag  */
	BCF	_PIR1bits, 0
_00633_DS_:
;	.line	734; meter_logger.c	if (usart_drdy()) {
	CALL	_usart_drdy
	MOVWF	r0x00
	MOVF	r0x00, W
	BZ	_00636_DS_
;	.line	736; meter_logger.c	c = usart_getc();
	CALL	_usart_getc
	MOVWF	r0x00
;	.line	737; meter_logger.c	usart_putc(c);
	MOVF	r0x00, W
	CALL	_usart_putc
_00636_DS_:
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
;	.line	351; meter_logger.c	static void isr_high_prio(void) __interrupt 1 {
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
;	.line	353; meter_logger.c	if (INTCONbits.INT0IF && INTCONbits.INT0IE) {
	BTFSS	_INTCONbits, 1
	BRA	_00300_DS_
	BTFSS	_INTCONbits, 4
	BRA	_00300_DS_
;	.line	354; meter_logger.c	timer_0 = (unsigned int)(TMR0L) | ((unsigned int)(TMR0H) << 8);
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
;	.line	355; meter_logger.c	TMR0H = (unsigned char)(timer0_reload >> 8);
	MOVF	(_timer0_reload + 1), W, B
	MOVWF	r0x00
	CLRF	r0x01
	MOVF	r0x00, W
	MOVWF	_TMR0H
; removed redundant BANKSEL
;	.line	356; meter_logger.c	TMR0L = (unsigned char)timer0_reload;
	MOVF	_timer0_reload, W, B
	MOVWF	_TMR0L
	BANKSEL	_codec_type
;	.line	358; meter_logger.c	switch (codec_type) {
	MOVF	_codec_type, W, B
	XORLW	0x01
	BZ	_00261_DS_
_00544_DS_:
	BANKSEL	_codec_type
	MOVF	_codec_type, W, B
	XORLW	0x02
	BNZ	_00546_DS_
	BRA	_00295_DS_
_00546_DS_:
	BRA	_00298_DS_
_00261_DS_:
;	.line	360; meter_logger.c	flash_led(100);
	MOVLW	0x64
	MOVWF	POSTDEC1
	CALL	_flash_led
	MOVF	POSTINC1, F
;	.line	361; meter_logger.c	switch (testo_ir_proto.state) {
	MOVFF	_testo_ir_proto, r0x00
	MOVF	r0x00, W
	MOVWF	r0x01
	MOVF	r0x01, W
	BZ	_00262_DS_
	MOVF	r0x00, W
	XORLW	0x02
	BZ	_00263_DS_
	MOVF	r0x00, W
	XORLW	0x04
	BNZ	_00552_DS_
	BRA	_00271_DS_
_00552_DS_:
	BRA	_00298_DS_
_00262_DS_:
;	.line	363; meter_logger.c	T0CONbits.TMR0ON = 1;		// Start TMR0
	BSF	_T0CONbits, 7
;	.line	364; meter_logger.c	testo_ir_proto.start_bit_len = 1;
	MOVLW	0x01
	BANKSEL	(_testo_ir_proto + 2)
	MOVWF	(_testo_ir_proto + 2), B
;	.line	365; meter_logger.c	testo_ir_proto.state = START_BIT_WAIT;
	MOVLW	0x02
; removed redundant BANKSEL
	MOVWF	_testo_ir_proto, B
;	.line	366; meter_logger.c	break;
	BRA	_00298_DS_
_00263_DS_:
	BANKSEL	_timer0_reload
;	.line	368; meter_logger.c	if ((TICK + timer0_reload - TICK_ADJ < timer_0) && (timer_0 < TICK + timer0_reload + TICK_ADJ)) {
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
	BNZ	_00553_DS_
; removed redundant BANKSEL
	MOVF	_timer_0, W, B
	SUBWF	r0x00, W
_00553_DS_:
	BC	_00268_DS_
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
	BNZ	_00554_DS_
	MOVF	r0x00, W
; removed redundant BANKSEL
	SUBWF	_timer_0, W, B
_00554_DS_:
	BC	_00268_DS_
;	.line	369; meter_logger.c	if (testo_ir_proto.start_bit_len < 2) {
	MOVLW	0x02
	BANKSEL	(_testo_ir_proto + 2)
	SUBWF	(_testo_ir_proto + 2), W, B
	BC	_00265_DS_
;	.line	370; meter_logger.c	testo_ir_proto.start_bit_len++;
	MOVFF	(_testo_ir_proto + 2), r0x00
	INCF	r0x00, F
	MOVF	r0x00, W
; removed redundant BANKSEL
	MOVWF	(_testo_ir_proto + 2), B
	BRA	_00298_DS_
_00265_DS_:
	BANKSEL	(_testo_ir_proto + 3)
;	.line	374; meter_logger.c	testo_ir_proto.data = 0;
	CLRF	(_testo_ir_proto + 3), B
; removed redundant BANKSEL
	CLRF	(_testo_ir_proto + 4), B
; removed redundant BANKSEL
;	.line	375; meter_logger.c	testo_ir_proto.data_len = 0;
	CLRF	(_testo_ir_proto + 5), B
;	.line	376; meter_logger.c	testo_ir_proto.state = DATA_WAIT;
	MOVLW	0x04
; removed redundant BANKSEL
	MOVWF	_testo_ir_proto, B
	BRA	_00298_DS_
_00268_DS_:
;	.line	381; meter_logger.c	testo_ir_proto.start_bit_len = 1;
	MOVLW	0x01
	BANKSEL	(_testo_ir_proto + 2)
	MOVWF	(_testo_ir_proto + 2), B
;	.line	382; meter_logger.c	testo_ir_proto.state = START_BIT_WAIT;
	MOVLW	0x02
; removed redundant BANKSEL
	MOVWF	_testo_ir_proto, B
;	.line	384; meter_logger.c	break;
	BRA	_00298_DS_
_00271_DS_:
;	.line	386; meter_logger.c	if (testo_ir_proto.data_len <= 12) {
	MOVLW	0x0d
	BANKSEL	(_testo_ir_proto + 5)
	SUBWF	(_testo_ir_proto + 5), W, B
	BTFSC	STATUS, 0
	BRA	_00298_DS_
	BANKSEL	_timer0_reload
;	.line	387; meter_logger.c	if (((TICK + timer0_reload - TICK_ADJ < timer_0) && (timer_0 < TICK + timer0_reload + TICK_ADJ)) || ((3 * TICK + timer0_reload - TICK_ADJ < timer_0) && (timer_0 < 3 * TICK + timer0_reload + TICK_ADJ))) {
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
	BNZ	_00557_DS_
; removed redundant BANKSEL
	MOVF	_timer_0, W, B
	SUBWF	r0x00, W
_00557_DS_:
	BC	_00287_DS_
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
	BNZ	_00558_DS_
	MOVF	r0x00, W
; removed redundant BANKSEL
	SUBWF	_timer_0, W, B
_00558_DS_:
	BNC	_00282_DS_
_00287_DS_:
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
	BNZ	_00559_DS_
; removed redundant BANKSEL
	MOVF	_timer_0, W, B
	SUBWF	r0x00, W
_00559_DS_:
	BC	_00283_DS_
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
	BNZ	_00560_DS_
	MOVF	r0x00, W
; removed redundant BANKSEL
	SUBWF	_timer_0, W, B
_00560_DS_:
	BC	_00283_DS_
_00282_DS_:
	BANKSEL	(_testo_ir_proto + 3)
;	.line	389; meter_logger.c	if ((testo_ir_proto.data & 1) != 0) {
	BTFSS	(_testo_ir_proto + 3), 0
	BRA	_00273_DS_
; removed redundant BANKSEL
;	.line	391; meter_logger.c	testo_ir_proto.data <<= 1;		// bitshift once to left
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
	BRA	_00274_DS_
_00273_DS_:
	BANKSEL	(_testo_ir_proto + 3)
;	.line	396; meter_logger.c	testo_ir_proto.data <<= 1;		// bitshift once to left
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
;	.line	397; meter_logger.c	testo_ir_proto.data |= 1;	// and set bit 0
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
_00274_DS_:
;	.line	399; meter_logger.c	testo_ir_proto.data_len++;
	MOVFF	(_testo_ir_proto + 5), r0x00
	INCF	r0x00, F
	MOVF	r0x00, W
	BANKSEL	(_testo_ir_proto + 5)
	MOVWF	(_testo_ir_proto + 5), B
	BRA	_00284_DS_
_00283_DS_:
	BANKSEL	_timer0_reload
;	.line	401; meter_logger.c	else if ((2 * TICK + timer0_reload - TICK_ADJ < timer_0) && (timer_0 < 2 * TICK + timer0_reload + TICK_ADJ)) {
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
	BNZ	_00563_DS_
; removed redundant BANKSEL
	MOVF	_timer_0, W, B
	SUBWF	r0x00, W
_00563_DS_:
	BC	_00279_DS_
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
	BNZ	_00564_DS_
	MOVF	r0x00, W
; removed redundant BANKSEL
	SUBWF	_timer_0, W, B
_00564_DS_:
	BC	_00279_DS_
	BANKSEL	(_testo_ir_proto + 3)
;	.line	403; meter_logger.c	if ((testo_ir_proto.data & 1) != 0) {
	BTFSS	(_testo_ir_proto + 3), 0
	BRA	_00276_DS_
; removed redundant BANKSEL
;	.line	405; meter_logger.c	testo_ir_proto.data <<= 1;		// bitshift once to left
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
;	.line	406; meter_logger.c	testo_ir_proto.data |= 1;	// and set bit 0
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
	BRA	_00277_DS_
_00276_DS_:
	BANKSEL	(_testo_ir_proto + 3)
;	.line	410; meter_logger.c	testo_ir_proto.data <<= 1;		// bitshift once to left
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
_00277_DS_:
;	.line	413; meter_logger.c	testo_ir_proto.data_len++;
	MOVFF	(_testo_ir_proto + 5), r0x00
	INCF	r0x00, F
	MOVF	r0x00, W
	BANKSEL	(_testo_ir_proto + 5)
	MOVWF	(_testo_ir_proto + 5), B
	BRA	_00284_DS_
_00279_DS_:
;	.line	418; meter_logger.c	testo_ir_proto.start_bit_len = 1;
	MOVLW	0x01
	BANKSEL	(_testo_ir_proto + 2)
	MOVWF	(_testo_ir_proto + 2), B
;	.line	419; meter_logger.c	testo_ir_proto.state = START_BIT_WAIT;
	MOVLW	0x02
; removed redundant BANKSEL
	MOVWF	_testo_ir_proto, B
_00284_DS_:
	BANKSEL	(_testo_ir_proto + 5)
;	.line	421; meter_logger.c	if (testo_ir_proto.data_len == 12) {
	MOVF	(_testo_ir_proto + 5), W, B
	XORLW	0x0c
	BNZ	_00298_DS_
_00568_DS_:
	BANKSEL	(_testo_ir_proto + 4)
;	.line	424; meter_logger.c	if (testo_valid_err_corr(testo_ir_proto.data & 0xffff)) {
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
	BZ	_00289_DS_
	BANKSEL	(_testo_ir_proto + 3)
;	.line	426; meter_logger.c	fifo_put(testo_ir_proto.data & 0xff);
	MOVF	(_testo_ir_proto + 3), W, B
	MOVWF	r0x00
	CLRF	r0x01
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	_fifo_put
	MOVF	POSTINC1, F
;	.line	427; meter_logger.c	LED_PIN = 1;
	BSF	_PORTBbits, 4
_00289_DS_:
	BANKSEL	_testo_ir_proto
;	.line	429; meter_logger.c	testo_ir_proto.state = INIT_STATE;
	CLRF	_testo_ir_proto, B
;	.line	434; meter_logger.c	break;
	BRA	_00298_DS_
_00295_DS_:
	BANKSEL	_rs232_proto
;	.line	436; meter_logger.c	switch (rs232_proto.state) {
	MOVF	_rs232_proto, W, B
	XORLW	0x02
	BNZ	_00298_DS_
;	.line	447; meter_logger.c	TMR0H = (unsigned char)(TIMER0_RS232_1200_START >> 8);
	MOVLW	0xf7
	MOVWF	_TMR0H
;	.line	448; meter_logger.c	TMR0L = (unsigned char)TIMER0_RS232_1200_START;
	CLRF	_TMR0L
;	.line	449; meter_logger.c	INTCONbits.INT0IE = 0;		// disable ext int while we are using timer to receive data bits
	BCF	_INTCONbits, 4
;	.line	450; meter_logger.c	T0CONbits.TMR0ON = 1;		// Start TMR0
	BSF	_T0CONbits, 7
;	.line	451; meter_logger.c	rs232_proto.state = DATA_WAIT;
	MOVLW	0x04
	BANKSEL	_rs232_proto
	MOVWF	_rs232_proto, B
_00298_DS_:
;	.line	456; meter_logger.c	INTCONbits.INT0IF = 0;	/* Clear Interrupt Flag */
	BCF	_INTCONbits, 1
_00300_DS_:
;	.line	460; meter_logger.c	if (INTCONbits.TMR0IF && INTCONbits.TMR0IE) {
	BTFSS	_INTCONbits, 2
	BRA	_00353_DS_
	BTFSS	_INTCONbits, 5
	BRA	_00353_DS_
	BANKSEL	(_timer0_reload + 1)
;	.line	462; meter_logger.c	TMR0H = (unsigned char)(timer0_reload >> 8);
	MOVF	(_timer0_reload + 1), W, B
	MOVWF	r0x00
	CLRF	r0x01
	MOVF	r0x00, W
	MOVWF	_TMR0H
; removed redundant BANKSEL
;	.line	463; meter_logger.c	TMR0L = (unsigned char)timer0_reload;
	MOVF	_timer0_reload, W, B
	MOVWF	_TMR0L
;	.line	465; meter_logger.c	switch (codec_type) {
	MOVLW	0x01
	BANKSEL	_codec_type
	SUBWF	_codec_type, W, B
	BTFSS	STATUS, 0
	BRA	_00351_DS_
	MOVLW	0x06
; removed redundant BANKSEL
	SUBWF	_codec_type, W, B
	BTFSC	STATUS, 0
	BRA	_00351_DS_
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
	ADDLW	LOW(_00574_DS_)
	MOVWF	POSTDEC1
	MOVLW	HIGH(_00574_DS_)
	ADDWFC	PCLATH, F
	MOVLW	UPPER(_00574_DS_)
	ADDWFC	PCLATU, F
	MOVF	PREINC1, W
	MOVWF	PCL
_00574_DS_:
	GOTO	_00302_DS_
	GOTO	_00314_DS_
	GOTO	_00303_DS_
	GOTO	_00324_DS_
	GOTO	_00335_DS_
_00302_DS_:
;	.line	467; meter_logger.c	T0CONbits.TMR0ON = 0;			// Stop TMR0
	BCF	_T0CONbits, 7
	BANKSEL	_testo_ir_proto
;	.line	468; meter_logger.c	testo_ir_proto.state = INIT_STATE;
	CLRF	_testo_ir_proto, B
	sleep 
;	.line	470; meter_logger.c	break;
	BRA	_00351_DS_
_00303_DS_:
;	.line	472; meter_logger.c	switch (rs232_proto.state) {
	MOVFF	_rs232_proto, r0x00
	MOVF	r0x00, W
	MOVWF	r0x01
	MOVF	r0x01, W
	BZ	_00304_DS_
	MOVF	r0x00, W
	XORLW	0x03
	BZ	_00307_DS_
	MOVF	r0x00, W
	XORLW	0x09
	BZ	_00311_DS_
	MOVF	r0x00, W
	XORLW	0x0a
	BZ	_00312_DS_
	BRA	_00351_DS_
_00304_DS_:
	BANKSEL	(_rs232_proto + 3)
;	.line	474; meter_logger.c	if (rs232_proto.data_len == 8) {
	MOVF	(_rs232_proto + 3), W, B
	XORLW	0x08
	BZ	_00584_DS_
	BRA	_00351_DS_
_00584_DS_:
;	.line	475; meter_logger.c	IR_LED_PIN = 1;		// inverted rs232 output on ir, start bit = ir light
	BSF	_PORTBbits, 1
;	.line	476; meter_logger.c	rs232_proto.state = START_BIT_SENT;
	MOVLW	0x03
	BANKSEL	_rs232_proto
	MOVWF	_rs232_proto, B
;	.line	478; meter_logger.c	break;
	BRA	_00351_DS_
_00307_DS_:
;	.line	480; meter_logger.c	if (rs232_proto.data_len >= 1) {
	MOVLW	0x01
	BANKSEL	(_rs232_proto + 3)
	SUBWF	(_rs232_proto + 3), W, B
	BNC	_00309_DS_
;	.line	481; meter_logger.c	IR_LED_PIN = (rs232_proto.data & 1) == 0;	// inverted rs232 output on ir
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
;	.line	482; meter_logger.c	rs232_proto.data = rs232_proto.data >> 1;
	RRNCF	(_rs232_proto + 2), W, B
	ANDLW	0x7f
	MOVWF	r0x00
	MOVF	r0x00, W
; removed redundant BANKSEL
	MOVWF	(_rs232_proto + 2), B
;	.line	483; meter_logger.c	rs232_proto.data_len--;
	MOVFF	(_rs232_proto + 3), r0x00
	DECF	r0x00, F
	MOVF	r0x00, W
; removed redundant BANKSEL
	MOVWF	(_rs232_proto + 3), B
	BRA	_00351_DS_
_00309_DS_:
;	.line	486; meter_logger.c	IR_LED_PIN = 0;								// inverted rs232 output on ir					
	BCF	_PORTBbits, 1
;	.line	487; meter_logger.c	rs232_proto.state = STOP_BIT_SENT;
	MOVLW	0x09
	BANKSEL	_rs232_proto
	MOVWF	_rs232_proto, B
;	.line	489; meter_logger.c	break;
	BRA	_00351_DS_
_00311_DS_:
;	.line	491; meter_logger.c	IR_LED_PIN = 0;									// inverted rs232 output on ir
	BCF	_PORTBbits, 1
;	.line	492; meter_logger.c	rs232_proto.state = STOP_BIT2_SENT;
	MOVLW	0x0a
	BANKSEL	_rs232_proto
	MOVWF	_rs232_proto, B
;	.line	493; meter_logger.c	break;
	BRA	_00351_DS_
_00312_DS_:
;	.line	495; meter_logger.c	IR_LED_PIN = 0;									// inverted rs232 output on ir
	BCF	_PORTBbits, 1
	BANKSEL	_rs232_proto
;	.line	496; meter_logger.c	rs232_proto.state = INIT_STATE;
	CLRF	_rs232_proto, B
;	.line	497; meter_logger.c	T0CONbits.TMR0ON = 0;							// stop timer 0
	BCF	_T0CONbits, 7
;	.line	500; meter_logger.c	break;
	BRA	_00351_DS_
_00314_DS_:
;	.line	502; meter_logger.c	switch (rs232_proto.state) {
	MOVFF	_rs232_proto, r0x00
	MOVF	r0x00, W
	XORLW	0x04
	BZ	_00315_DS_
	MOVF	r0x00, W
	XORLW	0x07
	BZ	_00322_DS_
	BRA	_00351_DS_
_00315_DS_:
;	.line	504; meter_logger.c	rs232_proto.data_len++;
	MOVFF	(_rs232_proto + 3), r0x00
	INCF	r0x00, F
	MOVF	r0x00, W
	BANKSEL	(_rs232_proto + 3)
	MOVWF	(_rs232_proto + 3), B
;	.line	505; meter_logger.c	if (rs232_proto.data_len <= 8) {
	MOVLW	0x09
; removed redundant BANKSEL
	SUBWF	(_rs232_proto + 3), W, B
	BC	_00320_DS_
;	.line	506; meter_logger.c	if (IR_PIN) {		
	BTFSS	_PORTBbits, 0
	BRA	_00317_DS_
; removed redundant BANKSEL
;	.line	508; meter_logger.c	rs232_proto.data >>= 1;
	RRNCF	(_rs232_proto + 2), W, B
	ANDLW	0x7f
	MOVWF	r0x00
	MOVF	r0x00, W
; removed redundant BANKSEL
	MOVWF	(_rs232_proto + 2), B
	BRA	_00351_DS_
_00317_DS_:
	BANKSEL	(_rs232_proto + 2)
;	.line	520; meter_logger.c	rs232_proto.data >>= 1;
	RRNCF	(_rs232_proto + 2), W, B
	ANDLW	0x7f
	MOVWF	r0x00
	MOVF	r0x00, W
; removed redundant BANKSEL
	MOVWF	(_rs232_proto + 2), B
;	.line	521; meter_logger.c	rs232_proto.data |= 0x80;
	MOVLW	0x80
; removed redundant BANKSEL
	IORWF	(_rs232_proto + 2), W, B
	MOVWF	r0x00
	MOVF	r0x00, W
; removed redundant BANKSEL
	MOVWF	(_rs232_proto + 2), B
	BRA	_00351_DS_
_00320_DS_:
;	.line	543; meter_logger.c	rs232_proto.state = STOP_BIT_WAIT;
	MOVLW	0x07
	BANKSEL	_rs232_proto
	MOVWF	_rs232_proto, B
;	.line	545; meter_logger.c	break;
	BRA	_00351_DS_
_00322_DS_:
	BANKSEL	(_rs232_proto + 2)
;	.line	551; meter_logger.c	fifo_put(rs232_proto.data);
	MOVF	(_rs232_proto + 2), W, B
	MOVWF	POSTDEC1
	CALL	_fifo_put
	MOVF	POSTINC1, F
	BANKSEL	(_rs232_proto + 2)
;	.line	552; meter_logger.c	rs232_proto.data = 0;
	CLRF	(_rs232_proto + 2), B
; removed redundant BANKSEL
;	.line	553; meter_logger.c	rs232_proto.data_len = 0;
	CLRF	(_rs232_proto + 3), B
;	.line	554; meter_logger.c	rs232_proto.state = START_BIT_WAIT;
	MOVLW	0x02
; removed redundant BANKSEL
	MOVWF	_rs232_proto, B
;	.line	555; meter_logger.c	T0CONbits.TMR0ON = 0;
	BCF	_T0CONbits, 7
;	.line	556; meter_logger.c	INTCONbits.INT0IF = 0;		// dont enter ext int now
	BCF	_INTCONbits, 1
;	.line	557; meter_logger.c	INTCONbits.INT0IE = 1;		// enable ext int again
	BSF	_INTCONbits, 4
;	.line	560; meter_logger.c	break;
	BRA	_00351_DS_
_00324_DS_:
;	.line	562; meter_logger.c	switch (fsk_proto.state) {
	MOVFF	_fsk_proto, r0x00
	MOVF	r0x00, W
	XORLW	0x04
	BZ	_00325_DS_
	MOVF	r0x00, W
	XORLW	0x07
	BZ	_00333_DS_
	BRA	_00351_DS_
_00325_DS_:
;	.line	564; meter_logger.c	fsk_proto.data_len++;						
	MOVFF	(_fsk_proto + 13), r0x00
	INCF	r0x00, F
	MOVF	r0x00, W
	BANKSEL	(_fsk_proto + 13)
	MOVWF	(_fsk_proto + 13), B
;	.line	565; meter_logger.c	if (fsk_proto.data_len <= 8) {
	MOVLW	0x09
; removed redundant BANKSEL
	SUBWF	(_fsk_proto + 13), W, B
	BC	_00331_DS_
;	.line	566; meter_logger.c	if ((fsk_proto.diff > 340) && (fsk_proto.diff < 476)) {
	MOVLW	0x01
; removed redundant BANKSEL
	SUBWF	(_fsk_proto + 2), W, B
	BNZ	_00598_DS_
	MOVLW	0x55
; removed redundant BANKSEL
	SUBWF	(_fsk_proto + 1), W, B
_00598_DS_:
	BNC	_00327_DS_
	MOVLW	0x01
	BANKSEL	(_fsk_proto + 2)
	SUBWF	(_fsk_proto + 2), W, B
	BNZ	_00599_DS_
	MOVLW	0xdc
; removed redundant BANKSEL
	SUBWF	(_fsk_proto + 1), W, B
_00599_DS_:
	BC	_00327_DS_
	BANKSEL	(_fsk_proto + 12)
;	.line	569; meter_logger.c	fsk_proto.data >>= 1;
	RRNCF	(_fsk_proto + 12), W, B
	ANDLW	0x7f
	MOVWF	r0x00
	MOVF	r0x00, W
; removed redundant BANKSEL
	MOVWF	(_fsk_proto + 12), B
	BRA	_00351_DS_
_00327_DS_:
	BANKSEL	(_fsk_proto + 12)
;	.line	574; meter_logger.c	fsk_proto.data >>= 1;
	RRNCF	(_fsk_proto + 12), W, B
	ANDLW	0x7f
	MOVWF	r0x00
	MOVF	r0x00, W
; removed redundant BANKSEL
	MOVWF	(_fsk_proto + 12), B
;	.line	575; meter_logger.c	fsk_proto.data |= 0x80;
	MOVLW	0x80
; removed redundant BANKSEL
	IORWF	(_fsk_proto + 12), W, B
	MOVWF	r0x00
	MOVF	r0x00, W
; removed redundant BANKSEL
	MOVWF	(_fsk_proto + 12), B
	BRA	_00351_DS_
_00331_DS_:
;	.line	583; meter_logger.c	fsk_proto.state = STOP_BIT_WAIT;
	MOVLW	0x07
	BANKSEL	_fsk_proto
	MOVWF	_fsk_proto, B
;	.line	585; meter_logger.c	break;
	BRA	_00351_DS_
_00333_DS_:
	BANKSEL	(_fsk_proto + 12)
;	.line	588; meter_logger.c	fifo_put(fsk_proto.data);
	MOVF	(_fsk_proto + 12), W, B
	MOVWF	POSTDEC1
	CALL	_fifo_put
	MOVF	POSTINC1, F
	BANKSEL	(_fsk_proto + 12)
;	.line	589; meter_logger.c	fsk_proto.data = 0;
	CLRF	(_fsk_proto + 12), B
;	.line	590; meter_logger.c	fsk_proto.state = START_BIT_WAIT;
	MOVLW	0x02
; removed redundant BANKSEL
	MOVWF	_fsk_proto, B
;	.line	592; meter_logger.c	INTCONbits.TMR0IE = 0;						
	BCF	_INTCONbits, 5
;	.line	595; meter_logger.c	break;
	BRA	_00351_DS_
_00335_DS_:
;	.line	597; meter_logger.c	switch (fsk_proto.state) {
	MOVFF	_fsk_proto, r0x00
	MOVF	r0x00, W
	MOVWF	r0x01
	MOVF	r0x01, W
	BZ	_00336_DS_
	MOVF	r0x00, W
	XORLW	0x01
	BZ	_00339_DS_
	MOVF	r0x00, W
	XORLW	0x03
	BZ	_00340_DS_
	MOVF	r0x00, W
	XORLW	0x05
	BNZ	_00608_DS_
	BRA	_00348_DS_
_00608_DS_:
	MOVF	r0x00, W
	XORLW	0x09
	BNZ	_00610_DS_
	BRA	_00349_DS_
_00610_DS_:
	BRA	_00351_DS_
_00336_DS_:
	BANKSEL	(_fsk_proto + 13)
;	.line	600; meter_logger.c	if (fsk_proto.data_len == 8) {
	MOVF	(_fsk_proto + 13), W, B
	XORLW	0x08
	BNZ	_00338_DS_
;	.line	601; meter_logger.c	fsk_proto.state = IDLE;
	MOVLW	0x01
	BANKSEL	_fsk_proto
	MOVWF	_fsk_proto, B
_00338_DS_:
;	.line	604; meter_logger.c	DEBUG_PIN = 0;
	BCF	_PORTBbits, 2
;	.line	606; meter_logger.c	break;
	BRA	_00351_DS_
_00339_DS_:
;	.line	608; meter_logger.c	send_fsk_low();
	CALL	_send_fsk_low
;	.line	609; meter_logger.c	fsk_proto.state = START_BIT_SENT;
	MOVLW	0x03
	BANKSEL	_fsk_proto
	MOVWF	_fsk_proto, B
;	.line	611; meter_logger.c	DEBUG_PIN = 1;
	BSF	_PORTBbits, 2
;	.line	613; meter_logger.c	break;
	BRA	_00351_DS_
_00340_DS_:
;	.line	615; meter_logger.c	if (fsk_proto.data_len--) {
	MOVFF	(_fsk_proto + 13), r0x00
	DECF	r0x00, W
	MOVWF	r0x01
	MOVF	r0x01, W
	BANKSEL	(_fsk_proto + 13)
	MOVWF	(_fsk_proto + 13), B
	MOVF	r0x00, W
	BZ	_00345_DS_
;	.line	616; meter_logger.c	if (fsk_proto.data & (0x80 >> fsk_proto.data_len)) {
	MOVLW	0x80
	MOVWF	r0x00
; removed redundant BANKSEL
	MOVF	(_fsk_proto + 13), W, B
	BZ	_00613_DS_
	NEGF	WREG
	BCF	STATUS, 0
_00614_DS_:
	RRCF	r0x00, F
	ADDLW	0x01
	BNC	_00614_DS_
_00613_DS_:
	BANKSEL	(_fsk_proto + 12)
	MOVF	(_fsk_proto + 12), W, B
	ANDWF	r0x00, F
	MOVF	r0x00, W
	BZ	_00342_DS_
;	.line	617; meter_logger.c	send_fsk_high();
	CALL	_send_fsk_high
;	.line	619; meter_logger.c	DEBUG_PIN = 0;
	BCF	_PORTBbits, 2
	BRA	_00345_DS_
_00342_DS_:
;	.line	623; meter_logger.c	send_fsk_low();
	CALL	_send_fsk_low
;	.line	625; meter_logger.c	DEBUG_PIN = 1;
	BSF	_PORTBbits, 2
_00345_DS_:
	BANKSEL	(_fsk_proto + 13)
;	.line	629; meter_logger.c	if (fsk_proto.data_len == 0) {
	MOVF	(_fsk_proto + 13), W, B
	BNZ	_00351_DS_
;	.line	630; meter_logger.c	fsk_proto.state = DATA_SENT;
	MOVLW	0x05
; removed redundant BANKSEL
	MOVWF	_fsk_proto, B
;	.line	632; meter_logger.c	break;
	BRA	_00351_DS_
_00348_DS_:
;	.line	634; meter_logger.c	send_fsk_high();
	CALL	_send_fsk_high
;	.line	635; meter_logger.c	fsk_proto.state = STOP_BIT_SENT;
	MOVLW	0x09
	BANKSEL	_fsk_proto
	MOVWF	_fsk_proto, B
;	.line	637; meter_logger.c	DEBUG_PIN = 0;
	BCF	_PORTBbits, 2
;	.line	639; meter_logger.c	break;
	BRA	_00351_DS_
_00349_DS_:
;	.line	641; meter_logger.c	send_fsk_high();
	CALL	_send_fsk_high
	BANKSEL	_fsk_proto
;	.line	642; meter_logger.c	fsk_proto.state = INIT_STATE;
	CLRF	_fsk_proto, B
;	.line	644; meter_logger.c	DEBUG_PIN = 0;
	BCF	_PORTBbits, 2
_00351_DS_:
;	.line	651; meter_logger.c	INTCONbits.TMR0IF = 0;
	BCF	_INTCONbits, 2
_00353_DS_:
;	.line	654; meter_logger.c	if (PIR2bits.CMIF && PIE2bits.CMIE) {
	BTFSS	_PIR2bits, 6
	BRA	_00372_DS_
	BTFSS	_PIE2bits, 6
	BRA	_00372_DS_
;	.line	656; meter_logger.c	if (CMCONbits.C1OUT) {		// rising edge
	BTFSS	_CMCONbits, 6
	BRA	_00367_DS_
;	.line	657; meter_logger.c	timer_0 = (unsigned int)(TMR0L) | ((unsigned int)(TMR0H) << 8);
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
;	.line	662; meter_logger.c	DEBUG_PIN = 1;
	BSF	_PORTBbits, 2
	BANKSEL	_last_timer_0
;	.line	664; meter_logger.c	fsk_proto.diff = timer_0 - last_timer_0;
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
;	.line	665; meter_logger.c	last_timer_0 = timer_0;
	MOVFF	_timer_0, _last_timer_0
	MOVFF	(_timer_0 + 1), (_last_timer_0 + 1)
;	.line	667; meter_logger.c	if ((fsk_proto.diff > 340) && (fsk_proto.diff < 476)) {
	MOVLW	0x01
; removed redundant BANKSEL
	SUBWF	(_fsk_proto + 2), W, B
	BNZ	_00616_DS_
	MOVLW	0x55
; removed redundant BANKSEL
	SUBWF	(_fsk_proto + 1), W, B
_00616_DS_:
	BNC	_00363_DS_
	MOVLW	0x01
	BANKSEL	(_fsk_proto + 2)
	SUBWF	(_fsk_proto + 2), W, B
	BNZ	_00617_DS_
	MOVLW	0xdc
; removed redundant BANKSEL
	SUBWF	(_fsk_proto + 1), W, B
_00617_DS_:
	BC	_00363_DS_
	BANKSEL	(_fsk_proto + 1)
;	.line	668; meter_logger.c	fsk_proto.low_count += fsk_proto.diff;
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
;	.line	669; meter_logger.c	if (fsk_proto.state == START_BIT_WAIT) {
	MOVF	_fsk_proto, W, B
	XORLW	0x02
	BNZ	_00368_DS_
;	.line	670; meter_logger.c	if (fsk_proto.low_count >= 800) {								// start bit received
	MOVLW	0x03
	BANKSEL	(_fsk_proto + 6)
	SUBWF	(_fsk_proto + 6), W, B
	BNZ	_00620_DS_
	MOVLW	0x20
; removed redundant BANKSEL
	SUBWF	(_fsk_proto + 5), W, B
_00620_DS_:
	BNC	_00368_DS_
	BANKSEL	(_timer0_reload + 1)
;	.line	672; meter_logger.c	TMR0H = (unsigned char)(timer0_reload >> 8);
	MOVF	(_timer0_reload + 1), W, B
	MOVWF	r0x00
	CLRF	r0x01
	MOVF	r0x00, W
	MOVWF	_TMR0H
; removed redundant BANKSEL
;	.line	673; meter_logger.c	TMR0L = (unsigned char)timer0_reload;
	MOVF	_timer0_reload, W, B
	MOVWF	_TMR0L
	BANKSEL	(_fsk_proto + 5)
;	.line	674; meter_logger.c	fsk_proto.low_count = 0;
	CLRF	(_fsk_proto + 5), B
; removed redundant BANKSEL
	CLRF	(_fsk_proto + 6), B
; removed redundant BANKSEL
;	.line	675; meter_logger.c	fsk_proto.high_count = 0;
	CLRF	(_fsk_proto + 7), B
; removed redundant BANKSEL
	CLRF	(_fsk_proto + 8), B
; removed redundant BANKSEL
;	.line	677; meter_logger.c	fsk_proto.data_len = 0;
	CLRF	(_fsk_proto + 13), B
; removed redundant BANKSEL
;	.line	678; meter_logger.c	fsk_proto.data = 0;
	CLRF	(_fsk_proto + 12), B
;	.line	679; meter_logger.c	fsk_proto.state = DATA_WAIT;
	MOVLW	0x04
; removed redundant BANKSEL
	MOVWF	_fsk_proto, B
;	.line	680; meter_logger.c	INTCONbits.TMR0IF = 0;		// clear flag so it dont enter isr now
	BCF	_INTCONbits, 2
;	.line	681; meter_logger.c	INTCONbits.TMR0IE = 1;		// Enable TMR0 Interrupt
	BSF	_INTCONbits, 5
	BRA	_00368_DS_
_00363_DS_:
	BANKSEL	_fsk_proto
;	.line	687; meter_logger.c	if (fsk_proto.state == START_BIT_WAIT) {
	MOVF	_fsk_proto, W, B
	XORLW	0x02
	BNZ	_00360_DS_
_00622_DS_:
	BANKSEL	(_fsk_proto + 5)
;	.line	688; meter_logger.c	fsk_proto.low_count = 0;
	CLRF	(_fsk_proto + 5), B
; removed redundant BANKSEL
	CLRF	(_fsk_proto + 6), B
; removed redundant BANKSEL
;	.line	689; meter_logger.c	fsk_proto.high_count = 0;
	CLRF	(_fsk_proto + 7), B
; removed redundant BANKSEL
	CLRF	(_fsk_proto + 8), B
	BRA	_00368_DS_
_00360_DS_:
	BANKSEL	(_fsk_proto + 1)
;	.line	692; meter_logger.c	fsk_proto.high_count += fsk_proto.diff;
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
	BRA	_00368_DS_
_00367_DS_:
;	.line	698; meter_logger.c	DEBUG_PIN = 0;
	BCF	_PORTBbits, 2
_00368_DS_:
;	.line	702; meter_logger.c	PIR2bits.CMIF = 0;
	BCF	_PIR2bits, 6
_00372_DS_:
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
	DB	0x25, 0x64, 0x20, 0x00
; ; Starting pCode block
___str_4:
	DB	0x0a, 0x0d, 0x77, 0x61, 0x69, 0x74, 0x69, 0x6e, 0x67, 0x20, 0x66, 0x6f
	DB	0x72, 0x20, 0x6e, 0x65, 0x77, 0x20, 0x63, 0x6f, 0x6d, 0x6d, 0x61, 0x6e
	DB	0x64, 0x0a, 0x0d, 0x00
; ; Starting pCode block
___str_5:
	DB	0x0a, 0x0d, 0x65, 0x63, 0x68, 0x6f, 0x20, 0x74, 0x65, 0x73, 0x74, 0x20
	DB	0x2d, 0x20, 0x73, 0x65, 0x6e, 0x64, 0x20, 0x73, 0x6f, 0x6d, 0x65, 0x20
	DB	0x64, 0x61, 0x74, 0x61, 0x0a, 0x0d, 0x00
; ; Starting pCode block
___str_6:
	DB	0x0a, 0x0d, 0x6b, 0x61, 0x6d, 0x73, 0x74, 0x72, 0x75, 0x70, 0x20, 0x2d
	DB	0x20, 0x73, 0x65, 0x6e, 0x64, 0x20, 0x6b, 0x6d, 0x70, 0x20, 0x66, 0x72
	DB	0x61, 0x6d, 0x65, 0x20, 0x64, 0x61, 0x74, 0x61, 0x0a, 0x0d, 0x00
; ; Starting pCode block
___str_7:
	DB	0x0a, 0x0d, 0x6b, 0x61, 0x6d, 0x73, 0x74, 0x72, 0x75, 0x70, 0x20, 0x2d
	DB	0x20, 0x6b, 0x6d, 0x70, 0x20, 0x66, 0x72, 0x61, 0x6d, 0x65, 0x20, 0x72
	DB	0x65, 0x63, 0x65, 0x69, 0x76, 0x65, 0x64, 0x3a, 0x0a, 0x0d, 0x00
; ; Starting pCode block
___str_8:
	DB	0x0a, 0x0d, 0x6b, 0x61, 0x6d, 0x73, 0x74, 0x72, 0x75, 0x70, 0x20, 0x2d
	DB	0x20, 0x6b, 0x6d, 0x70, 0x20, 0x72, 0x65, 0x70, 0x6c, 0x79, 0x20, 0x72
	DB	0x65, 0x63, 0x65, 0x69, 0x76, 0x65, 0x64, 0x3a, 0x0a, 0x0d, 0x00
; ; Starting pCode block
___str_9:
	DB	0x0a, 0x0d, 0x6e, 0x6f, 0x20, 0x72, 0x65, 0x70, 0x6c, 0x79, 0x20, 0x66
	DB	0x72, 0x6f, 0x6d, 0x20, 0x6d, 0x65, 0x74, 0x65, 0x72, 0x0a, 0x0d, 0x00


; Statistics:
; code size:	13032 (0x32e8) bytes ( 9.94%)
;           	 6516 (0x1974) words
; udata size:	 1198 (0x04ae) bytes (66.85%)
; access size:	   14 (0x000e) bytes


	end
