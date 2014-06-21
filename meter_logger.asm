;--------------------------------------------------------
; File Created by SDCC : free open source ANSI-C Compiler
; Version 3.3.0 #8604 (Oct 27 2013) (Mac OS X x86_64)
; This file was generated Sat Jun 21 03:19:29 2014
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
	global	_get_dev_id
	global	_my_usart_open
	global	_reverse
	global	_testo_valid_err_corr
	global	_testo_ir_enable
	global	_testo_ir_disable
	global	_rs232_8n2_tx_enable
	global	_rs232_8n2_tx_disable
	global	_rs232_8n2_rx_enable
	global	_rs232_8n2_rx_disable
	global	_rs232_8n2_tx_byte
	global	_rs232_7e1_tx_enable
	global	_rs232_7e1_tx_disable
	global	_rs232_7e1_rx_enable
	global	_rs232_7e1_rx_disable
	global	_rs232_7e1_tx_byte
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
	global	_get_battery_level
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
	extern	_adc_open
	extern	_adc_close
	extern	_adc_conv
	extern	_adc_busy
	extern	_adc_read
	extern	_adc_setchannel
	extern	__mullong
	extern	__divulong

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
_debug_buffer	res	128

udata_meter_logger_5	udata
_main_cmd_1_90	res	1

udata_meter_logger_6	udata
_main_sub_cmd_1_90	res	1

udata_meter_logger_7	udata
_timer_0	res	2

udata_meter_logger_8	udata
_timer0_reload	res	2

udata_meter_logger_9	udata
_testo_ir_proto	res	6

udata_meter_logger_10	udata
_rs232_proto	res	7

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
;	.line	121; meter_logger.c	OSCCONbits.SCS = 0x10;
	MOVF	_OSCCONbits, W
	ANDLW	0xfc
	MOVWF	_OSCCONbits
;	.line	123; meter_logger.c	OSCCONbits.IRCF = 0x7;	// 8 MHz
	MOVF	_OSCCONbits, W
	ANDLW	0x8f
	IORLW	0x70
	MOVWF	_OSCCONbits
	BANKSEL	_timer_1_ms
;	.line	126; meter_logger.c	timer_1_ms = 0;
	CLRF	_timer_1_ms, B
; removed redundant BANKSEL
	CLRF	(_timer_1_ms + 1), B
	BANKSEL	_fifo_head
;	.line	128; meter_logger.c	fifo_head = 0;
	CLRF	_fifo_head, B
; removed redundant BANKSEL
	CLRF	(_fifo_head + 1), B
	BANKSEL	_fifo_tail
;	.line	129; meter_logger.c	fifo_tail = 0;
	CLRF	_fifo_tail, B
; removed redundant BANKSEL
	CLRF	(_fifo_tail + 1), B
;	.line	131; meter_logger.c	init_system();	
	CALL	_init_system
;	.line	132; meter_logger.c	sleep_ms(100);
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x64
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVF	POSTINC1, F
	MOVF	POSTINC1, F
;	.line	139; meter_logger.c	get_battery_level();
	CALL	_get_battery_level
;	.line	142; meter_logger.c	dev_id = get_dev_id();
	CALL	_get_dev_id
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
;	.line	144; meter_logger.c	if (dev_id == 0x1240) {
	MOVF	r0x00, W
	XORLW	0x40
	BNZ	_00334_DS_
	MOVF	r0x01, W
	XORLW	0x12
	BZ	_00335_DS_
_00334_DS_:
	BRA	_00109_DS_
_00335_DS_:
;	.line	145; meter_logger.c	sprintf(debug_buffer, "Processor: pic18f2550\n\r");
	MOVLW	UPPER(__str_0)
	MOVWF	r0x04
	MOVLW	HIGH(__str_0)
	MOVWF	r0x03
	MOVLW	LOW(__str_0)
	MOVWF	r0x02
	MOVLW	HIGH(_debug_buffer)
	MOVWF	r0x06
	MOVLW	LOW(_debug_buffer)
	MOVWF	r0x05
	MOVLW	0x80
	MOVWF	r0x07
	MOVF	r0x04, W
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x07, W
	MOVWF	POSTDEC1
	MOVF	r0x06, W
	MOVWF	POSTDEC1
	MOVF	r0x05, W
	MOVWF	POSTDEC1
	CALL	_sprintf
	MOVLW	0x06
	ADDWF	FSR1L, F
	BRA	_00110_DS_
_00109_DS_:
;	.line	147; meter_logger.c	else if (dev_id == 0x2a40) {
	MOVF	r0x00, W
	XORLW	0x40
	BNZ	_00336_DS_
	MOVF	r0x01, W
	XORLW	0x2a
	BZ	_00337_DS_
_00336_DS_:
	BRA	_00106_DS_
_00337_DS_:
;	.line	148; meter_logger.c	sprintf(debug_buffer, "Processor: pic18f2553\n\r");
	MOVLW	UPPER(__str_1)
	MOVWF	r0x04
	MOVLW	HIGH(__str_1)
	MOVWF	r0x03
	MOVLW	LOW(__str_1)
	MOVWF	r0x02
	MOVLW	HIGH(_debug_buffer)
	MOVWF	r0x06
	MOVLW	LOW(_debug_buffer)
	MOVWF	r0x05
	MOVLW	0x80
	MOVWF	r0x07
	MOVF	r0x04, W
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x07, W
	MOVWF	POSTDEC1
	MOVF	r0x06, W
	MOVWF	POSTDEC1
	MOVF	r0x05, W
	MOVWF	POSTDEC1
	CALL	_sprintf
	MOVLW	0x06
	ADDWF	FSR1L, F
	BRA	_00110_DS_
_00106_DS_:
;	.line	151; meter_logger.c	sprintf(debug_buffer, "Processor: unsupported, device id: 0x%04x\n\r", dev_id);
	MOVLW	UPPER(__str_2)
	MOVWF	r0x04
	MOVLW	HIGH(__str_2)
	MOVWF	r0x03
	MOVLW	LOW(__str_2)
	MOVWF	r0x02
	MOVLW	HIGH(_debug_buffer)
	MOVWF	r0x06
	MOVLW	LOW(_debug_buffer)
	MOVWF	r0x05
	MOVLW	0x80
	MOVWF	r0x07
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	MOVF	r0x04, W
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x07, W
	MOVWF	POSTDEC1
	MOVF	r0x06, W
	MOVWF	POSTDEC1
	MOVF	r0x05, W
	MOVWF	POSTDEC1
	CALL	_sprintf
	MOVLW	0x08
	ADDWF	FSR1L, F
_00110_DS_:
;	.line	153; meter_logger.c	usart_puts(debug_buffer);
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
;	.line	156; meter_logger.c	fsk_rx_enable();
	CALL	_fsk_rx_enable
_00171_DS_:
;	.line	158; meter_logger.c	if (fifo_get(&cmd)) {
	MOVLW	HIGH(_main_cmd_1_90)
	MOVWF	r0x01
	MOVLW	LOW(_main_cmd_1_90)
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
	BZ	_00171_DS_
;	.line	159; meter_logger.c	switch (cmd) {
	MOVLW	0xfb
	BANKSEL	_main_cmd_1_90
	SUBWF	_main_cmd_1_90, W, B
	BTFSS	STATUS, 0
	GOTO	_00165_DS_
	MOVLW	0x05
; removed redundant BANKSEL
	ADDWF	_main_cmd_1_90, W, B
	MOVWF	r0x00
	CLRF	PCLATH
	CLRF	PCLATU
	RLCF	r0x00, W
	RLCF	PCLATH, F
	RLCF	WREG, W
	RLCF	PCLATH, F
	ANDLW	0xfc
	ADDLW	LOW(_00339_DS_)
	MOVWF	POSTDEC1
	MOVLW	HIGH(_00339_DS_)
	ADDWFC	PCLATH, F
	MOVLW	UPPER(_00339_DS_)
	ADDWFC	PCLATU, F
	MOVF	PREINC1, W
	MOVWF	PCL
_00339_DS_:
	GOTO	_00163_DS_
	GOTO	_00145_DS_
	GOTO	_00127_DS_
	GOTO	_00111_DS_
	GOTO	_00119_DS_
_00111_DS_:
;	.line	161; meter_logger.c	fsk_rx_disable();
	CALL	_fsk_rx_disable
;	.line	162; meter_logger.c	usart_puts("\n\rpress print on testo\n\r");
	MOVLW	UPPER(__str_3)
	MOVWF	r0x02
	MOVLW	HIGH(__str_3)
	MOVWF	r0x01
	MOVLW	LOW(__str_3)
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
;	.line	163; meter_logger.c	testo_ir_enable();
	CALL	_testo_ir_enable
;	.line	165; meter_logger.c	last_fifo_size = 0;
	CLRF	r0x00
	CLRF	r0x01
;	.line	166; meter_logger.c	sleep_ms(10000);						// 10 seconds to start printing
	MOVLW	0x27
	MOVWF	POSTDEC1
	MOVLW	0x10
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVF	POSTINC1, F
	MOVF	POSTINC1, F
;	.line	167; meter_logger.c	fifo_size = fifo_in_use();
	CALL	_fifo_in_use
	MOVWF	r0x02
	MOVFF	PRODL, r0x03
_00112_DS_:
;	.line	168; meter_logger.c	while (fifo_size > last_fifo_size) {	// and wait while we are still receiving data
	MOVF	r0x03, W
	SUBWF	r0x01, W
	BNZ	_00340_DS_
	MOVF	r0x02, W
	SUBWF	r0x00, W
_00340_DS_:
	BC	_00114_DS_
;	.line	169; meter_logger.c	last_fifo_size = fifo_size;
	MOVFF	r0x02, r0x00
	MOVFF	r0x03, r0x01
;	.line	170; meter_logger.c	sleep_ms(200);						// return data when no data for 200 ms
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0xc8
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVF	POSTINC1, F
	MOVF	POSTINC1, F
;	.line	171; meter_logger.c	fifo_size = fifo_in_use();
	CALL	_fifo_in_use
	MOVWF	r0x02
	MOVFF	PRODL, r0x03
	BRA	_00112_DS_
_00114_DS_:
;	.line	173; meter_logger.c	testo_ir_disable();
	CALL	_testo_ir_disable
;	.line	179; meter_logger.c	sprintf(debug_buffer, "<- ");
	MOVLW	UPPER(__str_4)
	MOVWF	r0x06
	MOVLW	HIGH(__str_4)
	MOVWF	r0x05
	MOVLW	LOW(__str_4)
	MOVWF	r0x04
	MOVLW	HIGH(_debug_buffer)
	MOVWF	r0x08
	MOVLW	LOW(_debug_buffer)
	MOVWF	r0x07
	MOVLW	0x80
	MOVWF	r0x09
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
	MOVLW	0x06
	ADDWF	FSR1L, F
;	.line	180; meter_logger.c	usart_puts(debug_buffer);
	MOVLW	HIGH(_debug_buffer)
	MOVWF	r0x05
	MOVLW	LOW(_debug_buffer)
	MOVWF	r0x04
	MOVLW	0x80
	MOVWF	r0x06
	MOVF	r0x06, W
	MOVWF	POSTDEC1
	MOVF	r0x05, W
	MOVWF	POSTDEC1
	MOVF	r0x04, W
	MOVWF	POSTDEC1
	CALL	_usart_puts
	MOVLW	0x03
	ADDWF	FSR1L, F
;	.line	181; meter_logger.c	for (i = 0; i < fifo_in_use(); i++) {
	CLRF	r0x04
	CLRF	r0x05
_00174_DS_:
	CALL	_fifo_in_use
	MOVWF	r0x06
	MOVFF	PRODL, r0x07
	MOVF	r0x07, W
	SUBWF	r0x05, W
	BNZ	_00341_DS_
	MOVF	r0x06, W
	SUBWF	r0x04, W
_00341_DS_:
	BTFSC	STATUS, 0
	BRA	_00115_DS_
;	.line	183; meter_logger.c	fifo_get(&sub_cmd);
	MOVLW	HIGH(_main_sub_cmd_1_90)
	MOVWF	r0x07
	MOVLW	LOW(_main_sub_cmd_1_90)
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
;	.line	185; meter_logger.c	sprintf(debug_buffer, "%d ", sub_cmd);
	MOVFF	_main_sub_cmd_1_90, r0x06
	CLRF	r0x07
	MOVLW	UPPER(__str_5)
	MOVWF	r0x0a
	MOVLW	HIGH(__str_5)
	MOVWF	r0x09
	MOVLW	LOW(__str_5)
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
;	.line	186; meter_logger.c	usart_puts(debug_buffer);
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
	BANKSEL	_main_sub_cmd_1_90
;	.line	188; meter_logger.c	fifo_put(sub_cmd);
	MOVF	_main_sub_cmd_1_90, W, B
	MOVWF	POSTDEC1
	CALL	_fifo_put
	MOVF	POSTINC1, F
;	.line	181; meter_logger.c	for (i = 0; i < fifo_in_use(); i++) {
	INFSNZ	r0x04, F
	INCF	r0x05, F
	BRA	_00174_DS_
_00115_DS_:
;	.line	190; meter_logger.c	sprintf(debug_buffer, "\n\r");
	MOVLW	UPPER(__str_6)
	MOVWF	r0x06
	MOVLW	HIGH(__str_6)
	MOVWF	r0x05
	MOVLW	LOW(__str_6)
	MOVWF	r0x04
	MOVLW	HIGH(_debug_buffer)
	MOVWF	r0x08
	MOVLW	LOW(_debug_buffer)
	MOVWF	r0x07
	MOVLW	0x80
	MOVWF	r0x09
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
	MOVLW	0x06
	ADDWF	FSR1L, F
;	.line	191; meter_logger.c	usart_puts(debug_buffer);
	MOVLW	HIGH(_debug_buffer)
	MOVWF	r0x05
	MOVLW	LOW(_debug_buffer)
	MOVWF	r0x04
	MOVLW	0x80
	MOVWF	r0x06
	MOVF	r0x06, W
	MOVWF	POSTDEC1
	MOVF	r0x05, W
	MOVWF	POSTDEC1
	MOVF	r0x04, W
	MOVWF	POSTDEC1
	CALL	_usart_puts
	MOVLW	0x03
	ADDWF	FSR1L, F
;	.line	193; meter_logger.c	fsk_tx_enable();
	CALL	_fsk_tx_enable
_00116_DS_:
;	.line	194; meter_logger.c	while (fifo_get(&cmd)) {	// and send them via fsk
	MOVLW	HIGH(_main_cmd_1_90)
	MOVWF	r0x05
	MOVLW	LOW(_main_cmd_1_90)
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
	BANKSEL	_main_cmd_1_90
;	.line	195; meter_logger.c	fsk_tx_byte(cmd);
	MOVF	_main_cmd_1_90, W, B
	MOVWF	POSTDEC1
	CALL	_fsk_tx_byte
	MOVF	POSTINC1, F
;	.line	196; meter_logger.c	sleep_ms(FSK_TX_SLEEP);
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x04
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVF	POSTINC1, F
	MOVF	POSTINC1, F
	BRA	_00116_DS_
_00118_DS_:
;	.line	198; meter_logger.c	fsk_tx_disable();
	CALL	_fsk_tx_disable
;	.line	202; meter_logger.c	fsk_rx_enable();
	CALL	_fsk_rx_enable
;	.line	203; meter_logger.c	break;
	GOTO	_00165_DS_
_00119_DS_:
;	.line	213; meter_logger.c	last_fifo_size = 0;
	CLRF	r0x00
	CLRF	r0x01
;	.line	214; meter_logger.c	sleep_ms(1000);							// 1 second
	MOVLW	0x03
	MOVWF	POSTDEC1
	MOVLW	0xe8
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVF	POSTINC1, F
	MOVF	POSTINC1, F
;	.line	215; meter_logger.c	fifo_size = fifo_in_use();
	CALL	_fifo_in_use
	MOVWF	r0x02
	MOVFF	PRODL, r0x03
_00120_DS_:
;	.line	216; meter_logger.c	while (fifo_size > last_fifo_size) {	// and wait while we are still receiving data
	MOVF	r0x03, W
	SUBWF	r0x01, W
	BNZ	_00342_DS_
	MOVF	r0x02, W
	SUBWF	r0x00, W
_00342_DS_:
	BC	_00122_DS_
;	.line	217; meter_logger.c	last_fifo_size = fifo_size;
	MOVFF	r0x02, r0x00
	MOVFF	r0x03, r0x01
;	.line	218; meter_logger.c	sleep_ms(500);						// return data when no data for 500 ms
	MOVLW	0x01
	MOVWF	POSTDEC1
	MOVLW	0xf4
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVF	POSTINC1, F
	MOVF	POSTINC1, F
;	.line	219; meter_logger.c	fifo_size = fifo_in_use();
	CALL	_fifo_in_use
	MOVWF	r0x02
	MOVFF	PRODL, r0x03
	BRA	_00120_DS_
_00122_DS_:
;	.line	221; meter_logger.c	fsk_rx_disable();
	CALL	_fsk_rx_disable
;	.line	224; meter_logger.c	sprintf(debug_buffer, "-> ");
	MOVLW	UPPER(__str_7)
	MOVWF	r0x06
	MOVLW	HIGH(__str_7)
	MOVWF	r0x05
	MOVLW	LOW(__str_7)
	MOVWF	r0x04
	MOVLW	HIGH(_debug_buffer)
	MOVWF	r0x08
	MOVLW	LOW(_debug_buffer)
	MOVWF	r0x07
	MOVLW	0x80
	MOVWF	r0x09
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
	MOVLW	0x06
	ADDWF	FSR1L, F
;	.line	225; meter_logger.c	usart_puts(debug_buffer);
	MOVLW	HIGH(_debug_buffer)
	MOVWF	r0x05
	MOVLW	LOW(_debug_buffer)
	MOVWF	r0x04
	MOVLW	0x80
	MOVWF	r0x06
	MOVF	r0x06, W
	MOVWF	POSTDEC1
	MOVF	r0x05, W
	MOVWF	POSTDEC1
	MOVF	r0x04, W
	MOVWF	POSTDEC1
	CALL	_usart_puts
	MOVLW	0x03
	ADDWF	FSR1L, F
;	.line	226; meter_logger.c	for (i = 0; i < fifo_in_use(); i++) {
	CLRF	r0x04
	CLRF	r0x05
_00177_DS_:
	CALL	_fifo_in_use
	MOVWF	r0x06
	MOVFF	PRODL, r0x07
	MOVF	r0x07, W
	SUBWF	r0x05, W
	BNZ	_00343_DS_
	MOVF	r0x06, W
	SUBWF	r0x04, W
_00343_DS_:
	BTFSC	STATUS, 0
	BRA	_00123_DS_
;	.line	228; meter_logger.c	fifo_get(&sub_cmd);
	MOVLW	HIGH(_main_sub_cmd_1_90)
	MOVWF	r0x07
	MOVLW	LOW(_main_sub_cmd_1_90)
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
;	.line	230; meter_logger.c	sprintf(debug_buffer, "%d ", sub_cmd);
	MOVFF	_main_sub_cmd_1_90, r0x06
	CLRF	r0x07
	MOVLW	UPPER(__str_5)
	MOVWF	r0x0a
	MOVLW	HIGH(__str_5)
	MOVWF	r0x09
	MOVLW	LOW(__str_5)
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
;	.line	231; meter_logger.c	usart_puts(debug_buffer);
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
	BANKSEL	_main_sub_cmd_1_90
;	.line	233; meter_logger.c	fifo_put(sub_cmd);
	MOVF	_main_sub_cmd_1_90, W, B
	MOVWF	POSTDEC1
	CALL	_fifo_put
	MOVF	POSTINC1, F
;	.line	226; meter_logger.c	for (i = 0; i < fifo_in_use(); i++) {
	INFSNZ	r0x04, F
	INCF	r0x05, F
	BRA	_00177_DS_
_00123_DS_:
;	.line	235; meter_logger.c	sprintf(debug_buffer, "\n\r");
	MOVLW	UPPER(__str_6)
	MOVWF	r0x06
	MOVLW	HIGH(__str_6)
	MOVWF	r0x05
	MOVLW	LOW(__str_6)
	MOVWF	r0x04
	MOVLW	HIGH(_debug_buffer)
	MOVWF	r0x08
	MOVLW	LOW(_debug_buffer)
	MOVWF	r0x07
	MOVLW	0x80
	MOVWF	r0x09
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
	MOVLW	0x06
	ADDWF	FSR1L, F
;	.line	236; meter_logger.c	usart_puts(debug_buffer);
	MOVLW	HIGH(_debug_buffer)
	MOVWF	r0x05
	MOVLW	LOW(_debug_buffer)
	MOVWF	r0x04
	MOVLW	0x80
	MOVWF	r0x06
	MOVF	r0x06, W
	MOVWF	POSTDEC1
	MOVF	r0x05, W
	MOVWF	POSTDEC1
	MOVF	r0x04, W
	MOVWF	POSTDEC1
	CALL	_usart_puts
	MOVLW	0x03
	ADDWF	FSR1L, F
;	.line	240; meter_logger.c	fsk_tx_enable();
	CALL	_fsk_tx_enable
_00124_DS_:
;	.line	241; meter_logger.c	while (fifo_get(&sub_cmd)) {
	MOVLW	HIGH(_main_sub_cmd_1_90)
	MOVWF	r0x05
	MOVLW	LOW(_main_sub_cmd_1_90)
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
	BZ	_00126_DS_
	BANKSEL	_main_sub_cmd_1_90
;	.line	242; meter_logger.c	fsk_tx_byte(sub_cmd);
	MOVF	_main_sub_cmd_1_90, W, B
	MOVWF	POSTDEC1
	CALL	_fsk_tx_byte
	MOVF	POSTINC1, F
;	.line	243; meter_logger.c	sleep_ms(FSK_TX_SLEEP);
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x04
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVF	POSTINC1, F
	MOVF	POSTINC1, F
	BRA	_00124_DS_
_00126_DS_:
;	.line	245; meter_logger.c	fsk_tx_disable();
	CALL	_fsk_tx_disable
;	.line	250; meter_logger.c	fsk_rx_enable();
	CALL	_fsk_rx_enable
;	.line	251; meter_logger.c	break;
	GOTO	_00165_DS_
_00127_DS_:
;	.line	254; meter_logger.c	fsk_rx_disable();
	CALL	_fsk_rx_disable
;	.line	260; meter_logger.c	fsk_rx_enable();
	CALL	_fsk_rx_enable
;	.line	261; meter_logger.c	last_fifo_size = 0;
	CLRF	r0x00
	CLRF	r0x01
;	.line	262; meter_logger.c	sleep_ms(400);							// sleep 400 ms to let some data come in
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
_00128_DS_:
;	.line	264; meter_logger.c	while (fifo_size > last_fifo_size) {	// and wait while we are still receiving data
	MOVF	r0x03, W
	SUBWF	r0x01, W
	BNZ	_00344_DS_
	MOVF	r0x02, W
	SUBWF	r0x00, W
_00344_DS_:
	BC	_00130_DS_
;	.line	265; meter_logger.c	last_fifo_size = fifo_size;
	MOVFF	r0x02, r0x00
	MOVFF	r0x03, r0x01
;	.line	266; meter_logger.c	sleep_ms(200);						// return data when no data for 100 ms
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0xc8
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVF	POSTINC1, F
	MOVF	POSTINC1, F
;	.line	267; meter_logger.c	fifo_size = fifo_in_use();
	CALL	_fifo_in_use
	MOVWF	r0x02
	MOVFF	PRODL, r0x03
	BRA	_00128_DS_
_00130_DS_:
;	.line	269; meter_logger.c	fsk_rx_disable();
	CALL	_fsk_rx_disable
;	.line	275; meter_logger.c	sprintf(debug_buffer, "-> ");
	MOVLW	UPPER(__str_7)
	MOVWF	r0x06
	MOVLW	HIGH(__str_7)
	MOVWF	r0x05
	MOVLW	LOW(__str_7)
	MOVWF	r0x04
	MOVLW	HIGH(_debug_buffer)
	MOVWF	r0x08
	MOVLW	LOW(_debug_buffer)
	MOVWF	r0x07
	MOVLW	0x80
	MOVWF	r0x09
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
	MOVLW	0x06
	ADDWF	FSR1L, F
;	.line	276; meter_logger.c	usart_puts(debug_buffer);
	MOVLW	HIGH(_debug_buffer)
	MOVWF	r0x05
	MOVLW	LOW(_debug_buffer)
	MOVWF	r0x04
	MOVLW	0x80
	MOVWF	r0x06
	MOVF	r0x06, W
	MOVWF	POSTDEC1
	MOVF	r0x05, W
	MOVWF	POSTDEC1
	MOVF	r0x04, W
	MOVWF	POSTDEC1
	CALL	_usart_puts
	MOVLW	0x03
	ADDWF	FSR1L, F
;	.line	277; meter_logger.c	for (i = 0; i < fifo_in_use(); i++) {
	CLRF	r0x04
	CLRF	r0x05
_00180_DS_:
	CALL	_fifo_in_use
	MOVWF	r0x06
	MOVFF	PRODL, r0x07
	MOVF	r0x07, W
	SUBWF	r0x05, W
	BNZ	_00345_DS_
	MOVF	r0x06, W
	SUBWF	r0x04, W
_00345_DS_:
	BTFSC	STATUS, 0
	BRA	_00131_DS_
;	.line	279; meter_logger.c	fifo_get(&sub_cmd);
	MOVLW	HIGH(_main_sub_cmd_1_90)
	MOVWF	r0x07
	MOVLW	LOW(_main_sub_cmd_1_90)
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
;	.line	281; meter_logger.c	sprintf(debug_buffer, "%d ", sub_cmd);
	MOVFF	_main_sub_cmd_1_90, r0x06
	CLRF	r0x07
	MOVLW	UPPER(__str_5)
	MOVWF	r0x0a
	MOVLW	HIGH(__str_5)
	MOVWF	r0x09
	MOVLW	LOW(__str_5)
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
;	.line	282; meter_logger.c	usart_puts(debug_buffer);
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
	BANKSEL	_main_sub_cmd_1_90
;	.line	284; meter_logger.c	fifo_put(sub_cmd);
	MOVF	_main_sub_cmd_1_90, W, B
	MOVWF	POSTDEC1
	CALL	_fifo_put
	MOVF	POSTINC1, F
;	.line	277; meter_logger.c	for (i = 0; i < fifo_in_use(); i++) {
	INFSNZ	r0x04, F
	INCF	r0x05, F
	BRA	_00180_DS_
_00131_DS_:
;	.line	286; meter_logger.c	sprintf(debug_buffer, "\n\r");
	MOVLW	UPPER(__str_6)
	MOVWF	r0x06
	MOVLW	HIGH(__str_6)
	MOVWF	r0x05
	MOVLW	LOW(__str_6)
	MOVWF	r0x04
	MOVLW	HIGH(_debug_buffer)
	MOVWF	r0x08
	MOVLW	LOW(_debug_buffer)
	MOVWF	r0x07
	MOVLW	0x80
	MOVWF	r0x09
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
	MOVLW	0x06
	ADDWF	FSR1L, F
;	.line	287; meter_logger.c	usart_puts(debug_buffer);
	MOVLW	HIGH(_debug_buffer)
	MOVWF	r0x05
	MOVLW	LOW(_debug_buffer)
	MOVWF	r0x04
	MOVLW	0x80
	MOVWF	r0x06
	MOVF	r0x06, W
	MOVWF	POSTDEC1
	MOVF	r0x05, W
	MOVWF	POSTDEC1
	MOVF	r0x04, W
	MOVWF	POSTDEC1
	CALL	_usart_puts
	MOVLW	0x03
	ADDWF	FSR1L, F
;	.line	290; meter_logger.c	rs232_8n2_tx_enable(TIMER0_RS232_1200);
	MOVLW	0xf9
	MOVWF	POSTDEC1
	MOVLW	0xae
	MOVWF	POSTDEC1
	CALL	_rs232_8n2_tx_enable
	MOVF	POSTINC1, F
	MOVF	POSTINC1, F
_00132_DS_:
;	.line	291; meter_logger.c	while (fifo_get(&sub_cmd)) {
	MOVLW	HIGH(_main_sub_cmd_1_90)
	MOVWF	r0x05
	MOVLW	LOW(_main_sub_cmd_1_90)
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
	BZ	_00134_DS_
	BANKSEL	_main_sub_cmd_1_90
;	.line	292; meter_logger.c	rs232_8n2_tx_byte(sub_cmd);
	MOVF	_main_sub_cmd_1_90, W, B
	MOVWF	POSTDEC1
	CALL	_rs232_8n2_tx_byte
	MOVF	POSTINC1, F
;	.line	293; meter_logger.c	sleep_ms(RS232_TX_SLEEP);
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x0c
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVF	POSTINC1, F
	MOVF	POSTINC1, F
	BRA	_00132_DS_
_00134_DS_:
;	.line	295; meter_logger.c	rs232_8n2_tx_disable();
	CALL	_rs232_8n2_tx_disable
;	.line	301; meter_logger.c	rs232_8n2_rx_enable(TIMER0_RS232_1200);
	MOVLW	0xf9
	MOVWF	POSTDEC1
	MOVLW	0xae
	MOVWF	POSTDEC1
	CALL	_rs232_8n2_rx_enable
	MOVF	POSTINC1, F
	MOVF	POSTINC1, F
;	.line	302; meter_logger.c	last_fifo_size = 0;
	CLRF	r0x00
	CLRF	r0x01
;	.line	303; meter_logger.c	sleep_ms(400);							// sleep 400 ms to let some data come in
	MOVLW	0x01
	MOVWF	POSTDEC1
	MOVLW	0x90
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVF	POSTINC1, F
	MOVF	POSTINC1, F
;	.line	304; meter_logger.c	fifo_size = fifo_in_use();
	CALL	_fifo_in_use
	MOVWF	r0x02
	MOVFF	PRODL, r0x03
_00135_DS_:
;	.line	306; meter_logger.c	while (fifo_size > last_fifo_size) {	// and wait while we are still receiving data
	MOVF	r0x03, W
	SUBWF	r0x01, W
	BNZ	_00346_DS_
	MOVF	r0x02, W
	SUBWF	r0x00, W
_00346_DS_:
	BC	_00137_DS_
;	.line	307; meter_logger.c	last_fifo_size = fifo_size;
	MOVFF	r0x02, r0x00
	MOVFF	r0x03, r0x01
;	.line	308; meter_logger.c	sleep_ms(200);						// return data when no data for 200 ms
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0xc8
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVF	POSTINC1, F
	MOVF	POSTINC1, F
;	.line	309; meter_logger.c	fifo_size = fifo_in_use();
	CALL	_fifo_in_use
	MOVWF	r0x02
	MOVFF	PRODL, r0x03
	BRA	_00135_DS_
_00137_DS_:
;	.line	312; meter_logger.c	rs232_8n2_rx_disable();
	CALL	_rs232_8n2_rx_disable
;	.line	319; meter_logger.c	sprintf(debug_buffer, "<- ");
	MOVLW	UPPER(__str_4)
	MOVWF	r0x06
	MOVLW	HIGH(__str_4)
	MOVWF	r0x05
	MOVLW	LOW(__str_4)
	MOVWF	r0x04
	MOVLW	HIGH(_debug_buffer)
	MOVWF	r0x08
	MOVLW	LOW(_debug_buffer)
	MOVWF	r0x07
	MOVLW	0x80
	MOVWF	r0x09
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
	MOVLW	0x06
	ADDWF	FSR1L, F
;	.line	320; meter_logger.c	usart_puts(debug_buffer);
	MOVLW	HIGH(_debug_buffer)
	MOVWF	r0x05
	MOVLW	LOW(_debug_buffer)
	MOVWF	r0x04
	MOVLW	0x80
	MOVWF	r0x06
	MOVF	r0x06, W
	MOVWF	POSTDEC1
	MOVF	r0x05, W
	MOVWF	POSTDEC1
	MOVF	r0x04, W
	MOVWF	POSTDEC1
	CALL	_usart_puts
	MOVLW	0x03
	ADDWF	FSR1L, F
;	.line	321; meter_logger.c	for (i = 0; i < fifo_in_use(); i++) {
	CLRF	r0x04
	CLRF	r0x05
_00183_DS_:
	CALL	_fifo_in_use
	MOVWF	r0x06
	MOVFF	PRODL, r0x07
	MOVF	r0x07, W
	SUBWF	r0x05, W
	BNZ	_00347_DS_
	MOVF	r0x06, W
	SUBWF	r0x04, W
_00347_DS_:
	BTFSC	STATUS, 0
	BRA	_00138_DS_
;	.line	323; meter_logger.c	fifo_get(&sub_cmd);
	MOVLW	HIGH(_main_sub_cmd_1_90)
	MOVWF	r0x07
	MOVLW	LOW(_main_sub_cmd_1_90)
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
;	.line	325; meter_logger.c	sprintf(debug_buffer, "%d ", sub_cmd);
	MOVFF	_main_sub_cmd_1_90, r0x06
	CLRF	r0x07
	MOVLW	UPPER(__str_5)
	MOVWF	r0x0a
	MOVLW	HIGH(__str_5)
	MOVWF	r0x09
	MOVLW	LOW(__str_5)
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
;	.line	326; meter_logger.c	usart_puts(debug_buffer);
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
	BANKSEL	_main_sub_cmd_1_90
;	.line	328; meter_logger.c	fifo_put(sub_cmd);
	MOVF	_main_sub_cmd_1_90, W, B
	MOVWF	POSTDEC1
	CALL	_fifo_put
	MOVF	POSTINC1, F
;	.line	321; meter_logger.c	for (i = 0; i < fifo_in_use(); i++) {
	INFSNZ	r0x04, F
	INCF	r0x05, F
	BRA	_00183_DS_
_00138_DS_:
;	.line	330; meter_logger.c	sprintf(debug_buffer, "\n\r");
	MOVLW	UPPER(__str_6)
	MOVWF	r0x06
	MOVLW	HIGH(__str_6)
	MOVWF	r0x05
	MOVLW	LOW(__str_6)
	MOVWF	r0x04
	MOVLW	HIGH(_debug_buffer)
	MOVWF	r0x08
	MOVLW	LOW(_debug_buffer)
	MOVWF	r0x07
	MOVLW	0x80
	MOVWF	r0x09
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
	MOVLW	0x06
	ADDWF	FSR1L, F
;	.line	331; meter_logger.c	usart_puts(debug_buffer);
	MOVLW	HIGH(_debug_buffer)
	MOVWF	r0x05
	MOVLW	LOW(_debug_buffer)
	MOVWF	r0x04
	MOVLW	0x80
	MOVWF	r0x06
	MOVF	r0x06, W
	MOVWF	POSTDEC1
	MOVF	r0x05, W
	MOVWF	POSTDEC1
	MOVF	r0x04, W
	MOVWF	POSTDEC1
	CALL	_usart_puts
	MOVLW	0x03
	ADDWF	FSR1L, F
;	.line	333; meter_logger.c	if (fifo_in_use()) {
	CALL	_fifo_in_use
	MOVWF	r0x04
	MOVFF	PRODL, r0x05
	MOVF	r0x04, W
	IORWF	r0x05, W
	BZ	_00143_DS_
;	.line	335; meter_logger.c	fsk_tx_enable();
	CALL	_fsk_tx_enable
_00139_DS_:
;	.line	336; meter_logger.c	while (fifo_get(&sub_cmd)) {
	MOVLW	HIGH(_main_sub_cmd_1_90)
	MOVWF	r0x05
	MOVLW	LOW(_main_sub_cmd_1_90)
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
	BZ	_00141_DS_
	BANKSEL	_main_sub_cmd_1_90
;	.line	337; meter_logger.c	fsk_tx_byte(sub_cmd);
	MOVF	_main_sub_cmd_1_90, W, B
	MOVWF	POSTDEC1
	CALL	_fsk_tx_byte
	MOVF	POSTINC1, F
;	.line	338; meter_logger.c	sleep_ms(FSK_TX_SLEEP);
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x04
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVF	POSTINC1, F
	MOVF	POSTINC1, F
	BRA	_00139_DS_
_00141_DS_:
;	.line	340; meter_logger.c	fsk_tx_disable();
	CALL	_fsk_tx_disable
	BRA	_00144_DS_
_00143_DS_:
;	.line	348; meter_logger.c	fsk_tx_enable();
	CALL	_fsk_tx_enable
;	.line	349; meter_logger.c	fsk_tx_byte(0x0d);
	MOVLW	0x0d
	MOVWF	POSTDEC1
	CALL	_fsk_tx_byte
	MOVF	POSTINC1, F
;	.line	350; meter_logger.c	sleep_ms(FSK_TX_SLEEP);
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x04
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVF	POSTINC1, F
	MOVF	POSTINC1, F
;	.line	351; meter_logger.c	fsk_tx_disable();
	CALL	_fsk_tx_disable
_00144_DS_:
;	.line	356; meter_logger.c	fsk_rx_enable();
	CALL	_fsk_rx_enable
;	.line	357; meter_logger.c	break;
	GOTO	_00165_DS_
_00145_DS_:
;	.line	359; meter_logger.c	fsk_rx_disable();
	CALL	_fsk_rx_disable
;	.line	365; meter_logger.c	fsk_rx_enable();
	CALL	_fsk_rx_enable
;	.line	366; meter_logger.c	last_fifo_size = 0;
	CLRF	r0x00
	CLRF	r0x01
;	.line	367; meter_logger.c	sleep_ms(400);							// sleep 400 ms to let some data come in
	MOVLW	0x01
	MOVWF	POSTDEC1
	MOVLW	0x90
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVF	POSTINC1, F
	MOVF	POSTINC1, F
;	.line	368; meter_logger.c	fifo_size = fifo_in_use();
	CALL	_fifo_in_use
	MOVWF	r0x02
	MOVFF	PRODL, r0x03
_00146_DS_:
;	.line	369; meter_logger.c	while (fifo_size > last_fifo_size) {	// and wait while we are still receiving data
	MOVF	r0x03, W
	SUBWF	r0x01, W
	BNZ	_00348_DS_
	MOVF	r0x02, W
	SUBWF	r0x00, W
_00348_DS_:
	BC	_00148_DS_
;	.line	370; meter_logger.c	last_fifo_size = fifo_size;
	MOVFF	r0x02, r0x00
	MOVFF	r0x03, r0x01
;	.line	371; meter_logger.c	sleep_ms(200);						// return data when no data for 100 ms
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0xc8
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVF	POSTINC1, F
	MOVF	POSTINC1, F
;	.line	372; meter_logger.c	fifo_size = fifo_in_use();
	CALL	_fifo_in_use
	MOVWF	r0x02
	MOVFF	PRODL, r0x03
	BRA	_00146_DS_
_00148_DS_:
;	.line	374; meter_logger.c	fsk_rx_disable();
	CALL	_fsk_rx_disable
;	.line	380; meter_logger.c	sprintf(debug_buffer, "-> ");
	MOVLW	UPPER(__str_7)
	MOVWF	r0x06
	MOVLW	HIGH(__str_7)
	MOVWF	r0x05
	MOVLW	LOW(__str_7)
	MOVWF	r0x04
	MOVLW	HIGH(_debug_buffer)
	MOVWF	r0x08
	MOVLW	LOW(_debug_buffer)
	MOVWF	r0x07
	MOVLW	0x80
	MOVWF	r0x09
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
	MOVLW	0x06
	ADDWF	FSR1L, F
;	.line	381; meter_logger.c	usart_puts(debug_buffer);
	MOVLW	HIGH(_debug_buffer)
	MOVWF	r0x05
	MOVLW	LOW(_debug_buffer)
	MOVWF	r0x04
	MOVLW	0x80
	MOVWF	r0x06
	MOVF	r0x06, W
	MOVWF	POSTDEC1
	MOVF	r0x05, W
	MOVWF	POSTDEC1
	MOVF	r0x04, W
	MOVWF	POSTDEC1
	CALL	_usart_puts
	MOVLW	0x03
	ADDWF	FSR1L, F
;	.line	382; meter_logger.c	for (i = 0; i < fifo_in_use(); i++) {
	CLRF	r0x04
	CLRF	r0x05
_00186_DS_:
	CALL	_fifo_in_use
	MOVWF	r0x06
	MOVFF	PRODL, r0x07
	MOVF	r0x07, W
	SUBWF	r0x05, W
	BNZ	_00349_DS_
	MOVF	r0x06, W
	SUBWF	r0x04, W
_00349_DS_:
	BTFSC	STATUS, 0
	BRA	_00149_DS_
;	.line	384; meter_logger.c	fifo_get(&sub_cmd);
	MOVLW	HIGH(_main_sub_cmd_1_90)
	MOVWF	r0x07
	MOVLW	LOW(_main_sub_cmd_1_90)
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
;	.line	386; meter_logger.c	sprintf(debug_buffer, "%d ", sub_cmd);
	MOVFF	_main_sub_cmd_1_90, r0x06
	CLRF	r0x07
	MOVLW	UPPER(__str_5)
	MOVWF	r0x0a
	MOVLW	HIGH(__str_5)
	MOVWF	r0x09
	MOVLW	LOW(__str_5)
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
;	.line	387; meter_logger.c	usart_puts(debug_buffer);
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
	BANKSEL	_main_sub_cmd_1_90
;	.line	389; meter_logger.c	fifo_put(sub_cmd);
	MOVF	_main_sub_cmd_1_90, W, B
	MOVWF	POSTDEC1
	CALL	_fifo_put
	MOVF	POSTINC1, F
;	.line	382; meter_logger.c	for (i = 0; i < fifo_in_use(); i++) {
	INFSNZ	r0x04, F
	INCF	r0x05, F
	BRA	_00186_DS_
_00149_DS_:
;	.line	391; meter_logger.c	sprintf(debug_buffer, "\n\r");
	MOVLW	UPPER(__str_6)
	MOVWF	r0x06
	MOVLW	HIGH(__str_6)
	MOVWF	r0x05
	MOVLW	LOW(__str_6)
	MOVWF	r0x04
	MOVLW	HIGH(_debug_buffer)
	MOVWF	r0x08
	MOVLW	LOW(_debug_buffer)
	MOVWF	r0x07
	MOVLW	0x80
	MOVWF	r0x09
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
	MOVLW	0x06
	ADDWF	FSR1L, F
;	.line	392; meter_logger.c	usart_puts(debug_buffer);
	MOVLW	HIGH(_debug_buffer)
	MOVWF	r0x05
	MOVLW	LOW(_debug_buffer)
	MOVWF	r0x04
	MOVLW	0x80
	MOVWF	r0x06
	MOVF	r0x06, W
	MOVWF	POSTDEC1
	MOVF	r0x05, W
	MOVWF	POSTDEC1
	MOVF	r0x04, W
	MOVWF	POSTDEC1
	CALL	_usart_puts
	MOVLW	0x03
	ADDWF	FSR1L, F
;	.line	394; meter_logger.c	rs232_7e1_tx_enable(TIMER0_RS232_300);
	MOVLW	0xe6
	MOVWF	POSTDEC1
	MOVLW	0x1b
	MOVWF	POSTDEC1
	CALL	_rs232_7e1_tx_enable
	MOVF	POSTINC1, F
	MOVF	POSTINC1, F
_00150_DS_:
;	.line	395; meter_logger.c	while (fifo_get(&sub_cmd)) {
	MOVLW	HIGH(_main_sub_cmd_1_90)
	MOVWF	r0x05
	MOVLW	LOW(_main_sub_cmd_1_90)
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
	BZ	_00152_DS_
	BANKSEL	_main_sub_cmd_1_90
;	.line	396; meter_logger.c	rs232_7e1_tx_byte(sub_cmd);
	MOVF	_main_sub_cmd_1_90, W, B
	MOVWF	POSTDEC1
	CALL	_rs232_7e1_tx_byte
	MOVF	POSTINC1, F
;	.line	397; meter_logger.c	sleep_ms(RS232_TX_SLEEP);
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x0c
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVF	POSTINC1, F
	MOVF	POSTINC1, F
	BRA	_00150_DS_
_00152_DS_:
;	.line	399; meter_logger.c	rs232_7e1_tx_disable();
	CALL	_rs232_7e1_tx_disable
;	.line	405; meter_logger.c	rs232_7e1_rx_enable(TIMER0_RS232_300);
	MOVLW	0xe6
	MOVWF	POSTDEC1
	MOVLW	0x1b
	MOVWF	POSTDEC1
	CALL	_rs232_7e1_rx_enable
	MOVF	POSTINC1, F
	MOVF	POSTINC1, F
;	.line	406; meter_logger.c	last_fifo_size = 0;
	CLRF	r0x00
	CLRF	r0x01
;	.line	407; meter_logger.c	sleep_ms(1500);							// sleep 1500 ms to let some data come in
	MOVLW	0x05
	MOVWF	POSTDEC1
	MOVLW	0xdc
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVF	POSTINC1, F
	MOVF	POSTINC1, F
;	.line	408; meter_logger.c	fifo_size = fifo_in_use();
	CALL	_fifo_in_use
	MOVWF	r0x02
	MOVFF	PRODL, r0x03
_00153_DS_:
;	.line	410; meter_logger.c	while (fifo_size > last_fifo_size) {	// and wait while we are still receiving data
	MOVF	r0x03, W
	SUBWF	r0x01, W
	BNZ	_00350_DS_
	MOVF	r0x02, W
	SUBWF	r0x00, W
_00350_DS_:
	BC	_00155_DS_
;	.line	411; meter_logger.c	last_fifo_size = fifo_size;
	MOVFF	r0x02, r0x00
	MOVFF	r0x03, r0x01
;	.line	412; meter_logger.c	sleep_ms(600);						// return data when no data for 600 ms
	MOVLW	0x02
	MOVWF	POSTDEC1
	MOVLW	0x58
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVF	POSTINC1, F
	MOVF	POSTINC1, F
;	.line	413; meter_logger.c	fifo_size = fifo_in_use();
	CALL	_fifo_in_use
	MOVWF	r0x02
	MOVFF	PRODL, r0x03
	BRA	_00153_DS_
_00155_DS_:
;	.line	416; meter_logger.c	rs232_7e1_rx_disable();
	CALL	_rs232_7e1_rx_disable
;	.line	423; meter_logger.c	sprintf(debug_buffer, "<- ");
	MOVLW	UPPER(__str_4)
	MOVWF	r0x02
	MOVLW	HIGH(__str_4)
	MOVWF	r0x01
	MOVLW	LOW(__str_4)
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
;	.line	424; meter_logger.c	usart_puts(debug_buffer);
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
;	.line	425; meter_logger.c	for (i = 0; i < fifo_in_use(); i++) {
	CLRF	r0x00
	CLRF	r0x01
_00189_DS_:
	CALL	_fifo_in_use
	MOVWF	r0x02
	MOVFF	PRODL, r0x03
	MOVF	r0x03, W
	SUBWF	r0x01, W
	BNZ	_00351_DS_
	MOVF	r0x02, W
	SUBWF	r0x00, W
_00351_DS_:
	BTFSC	STATUS, 0
	BRA	_00156_DS_
;	.line	427; meter_logger.c	fifo_get(&sub_cmd);
	MOVLW	HIGH(_main_sub_cmd_1_90)
	MOVWF	r0x03
	MOVLW	LOW(_main_sub_cmd_1_90)
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
;	.line	429; meter_logger.c	sprintf(debug_buffer, "%d ", sub_cmd);
	MOVFF	_main_sub_cmd_1_90, r0x02
	CLRF	r0x03
	MOVLW	UPPER(__str_5)
	MOVWF	r0x06
	MOVLW	HIGH(__str_5)
	MOVWF	r0x05
	MOVLW	LOW(__str_5)
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
;	.line	430; meter_logger.c	usart_puts(debug_buffer);
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
	BANKSEL	_main_sub_cmd_1_90
;	.line	432; meter_logger.c	fifo_put(sub_cmd);
	MOVF	_main_sub_cmd_1_90, W, B
	MOVWF	POSTDEC1
	CALL	_fifo_put
	MOVF	POSTINC1, F
;	.line	425; meter_logger.c	for (i = 0; i < fifo_in_use(); i++) {
	INFSNZ	r0x00, F
	INCF	r0x01, F
	BRA	_00189_DS_
_00156_DS_:
;	.line	434; meter_logger.c	sprintf(debug_buffer, "\n\r");
	MOVLW	UPPER(__str_6)
	MOVWF	r0x02
	MOVLW	HIGH(__str_6)
	MOVWF	r0x01
	MOVLW	LOW(__str_6)
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
;	.line	435; meter_logger.c	usart_puts(debug_buffer);
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
;	.line	437; meter_logger.c	if (fifo_in_use()) {
	CALL	_fifo_in_use
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVF	r0x00, W
	IORWF	r0x01, W
	BZ	_00161_DS_
;	.line	439; meter_logger.c	fsk_tx_enable();
	CALL	_fsk_tx_enable
_00157_DS_:
;	.line	440; meter_logger.c	while (fifo_get(&sub_cmd)) {
	MOVLW	HIGH(_main_sub_cmd_1_90)
	MOVWF	r0x01
	MOVLW	LOW(_main_sub_cmd_1_90)
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
	BZ	_00159_DS_
	BANKSEL	_main_sub_cmd_1_90
;	.line	441; meter_logger.c	fsk_tx_byte(sub_cmd);
	MOVF	_main_sub_cmd_1_90, W, B
	MOVWF	POSTDEC1
	CALL	_fsk_tx_byte
	MOVF	POSTINC1, F
;	.line	442; meter_logger.c	sleep_ms(FSK_TX_SLEEP);
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x04
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVF	POSTINC1, F
	MOVF	POSTINC1, F
	BRA	_00157_DS_
_00159_DS_:
;	.line	444; meter_logger.c	fsk_tx_disable();
	CALL	_fsk_tx_disable
	BRA	_00162_DS_
_00161_DS_:
;	.line	452; meter_logger.c	fsk_tx_enable();
	CALL	_fsk_tx_enable
;	.line	453; meter_logger.c	fsk_tx_byte(0x0d);
	MOVLW	0x0d
	MOVWF	POSTDEC1
	CALL	_fsk_tx_byte
	MOVF	POSTINC1, F
;	.line	454; meter_logger.c	sleep_ms(FSK_TX_SLEEP);
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x04
	MOVWF	POSTDEC1
	CALL	_sleep_ms
	MOVF	POSTINC1, F
	MOVF	POSTINC1, F
;	.line	455; meter_logger.c	fsk_tx_disable();
	CALL	_fsk_tx_disable
_00162_DS_:
;	.line	460; meter_logger.c	fsk_rx_enable();
	CALL	_fsk_rx_enable
;	.line	461; meter_logger.c	break;
	BRA	_00165_DS_
_00163_DS_:
;	.line	463; meter_logger.c	fsk_rx_disable();
	CALL	_fsk_rx_disable
;	.line	464; meter_logger.c	get_battery_level();
	CALL	_get_battery_level
;	.line	465; meter_logger.c	fsk_rx_enable();
	CALL	_fsk_rx_enable
_00165_DS_:
;	.line	470; meter_logger.c	while (fifo_in_use()) {
	CALL	_fifo_in_use
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVF	r0x00, W
	IORWF	r0x01, W
	BTFSC	STATUS, 2
	GOTO	_00171_DS_
;	.line	471; meter_logger.c	fifo_get(&sub_cmd);
	MOVLW	HIGH(_main_sub_cmd_1_90)
	MOVWF	r0x01
	MOVLW	LOW(_main_sub_cmd_1_90)
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
	MOVLW	0x03
	ADDWF	FSR1L, F
	BRA	_00165_DS_
	RETURN	

; ; Starting pCode block
S_meter_logger___debug2	code
__debug2:
;	.line	4658; meter_logger.c	void _debug2() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
;	.line	4659; meter_logger.c	DEBUG2_PIN = 0x1;
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
	
;	.line	4707; meter_logger.c	DEBUG2_PIN = 0x0;
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
;	.line	4558; meter_logger.c	void _debug() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
;	.line	4559; meter_logger.c	DEBUG_PIN = 0x1;
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
	
;	.line	4607; meter_logger.c	DEBUG_PIN = 0x0;
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
;	.line	4553; meter_logger.c	void flash_led(unsigned char ms) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
;	.line	4554; meter_logger.c	led_flash.timer = ms;
	MOVF	r0x00, W
	BANKSEL	(_led_flash + 1)
	MOVWF	(_led_flash + 1), B
; removed redundant BANKSEL
;	.line	4555; meter_logger.c	led_flash.state = LED_FLASH_RUN;
	CLRF	_led_flash, B
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__get_battery_level	code
_get_battery_level:
;	.line	4522; meter_logger.c	unsigned int get_battery_level() {
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
;	.line	4526; meter_logger.c	adc_open(ADC_CHN_4 , ADC_FOSC_64, ADC_CFG_5A, ADC_FRM_RJUST | ADC_INT_OFF | ADC_VCFG_VDD_VSS);
	MOVLW	0x80
	MOVWF	POSTDEC1
	MOVLW	0x0a
	MOVWF	POSTDEC1
	MOVLW	0x06
	MOVWF	POSTDEC1
	MOVLW	0x04
	MOVWF	POSTDEC1
	CALL	_adc_open
	MOVLW	0x04
	ADDWF	FSR1L, F
;	.line	4528; meter_logger.c	adc_setchannel(ADC_CHN_4);
	MOVLW	0x04
	MOVWF	POSTDEC1
	CALL	_adc_setchannel
	MOVF	POSTINC1, F
;	.line	4529; meter_logger.c	adc_conv();
	CALL	_adc_conv
_01195_DS_:
;	.line	4530; meter_logger.c	while(adc_busy()) {
	CALL	_adc_busy
	MOVWF	r0x00
	MOVF	r0x00, W
	BNZ	_01195_DS_
;	.line	4534; meter_logger.c	dev_id = get_dev_id();
	CALL	_get_dev_id
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
;	.line	4535; meter_logger.c	if (dev_id == 0x1240) {
	MOVF	r0x00, W
	XORLW	0x40
	BNZ	_01218_DS_
	MOVF	r0x01, W
	XORLW	0x12
	BZ	_01219_DS_
_01218_DS_:
	BRA	_01202_DS_
_01219_DS_:
;	.line	4537; meter_logger.c	v_level = (unsigned long)1000 * (unsigned long)adc_read() * (unsigned long)833/(unsigned long)93600;
	CALL	_adc_read
	MOVWF	r0x02
	MOVFF	PRODL, r0x03
	CLRF	WREG
	BTFSC	r0x03, 7
	MOVLW	0xff
	MOVWF	r0x04
	MOVWF	r0x05
	MOVF	r0x05, W
	MOVWF	POSTDEC1
	MOVF	r0x04, W
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x0c
	MOVWF	POSTDEC1
	MOVLW	0xb5
	MOVWF	POSTDEC1
	MOVLW	0xe8
	MOVWF	POSTDEC1
	CALL	__mullong
	MOVWF	r0x02
	MOVFF	PRODL, r0x03
	MOVFF	PRODH, r0x04
	MOVFF	FSR0L, r0x05
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x01
	MOVWF	POSTDEC1
	MOVLW	0x6d
	MOVWF	POSTDEC1
	MOVLW	0xa0
	MOVWF	POSTDEC1
	MOVF	r0x05, W
	MOVWF	POSTDEC1
	MOVF	r0x04, W
	MOVWF	POSTDEC1
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	CALL	__divulong
	MOVWF	r0x02
	MOVFF	PRODL, r0x03
	MOVFF	PRODH, r0x04
	MOVFF	FSR0L, r0x05
	MOVLW	0x08
	ADDWF	FSR1L, F
	BRA	_01203_DS_
_01202_DS_:
;	.line	4539; meter_logger.c	else if (dev_id == 0x2a40) {
	MOVF	r0x00, W
	XORLW	0x40
	BNZ	_01220_DS_
	MOVF	r0x01, W
	XORLW	0x2a
	BZ	_01221_DS_
_01220_DS_:
	BRA	_01199_DS_
_01221_DS_:
;	.line	4541; meter_logger.c	v_level = (unsigned long)1000 * (unsigned long)(adc_read() >> 2) * (unsigned long)833/(unsigned long)93600;
	CALL	_adc_read
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	RLCF	r0x01, W
	RRCF	r0x01, F
	RRCF	r0x00, F
	RLCF	r0x01, W
	RRCF	r0x01, F
	RRCF	r0x00, F
	CLRF	WREG
	BTFSC	r0x01, 7
	MOVLW	0xff
	MOVWF	r0x04
	MOVWF	r0x05
	MOVF	r0x05, W
	MOVWF	POSTDEC1
	MOVF	r0x04, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x0c
	MOVWF	POSTDEC1
	MOVLW	0xb5
	MOVWF	POSTDEC1
	MOVLW	0xe8
	MOVWF	POSTDEC1
	CALL	__mullong
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x04
	MOVFF	FSR0L, r0x05
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVLW	0x00
	MOVWF	POSTDEC1
	MOVLW	0x01
	MOVWF	POSTDEC1
	MOVLW	0x6d
	MOVWF	POSTDEC1
	MOVLW	0xa0
	MOVWF	POSTDEC1
	MOVF	r0x05, W
	MOVWF	POSTDEC1
	MOVF	r0x04, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	__divulong
	MOVWF	r0x00
	MOVFF	PRODL, r0x01
	MOVFF	PRODH, r0x04
	MOVFF	FSR0L, r0x05
	MOVLW	0x08
	ADDWF	FSR1L, F
	MOVF	r0x00, W
	MOVWF	r0x02
	MOVF	r0x01, W
	MOVWF	r0x03
	BRA	_01203_DS_
_01199_DS_:
;	.line	4544; meter_logger.c	v_level = 0;
	CLRF	r0x02
	CLRF	r0x03
_01203_DS_:
;	.line	4546; meter_logger.c	sprintf(debug_buffer, "Battery: %dmV\n\r", v_level);
	MOVLW	UPPER(__str_8)
	MOVWF	r0x04
	MOVLW	HIGH(__str_8)
	MOVWF	r0x01
	MOVLW	LOW(__str_8)
	MOVWF	r0x00
	MOVLW	HIGH(_debug_buffer)
	MOVWF	r0x06
	MOVLW	LOW(_debug_buffer)
	MOVWF	r0x05
	MOVLW	0x80
	MOVWF	r0x07
	MOVF	r0x03, W
	MOVWF	POSTDEC1
	MOVF	r0x02, W
	MOVWF	POSTDEC1
	MOVF	r0x04, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	MOVF	r0x07, W
	MOVWF	POSTDEC1
	MOVF	r0x06, W
	MOVWF	POSTDEC1
	MOVF	r0x05, W
	MOVWF	POSTDEC1
	CALL	_sprintf
	MOVLW	0x08
	ADDWF	FSR1L, F
;	.line	4547; meter_logger.c	usart_puts(debug_buffer);	
	MOVLW	HIGH(_debug_buffer)
	MOVWF	r0x01
	MOVLW	LOW(_debug_buffer)
	MOVWF	r0x00
	MOVLW	0x80
	MOVWF	r0x04
	MOVF	r0x04, W
	MOVWF	POSTDEC1
	MOVF	r0x01, W
	MOVWF	POSTDEC1
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	_usart_puts
	MOVLW	0x03
	ADDWF	FSR1L, F
;	.line	4549; meter_logger.c	adc_close();
	CALL	_adc_close
;	.line	4550; meter_logger.c	return v_level;
	MOVFF	r0x03, PRODL
	MOVF	r0x02, W
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
S_meter_logger__fifo_snoop	code
_fifo_snoop:
;	.line	4499; meter_logger.c	unsigned char fifo_snoop(unsigned char *c, unsigned int pos) {
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
;	.line	4500; meter_logger.c	if (fifo_in_use() > (pos)) {
	CALL	_fifo_in_use
	MOVWF	r0x05
	MOVFF	PRODL, r0x06
	MOVF	r0x06, W
	SUBWF	r0x04, W
	BNZ	_01188_DS_
	MOVF	r0x05, W
	SUBWF	r0x03, W
_01188_DS_:
	BTFSC	STATUS, 0
	BRA	_01178_DS_
	BANKSEL	(_fifo_tail + 1)
;	.line	4501; meter_logger.c	switch (fifo_tail/QUEUE_SIZE) {
	MOVF	(_fifo_tail + 1), W, B
	MOVWF	r0x05
	CLRF	r0x06
	MOVLW	0x00
	SUBWF	r0x06, W
	BNZ	_01189_DS_
	MOVLW	0x04
	SUBWF	r0x05, W
_01189_DS_:
	BTFSC	STATUS, 0
	BRA	_01176_DS_
	CLRF	PCLATH
	CLRF	PCLATU
	RLCF	r0x05, W
	RLCF	PCLATH, F
	RLCF	WREG, W
	RLCF	PCLATH, F
	ANDLW	0xfc
	ADDLW	LOW(_01190_DS_)
	MOVWF	POSTDEC1
	MOVLW	HIGH(_01190_DS_)
	ADDWFC	PCLATH, F
	MOVLW	UPPER(_01190_DS_)
	ADDWFC	PCLATU, F
	MOVF	PREINC1, W
	MOVWF	PCL
_01190_DS_:
	GOTO	_01172_DS_
	GOTO	_01173_DS_
	GOTO	_01174_DS_
	GOTO	_01175_DS_
_01172_DS_:
;	.line	4503; meter_logger.c	*c = fifo_buffer_0[(fifo_tail + pos) % QUEUE_SIZE];
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
;	.line	4504; meter_logger.c	break;
	BRA	_01176_DS_
_01173_DS_:
;	.line	4506; meter_logger.c	*c = fifo_buffer_1[(fifo_tail + pos) % QUEUE_SIZE];
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
;	.line	4507; meter_logger.c	break;
	BRA	_01176_DS_
_01174_DS_:
;	.line	4509; meter_logger.c	*c = fifo_buffer_2[(fifo_tail + pos) % QUEUE_SIZE];
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
;	.line	4510; meter_logger.c	break;
	BRA	_01176_DS_
_01175_DS_:
	BANKSEL	_fifo_tail
;	.line	4512; meter_logger.c	*c = fifo_buffer_3[(fifo_tail + pos) % QUEUE_SIZE];
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
_01176_DS_:
;	.line	4515; meter_logger.c	return 1;
	MOVLW	0x01
	BRA	_01180_DS_
_01178_DS_:
;	.line	4518; meter_logger.c	return 0;
	CLRF	WREG
_01180_DS_:
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
;	.line	4471; meter_logger.c	unsigned char fifo_get(unsigned char *c) {
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
;	.line	4472; meter_logger.c	if (fifo_in_use() != 0) {
	CALL	_fifo_in_use
	MOVWF	r0x03
	MOVFF	PRODL, r0x04
	MOVF	r0x03, W
	IORWF	r0x04, W
	BTFSC	STATUS, 2
	BRA	_01147_DS_
	BANKSEL	(_fifo_tail + 1)
;	.line	4473; meter_logger.c	switch (fifo_tail/QUEUE_SIZE) {
	MOVF	(_fifo_tail + 1), W, B
	MOVWF	r0x03
	CLRF	r0x04
	MOVLW	0x00
	SUBWF	r0x04, W
	BNZ	_01160_DS_
	MOVLW	0x04
	SUBWF	r0x03, W
_01160_DS_:
	BTFSC	STATUS, 0
	BRA	_01143_DS_
	CLRF	PCLATH
	CLRF	PCLATU
	RLCF	r0x03, W
	RLCF	PCLATH, F
	RLCF	WREG, W
	RLCF	PCLATH, F
	ANDLW	0xfc
	ADDLW	LOW(_01161_DS_)
	MOVWF	POSTDEC1
	MOVLW	HIGH(_01161_DS_)
	ADDWFC	PCLATH, F
	MOVLW	UPPER(_01161_DS_)
	ADDWFC	PCLATU, F
	MOVF	PREINC1, W
	MOVWF	PCL
_01161_DS_:
	GOTO	_01139_DS_
	GOTO	_01140_DS_
	GOTO	_01141_DS_
	GOTO	_01142_DS_
_01139_DS_:
	BANKSEL	_fifo_tail
;	.line	4475; meter_logger.c	*c = fifo_buffer_0[fifo_tail % QUEUE_SIZE];
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
;	.line	4476; meter_logger.c	break;
	BRA	_01143_DS_
_01140_DS_:
	BANKSEL	_fifo_tail
;	.line	4478; meter_logger.c	*c = fifo_buffer_1[fifo_tail % QUEUE_SIZE];
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
;	.line	4479; meter_logger.c	break;
	BRA	_01143_DS_
_01141_DS_:
	BANKSEL	_fifo_tail
;	.line	4481; meter_logger.c	*c = fifo_buffer_2[fifo_tail % QUEUE_SIZE];
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
;	.line	4482; meter_logger.c	break;
	BRA	_01143_DS_
_01142_DS_:
	BANKSEL	_fifo_tail
;	.line	4484; meter_logger.c	*c = fifo_buffer_3[fifo_tail % QUEUE_SIZE];
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
_01143_DS_:
	BANKSEL	_fifo_tail
;	.line	4487; meter_logger.c	fifo_tail++;
	INCFSZ	_fifo_tail, F, B
	BRA	_11236_DS_
; removed redundant BANKSEL
	INCF	(_fifo_tail + 1), F, B
_11236_DS_:
	BANKSEL	_fifo_tail
;	.line	4489; meter_logger.c	if (fifo_tail == QUEUE_SIZE_COMBINED) {
	MOVF	_fifo_tail, W, B
	BNZ	_01166_DS_
; removed redundant BANKSEL
	MOVF	(_fifo_tail + 1), W, B
	XORLW	0x04
	BZ	_01167_DS_
_01166_DS_:
	BRA	_01145_DS_
_01167_DS_:
	BANKSEL	_fifo_tail
;	.line	4490; meter_logger.c	fifo_tail = 0;
	CLRF	_fifo_tail, B
; removed redundant BANKSEL
	CLRF	(_fifo_tail + 1), B
_01145_DS_:
;	.line	4492; meter_logger.c	return 1;
	MOVLW	0x01
	BRA	_01149_DS_
_01147_DS_:
;	.line	4495; meter_logger.c	return 0;
	CLRF	WREG
_01149_DS_:
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
;	.line	4443; meter_logger.c	unsigned char fifo_put(unsigned char c) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVFF	r0x01, POSTDEC1
	MOVFF	r0x02, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
;	.line	4444; meter_logger.c	if (fifo_in_use() != QUEUE_SIZE_COMBINED) {
	CALL	_fifo_in_use
	MOVWF	r0x01
	MOVFF	PRODL, r0x02
	MOVF	r0x01, W
	BNZ	_01126_DS_
	MOVF	r0x02, W
	XORLW	0x04
	BNZ	_01126_DS_
	BRA	_01112_DS_
_01126_DS_:
	BANKSEL	(_fifo_head + 1)
;	.line	4445; meter_logger.c	switch (fifo_head/QUEUE_SIZE) {
	MOVF	(_fifo_head + 1), W, B
	MOVWF	r0x01
	CLRF	r0x02
	MOVLW	0x00
	SUBWF	r0x02, W
	BNZ	_01127_DS_
	MOVLW	0x04
	SUBWF	r0x01, W
_01127_DS_:
	BTFSC	STATUS, 0
	BRA	_01108_DS_
	CLRF	PCLATH
	CLRF	PCLATU
	RLCF	r0x01, W
	RLCF	PCLATH, F
	RLCF	WREG, W
	RLCF	PCLATH, F
	ANDLW	0xfc
	ADDLW	LOW(_01128_DS_)
	MOVWF	POSTDEC1
	MOVLW	HIGH(_01128_DS_)
	ADDWFC	PCLATH, F
	MOVLW	UPPER(_01128_DS_)
	ADDWFC	PCLATU, F
	MOVF	PREINC1, W
	MOVWF	PCL
_01128_DS_:
	GOTO	_01104_DS_
	GOTO	_01105_DS_
	GOTO	_01106_DS_
	GOTO	_01107_DS_
_01104_DS_:
	BANKSEL	_fifo_head
;	.line	4447; meter_logger.c	fifo_buffer_0[fifo_head % QUEUE_SIZE] = c;
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
;	.line	4448; meter_logger.c	break;
	BRA	_01108_DS_
_01105_DS_:
	BANKSEL	_fifo_head
;	.line	4450; meter_logger.c	fifo_buffer_1[fifo_head % QUEUE_SIZE] = c;
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
;	.line	4451; meter_logger.c	break;
	BRA	_01108_DS_
_01106_DS_:
	BANKSEL	_fifo_head
;	.line	4453; meter_logger.c	fifo_buffer_2[fifo_head % QUEUE_SIZE] = c;
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
;	.line	4454; meter_logger.c	break;
	BRA	_01108_DS_
_01107_DS_:
	BANKSEL	_fifo_head
;	.line	4456; meter_logger.c	fifo_buffer_3[fifo_head % QUEUE_SIZE] = c;
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
_01108_DS_:
	BANKSEL	_fifo_head
;	.line	4459; meter_logger.c	fifo_head++;
	INCFSZ	_fifo_head, F, B
	BRA	_21237_DS_
; removed redundant BANKSEL
	INCF	(_fifo_head + 1), F, B
_21237_DS_:
	BANKSEL	_fifo_head
;	.line	4461; meter_logger.c	if (fifo_head == QUEUE_SIZE_COMBINED) {
	MOVF	_fifo_head, W, B
	BNZ	_01133_DS_
; removed redundant BANKSEL
	MOVF	(_fifo_head + 1), W, B
	XORLW	0x04
	BZ	_01134_DS_
_01133_DS_:
	BRA	_01110_DS_
_01134_DS_:
	BANKSEL	_fifo_head
;	.line	4462; meter_logger.c	fifo_head = 0;
	CLRF	_fifo_head, B
; removed redundant BANKSEL
	CLRF	(_fifo_head + 1), B
_01110_DS_:
;	.line	4464; meter_logger.c	return 1;
	MOVLW	0x01
	BRA	_01114_DS_
_01112_DS_:
;	.line	4467; meter_logger.c	return 0;
	CLRF	WREG
_01114_DS_:
	MOVFF	PREINC1, r0x02
	MOVFF	PREINC1, r0x01
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__fifo_in_use	code
_fifo_in_use:
;	.line	4439; meter_logger.c	unsigned int fifo_in_use() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVFF	r0x01, POSTDEC1
	BANKSEL	_fifo_tail
;	.line	4440; meter_logger.c	return fifo_head - fifo_tail;
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
;	.line	4431; meter_logger.c	void fsk_tx_byte(unsigned char c) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
;	.line	4432; meter_logger.c	fsk_proto.data = c;
	MOVF	r0x00, W
	BANKSEL	(_fsk_proto + 12)
	MOVWF	(_fsk_proto + 12), B
;	.line	4433; meter_logger.c	fsk_proto.data_len = 8;
	MOVLW	0x08
; removed redundant BANKSEL
	MOVWF	(_fsk_proto + 13), B
_01091_DS_:
	BANKSEL	(_fsk_proto + 13)
;	.line	4434; meter_logger.c	while (fsk_proto.data_len) {
	MOVF	(_fsk_proto + 13), W, B
	BNZ	_01091_DS_
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__send_fsk_low	code
_send_fsk_low:
;	.line	2978; meter_logger.c	void send_fsk_low(void) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
;	.line	2979; meter_logger.c	PWM_PIN = 1;
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
	
;	.line	3186; meter_logger.c	PWM_PIN = 0;
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
	
;	.line	3393; meter_logger.c	PWM_PIN = 1;
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
	
;	.line	3600; meter_logger.c	PWM_PIN = 0;
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
	
;	.line	3807; meter_logger.c	PWM_PIN = 1;
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
	
;	.line	4014; meter_logger.c	PWM_PIN = 0;
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
	
;	.line	4221; meter_logger.c	PWM_PIN = 1;
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
	
;	.line	4428; meter_logger.c	PWM_PIN = 0;
	BCF	_PORTCbits, 1
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__send_fsk_high	code
_send_fsk_high:
;	.line	1456; meter_logger.c	void send_fsk_high(void) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
;	.line	1457; meter_logger.c	PWM_PIN = 1;
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
	
;	.line	1595; meter_logger.c	PWM_PIN = 0;
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
	
;	.line	1733; meter_logger.c	PWM_PIN = 1;
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
	
;	.line	1871; meter_logger.c	PWM_PIN = 0;
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
	
;	.line	2009; meter_logger.c	PWM_PIN = 1;
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
	
;	.line	2147; meter_logger.c	PWM_PIN = 0;
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
	
;	.line	2285; meter_logger.c	PWM_PIN = 1;
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
	
;	.line	2423; meter_logger.c	PWM_PIN = 0;
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
	
;	.line	2561; meter_logger.c	PWM_PIN = 1;
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
	
;	.line	2699; meter_logger.c	PWM_PIN = 0;
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
	
;	.line	2837; meter_logger.c	PWM_PIN = 1;
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
	
;	.line	2975; meter_logger.c	PWM_PIN = 0;
	BCF	_PORTCbits, 1
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__fsk_rx_disable	code
_fsk_rx_disable:
;	.line	1451; meter_logger.c	void fsk_rx_disable() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
;	.line	1452; meter_logger.c	PIE2bits.CMIE = 0;		// Disable comparator interrupt
	BCF	_PIE2bits, 6
	BANKSEL	_codec_type
;	.line	1453; meter_logger.c	codec_type = NONE;
	CLRF	_codec_type, B
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__fsk_rx_enable	code
_fsk_rx_enable:
;	.line	1415; meter_logger.c	void fsk_rx_enable() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
;	.line	1416; meter_logger.c	fsk_proto.state = START_BIT_WAIT;
	MOVLW	0x02
	BANKSEL	_fsk_proto
	MOVWF	_fsk_proto, B
; removed redundant BANKSEL
;	.line	1417; meter_logger.c	fsk_proto.start_bit_time = 0;
	CLRF	(_fsk_proto + 10), B
; removed redundant BANKSEL
	CLRF	(_fsk_proto + 11), B
;	.line	1419; meter_logger.c	timer0_reload = TIMER0_FSK;
	MOVLW	0x9f
	BANKSEL	_timer0_reload
	MOVWF	_timer0_reload, B
	MOVLW	0xf9
; removed redundant BANKSEL
	MOVWF	(_timer0_reload + 1), B
;	.line	1421; meter_logger.c	codec_type = FSK_RX;
	MOVLW	0x06
	BANKSEL	_codec_type
	MOVWF	_codec_type, B
;	.line	1424; meter_logger.c	T0CONbits.TMR0ON = 1;
	BSF	_T0CONbits, 7
;	.line	1425; meter_logger.c	T0CONbits.T0PS0 = 0;
	BCF	_T0CONbits, 0
;	.line	1426; meter_logger.c	T0CONbits.T0PS1 = 0;
	BCF	_T0CONbits, 1
;	.line	1427; meter_logger.c	T0CONbits.T0PS2 = 0;		// prescaler 1:2
	BCF	_T0CONbits, 2
;	.line	1428; meter_logger.c	T0CONbits.T08BIT = 0;		// use timer0 16-bit counter
	BCF	_T0CONbits, 6
;	.line	1429; meter_logger.c	T0CONbits.T0CS = 0;			// internal clock source
	BCF	_T0CONbits, 5
;	.line	1430; meter_logger.c	T0CONbits.PSA = 1;			// disable timer0 prescaler
	BSF	_T0CONbits, 3
;	.line	1431; meter_logger.c	INTCON2bits.TMR0IP = 1;		// high priority
	BSF	_INTCON2bits, 2
;	.line	1432; meter_logger.c	INTCONbits.TMR0IE = 0;		// Dont enable TMR0 Interrupt
	BCF	_INTCONbits, 5
;	.line	1435; meter_logger.c	CVRCONbits.CVREF = 0xf;	// 0V
	BSF	_CVRCONbits, 4
;	.line	1437; meter_logger.c	CVRCONbits.CVRSS = 0;	// VDD – VSS
	BCF	_CVRCONbits, 4
;	.line	1438; meter_logger.c	CVRCONbits.CVRR = 0;	// high range, 0.25 CVRSRC to 0.75 CVRSRC, with CVRSRC/32 step size
	BCF	_CVRCONbits, 5
;	.line	1439; meter_logger.c	CVRCONbits.CVR = 9;		// 2,65625 V
	MOVF	_CVRCONbits, W
	ANDLW	0xf0
	IORLW	0x09
	MOVWF	_CVRCONbits
;	.line	1440; meter_logger.c	CVRCONbits.CVROE = 0;	// Comparator VREF Output disabled, CVREF voltage is disconnected from the RA2/AN2/VREF-/CVREF pin
	BCF	_CVRCONbits, 6
;	.line	1441; meter_logger.c	CVRCONbits.CVREN = 1;	// Comparator Voltage Reference Enable bit
	BSF	_CVRCONbits, 7
;	.line	1443; meter_logger.c	CMCONbits.CM = 0x6;		// four inputs multiplexed to two comparators
	MOVF	_CMCONbits, W
	ANDLW	0xf8
	IORLW	0x06
	MOVWF	_CMCONbits
;	.line	1444; meter_logger.c	CMCONbits.CIS = 0;		// multiplexed to RA0/AN0 and RA1/AN1
	BCF	_CMCONbits, 3
;	.line	1445; meter_logger.c	CMCONbits.C1INV = 1;	// inverted output, C1 VIN+ < C1 VIN-
	BSF	_CMCONbits, 4
;	.line	1447; meter_logger.c	IPR2bits.CMIP = 1;		// high priority
	BSF	_IPR2bits, 6
;	.line	1448; meter_logger.c	PIE2bits.CMIE = 1;		// Enable comparator interrupt
	BSF	_PIE2bits, 6
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__fsk_tx_disable	code
_fsk_tx_disable:
;	.line	1409; meter_logger.c	void fsk_tx_disable() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	BANKSEL	_codec_type
;	.line	1410; meter_logger.c	codec_type = NONE;
	CLRF	_codec_type, B
;	.line	1411; meter_logger.c	T0CONbits.TMR0ON = 0;	// Disable TMR0 
	BCF	_T0CONbits, 7
;	.line	1412; meter_logger.c	PIE2bits.CMIE = 1;		// Disable comparator interrupt
	BSF	_PIE2bits, 6
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__fsk_tx_enable	code
_fsk_tx_enable:
;	.line	1391; meter_logger.c	void fsk_tx_enable() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
;	.line	1392; meter_logger.c	timer0_reload = TIMER0_FSK;
	MOVLW	0x9f
	BANKSEL	_timer0_reload
	MOVWF	_timer0_reload, B
	MOVLW	0xf9
; removed redundant BANKSEL
	MOVWF	(_timer0_reload + 1), B
	BANKSEL	_fsk_proto
;	.line	1394; meter_logger.c	fsk_proto.state = INIT_STATE;
	CLRF	_fsk_proto, B
;	.line	1395; meter_logger.c	codec_type = FSK_TX;
	MOVLW	0x07
	BANKSEL	_codec_type
	MOVWF	_codec_type, B
;	.line	1398; meter_logger.c	T0CONbits.TMR0ON = 1;
	BSF	_T0CONbits, 7
;	.line	1399; meter_logger.c	T0CONbits.T0PS0 = 0;
	BCF	_T0CONbits, 0
;	.line	1400; meter_logger.c	T0CONbits.T0PS1 = 0;
	BCF	_T0CONbits, 1
;	.line	1401; meter_logger.c	T0CONbits.T0PS2 = 0;		// prescaler 1:2
	BCF	_T0CONbits, 2
;	.line	1402; meter_logger.c	T0CONbits.T08BIT = 0;		// use timer0 16-bit counter
	BCF	_T0CONbits, 6
;	.line	1403; meter_logger.c	T0CONbits.T0CS = 0;			// internal clock source
	BCF	_T0CONbits, 5
;	.line	1404; meter_logger.c	T0CONbits.PSA = 1;			// disable timer0 prescaler
	BSF	_T0CONbits, 3
;	.line	1405; meter_logger.c	INTCON2bits.TMR0IP = 1;		// high priority
	BSF	_INTCON2bits, 2
;	.line	1406; meter_logger.c	INTCONbits.TMR0IE = 1;		// Enable TMR0 Interrupt
	BSF	_INTCONbits, 5
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__rs232_7e1_tx_byte	code
_rs232_7e1_tx_byte:
;	.line	1381; meter_logger.c	void rs232_7e1_tx_byte(unsigned char c) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
;	.line	1382; meter_logger.c	rs232_proto.data = c;
	MOVF	r0x00, W
	BANKSEL	(_rs232_proto + 2)
	MOVWF	(_rs232_proto + 2), B
;	.line	1383; meter_logger.c	rs232_proto.data_len = 7;
	MOVLW	0x07
; removed redundant BANKSEL
	MOVWF	(_rs232_proto + 3), B
;	.line	1384; meter_logger.c	T0CONbits.TMR0ON = 1;		// start timer 0
	BSF	_T0CONbits, 7
;	.line	1385; meter_logger.c	INTCONbits.TMR0IF = 1;		// enter timer interrupt handler now
	BSF	_INTCONbits, 2
_01053_DS_:
	BANKSEL	(_rs232_proto + 3)
;	.line	1386; meter_logger.c	while (rs232_proto.data_len) {
	MOVF	(_rs232_proto + 3), W, B
	BNZ	_01053_DS_
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__rs232_7e1_rx_disable	code
_rs232_7e1_rx_disable:
;	.line	1375; meter_logger.c	void rs232_7e1_rx_disable() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
;	.line	1376; meter_logger.c	INTCONbits.INT0IE = 0;		// disable ext int
	BCF	_INTCONbits, 4
	BANKSEL	_codec_type
;	.line	1377; meter_logger.c	codec_type = NONE;
	CLRF	_codec_type, B
;	.line	1378; meter_logger.c	T0CONbits.TMR0ON = 0;
	BCF	_T0CONbits, 7
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__rs232_7e1_rx_enable	code
_rs232_7e1_rx_enable:
;	.line	1351; meter_logger.c	void rs232_7e1_rx_enable(unsigned int t) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVLW	0x02
	MOVFF	PLUSW2, _timer0_reload
	MOVLW	0x03
	MOVFF	PLUSW2, (_timer0_reload + 1)
;	.line	1352; meter_logger.c	rs232_proto.state = START_BIT_WAIT;
	MOVLW	0x02
	BANKSEL	_rs232_proto
	MOVWF	_rs232_proto, B
; removed redundant BANKSEL
;	.line	1353; meter_logger.c	rs232_proto.data_len = 0;
	CLRF	(_rs232_proto + 3), B
;	.line	1357; meter_logger.c	codec_type = RS232_7E1_RX;
	MOVLW	0x04
	BANKSEL	_codec_type
	MOVWF	_codec_type, B
;	.line	1360; meter_logger.c	T0CONbits.TMR0ON = 0;
	BCF	_T0CONbits, 7
;	.line	1361; meter_logger.c	T0CONbits.T0PS0 = 0;
	BCF	_T0CONbits, 0
;	.line	1362; meter_logger.c	T0CONbits.T0PS1 = 0;
	BCF	_T0CONbits, 1
;	.line	1363; meter_logger.c	T0CONbits.T0PS2 = 0;		// prescaler 1:2
	BCF	_T0CONbits, 2
;	.line	1364; meter_logger.c	T0CONbits.T08BIT = 0;		// use timer0 16-bit counter
	BCF	_T0CONbits, 6
;	.line	1365; meter_logger.c	T0CONbits.T0CS = 0;			// internal clock source
	BCF	_T0CONbits, 5
;	.line	1366; meter_logger.c	T0CONbits.PSA = 1;			// disable timer0 prescaler
	BSF	_T0CONbits, 3
;	.line	1367; meter_logger.c	INTCON2bits.TMR0IP = 1;		// high priority
	BSF	_INTCON2bits, 2
;	.line	1368; meter_logger.c	INTCONbits.TMR0IE = 1;		// Enable TMR0 Interrupt
	BSF	_INTCONbits, 5
;	.line	1369; meter_logger.c	INTCONbits.TMR0IF = 0;
	BCF	_INTCONbits, 2
;	.line	1371; meter_logger.c	INTCONbits.INT0IE = 1;		// enable ext int
	BSF	_INTCONbits, 4
;	.line	1372; meter_logger.c	INTCON2bits.INTEDG0 = 1;	// rising edge
	BSF	_INTCON2bits, 6
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__rs232_7e1_tx_disable	code
_rs232_7e1_tx_disable:
;	.line	1345; meter_logger.c	void rs232_7e1_tx_disable() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	BANKSEL	_codec_type
;	.line	1346; meter_logger.c	codec_type = NONE;
	CLRF	_codec_type, B
;	.line	1347; meter_logger.c	IR_LED_PIN = 0;				// no need to set it to inverted idle
	BCF	_PORTBbits, 1
;	.line	1348; meter_logger.c	T0CONbits.TMR0ON = 0;
	BCF	_T0CONbits, 7
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__rs232_7e1_tx_enable	code
_rs232_7e1_tx_enable:
;	.line	1319; meter_logger.c	void rs232_7e1_tx_enable(unsigned int t) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVLW	0x02
	MOVFF	PLUSW2, _timer0_reload
	MOVLW	0x03
	MOVFF	PLUSW2, (_timer0_reload + 1)
	BANKSEL	_rs232_proto
;	.line	1322; meter_logger.c	rs232_proto.state = INIT_STATE;
	CLRF	_rs232_proto, B
; removed redundant BANKSEL
;	.line	1323; meter_logger.c	rs232_proto.data_len = 0;
	CLRF	(_rs232_proto + 3), B
;	.line	1325; meter_logger.c	IR_LED_PIN = 0;				// inverted rs232 output on ir, idle = no ir light
	BCF	_PORTBbits, 1
;	.line	1327; meter_logger.c	codec_type = RS232_7E1_TX;
	MOVLW	0x05
	BANKSEL	_codec_type
	MOVWF	_codec_type, B
;	.line	1330; meter_logger.c	T0CONbits.TMR0ON = 0;
	BCF	_T0CONbits, 7
;	.line	1331; meter_logger.c	T0CONbits.T0PS0 = 0;
	BCF	_T0CONbits, 0
;	.line	1332; meter_logger.c	T0CONbits.T0PS1 = 0;
	BCF	_T0CONbits, 1
;	.line	1333; meter_logger.c	T0CONbits.T0PS2 = 0;		// prescaler 1:2
	BCF	_T0CONbits, 2
;	.line	1334; meter_logger.c	T0CONbits.T08BIT = 0;		// use timer0 16-bit counter
	BCF	_T0CONbits, 6
;	.line	1335; meter_logger.c	T0CONbits.T0CS = 0;			// internal clock source
	BCF	_T0CONbits, 5
;	.line	1336; meter_logger.c	T0CONbits.PSA = 1;			// disable timer0 prescaler
	BSF	_T0CONbits, 3
;	.line	1337; meter_logger.c	INTCON2bits.TMR0IP = 1;		// high priority
	BSF	_INTCON2bits, 2
;	.line	1338; meter_logger.c	INTCONbits.TMR0IE = 1;		// Enable TMR0 Interrupt
	BSF	_INTCONbits, 5
;	.line	1339; meter_logger.c	INTCONbits.TMR0IF = 0;
	BCF	_INTCONbits, 2
;	.line	1342; meter_logger.c	T0CONbits.TMR0ON = 0;		// timer 0 started in rs232_8n2_tx_byte()
	BCF	_T0CONbits, 7
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__rs232_8n2_tx_byte	code
_rs232_8n2_tx_byte:
;	.line	1309; meter_logger.c	void rs232_8n2_tx_byte(unsigned char c) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
;	.line	1310; meter_logger.c	rs232_proto.data = c;
	MOVF	r0x00, W
	BANKSEL	(_rs232_proto + 2)
	MOVWF	(_rs232_proto + 2), B
;	.line	1311; meter_logger.c	rs232_proto.data_len = 8;
	MOVLW	0x08
; removed redundant BANKSEL
	MOVWF	(_rs232_proto + 3), B
;	.line	1312; meter_logger.c	T0CONbits.TMR0ON = 1;		// start timer 0
	BSF	_T0CONbits, 7
;	.line	1313; meter_logger.c	INTCONbits.TMR0IF = 1;		// enter timer interrupt handler now
	BSF	_INTCONbits, 2
_01025_DS_:
	BANKSEL	(_rs232_proto + 3)
;	.line	1314; meter_logger.c	while (rs232_proto.data_len) {
	MOVF	(_rs232_proto + 3), W, B
	BNZ	_01025_DS_
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__rs232_8n2_rx_disable	code
_rs232_8n2_rx_disable:
;	.line	1303; meter_logger.c	void rs232_8n2_rx_disable() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
;	.line	1304; meter_logger.c	INTCONbits.INT0IE = 0;		// disable ext int
	BCF	_INTCONbits, 4
	BANKSEL	_codec_type
;	.line	1305; meter_logger.c	codec_type = NONE;
	CLRF	_codec_type, B
;	.line	1306; meter_logger.c	T0CONbits.TMR0ON = 0;
	BCF	_T0CONbits, 7
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__rs232_8n2_rx_enable	code
_rs232_8n2_rx_enable:
;	.line	1279; meter_logger.c	void rs232_8n2_rx_enable(unsigned int t) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVLW	0x02
	MOVFF	PLUSW2, _timer0_reload
	MOVLW	0x03
	MOVFF	PLUSW2, (_timer0_reload + 1)
;	.line	1280; meter_logger.c	rs232_proto.state = START_BIT_WAIT;
	MOVLW	0x02
	BANKSEL	_rs232_proto
	MOVWF	_rs232_proto, B
; removed redundant BANKSEL
;	.line	1281; meter_logger.c	rs232_proto.data_len = 0;
	CLRF	(_rs232_proto + 3), B
;	.line	1285; meter_logger.c	codec_type = RS232_8N2_RX;
	MOVLW	0x02
	BANKSEL	_codec_type
	MOVWF	_codec_type, B
;	.line	1288; meter_logger.c	T0CONbits.TMR0ON = 0;
	BCF	_T0CONbits, 7
;	.line	1289; meter_logger.c	T0CONbits.T0PS0 = 0;
	BCF	_T0CONbits, 0
;	.line	1290; meter_logger.c	T0CONbits.T0PS1 = 0;
	BCF	_T0CONbits, 1
;	.line	1291; meter_logger.c	T0CONbits.T0PS2 = 0;		// prescaler 1:2
	BCF	_T0CONbits, 2
;	.line	1292; meter_logger.c	T0CONbits.T08BIT = 0;		// use timer0 16-bit counter
	BCF	_T0CONbits, 6
;	.line	1293; meter_logger.c	T0CONbits.T0CS = 0;			// internal clock source
	BCF	_T0CONbits, 5
;	.line	1294; meter_logger.c	T0CONbits.PSA = 1;			// disable timer0 prescaler
	BSF	_T0CONbits, 3
;	.line	1295; meter_logger.c	INTCON2bits.TMR0IP = 1;		// high priority
	BSF	_INTCON2bits, 2
;	.line	1296; meter_logger.c	INTCONbits.TMR0IE = 1;		// Enable TMR0 Interrupt
	BSF	_INTCONbits, 5
;	.line	1297; meter_logger.c	INTCONbits.TMR0IF = 0;
	BCF	_INTCONbits, 2
;	.line	1299; meter_logger.c	INTCONbits.INT0IE = 1;		// enable ext int
	BSF	_INTCONbits, 4
;	.line	1300; meter_logger.c	INTCON2bits.INTEDG0 = 1;	// rising edge
	BSF	_INTCON2bits, 6
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__rs232_8n2_tx_disable	code
_rs232_8n2_tx_disable:
;	.line	1273; meter_logger.c	void rs232_8n2_tx_disable() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	BANKSEL	_codec_type
;	.line	1274; meter_logger.c	codec_type = NONE;
	CLRF	_codec_type, B
;	.line	1275; meter_logger.c	IR_LED_PIN = 0;				// no need to set it to inverted idle
	BCF	_PORTBbits, 1
;	.line	1276; meter_logger.c	T0CONbits.TMR0ON = 0;
	BCF	_T0CONbits, 7
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__rs232_8n2_tx_enable	code
_rs232_8n2_tx_enable:
;	.line	1247; meter_logger.c	void rs232_8n2_tx_enable(unsigned int t) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVLW	0x02
	MOVFF	PLUSW2, _timer0_reload
	MOVLW	0x03
	MOVFF	PLUSW2, (_timer0_reload + 1)
	BANKSEL	_rs232_proto
;	.line	1250; meter_logger.c	rs232_proto.state = INIT_STATE;
	CLRF	_rs232_proto, B
; removed redundant BANKSEL
;	.line	1251; meter_logger.c	rs232_proto.data_len = 0;
	CLRF	(_rs232_proto + 3), B
;	.line	1253; meter_logger.c	IR_LED_PIN = 0;				// inverted rs232 output on ir, idle = no ir light
	BCF	_PORTBbits, 1
;	.line	1255; meter_logger.c	codec_type = RS232_8N2_TX;
	MOVLW	0x03
	BANKSEL	_codec_type
	MOVWF	_codec_type, B
;	.line	1258; meter_logger.c	T0CONbits.TMR0ON = 0;
	BCF	_T0CONbits, 7
;	.line	1259; meter_logger.c	T0CONbits.T0PS0 = 0;
	BCF	_T0CONbits, 0
;	.line	1260; meter_logger.c	T0CONbits.T0PS1 = 0;
	BCF	_T0CONbits, 1
;	.line	1261; meter_logger.c	T0CONbits.T0PS2 = 0;		// prescaler 1:2
	BCF	_T0CONbits, 2
;	.line	1262; meter_logger.c	T0CONbits.T08BIT = 0;		// use timer0 16-bit counter
	BCF	_T0CONbits, 6
;	.line	1263; meter_logger.c	T0CONbits.T0CS = 0;			// internal clock source
	BCF	_T0CONbits, 5
;	.line	1264; meter_logger.c	T0CONbits.PSA = 1;			// disable timer0 prescaler
	BSF	_T0CONbits, 3
;	.line	1265; meter_logger.c	INTCON2bits.TMR0IP = 1;		// high priority
	BSF	_INTCON2bits, 2
;	.line	1266; meter_logger.c	INTCONbits.TMR0IE = 1;		// Enable TMR0 Interrupt
	BSF	_INTCONbits, 5
;	.line	1267; meter_logger.c	INTCONbits.TMR0IF = 0;
	BCF	_INTCONbits, 2
;	.line	1270; meter_logger.c	T0CONbits.TMR0ON = 0;		// timer 0 started in rs232_8n2_tx_byte()
	BCF	_T0CONbits, 7
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__testo_ir_disable	code
_testo_ir_disable:
;	.line	1242; meter_logger.c	void testo_ir_disable() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	BANKSEL	_codec_type
;	.line	1243; meter_logger.c	codec_type = NONE;
	CLRF	_codec_type, B
;	.line	1244; meter_logger.c	INTCONbits.INT0IE = 0;		// disable ext int
	BCF	_INTCONbits, 4
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__testo_ir_enable	code
_testo_ir_enable:
;	.line	1218; meter_logger.c	void testo_ir_enable() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	BANKSEL	_testo_ir_proto
;	.line	1219; meter_logger.c	testo_ir_proto.state = INIT_STATE;
	CLRF	_testo_ir_proto, B
; removed redundant BANKSEL
;	.line	1220; meter_logger.c	testo_ir_proto.start_bit_len = 0;
	CLRF	(_testo_ir_proto + 2), B
;	.line	1222; meter_logger.c	timer0_reload = TIMER0_TESTO;
	MOVLW	0x23
	BANKSEL	_timer0_reload
	MOVWF	_timer0_reload, B
	MOVLW	0xf3
; removed redundant BANKSEL
	MOVWF	(_timer0_reload + 1), B
;	.line	1224; meter_logger.c	codec_type = TESTO;
	MOVLW	0x01
	BANKSEL	_codec_type
	MOVWF	_codec_type, B
;	.line	1227; meter_logger.c	T0CONbits.TMR0ON = 0;
	BCF	_T0CONbits, 7
;	.line	1228; meter_logger.c	T0CONbits.T0PS0 = 0;
	BCF	_T0CONbits, 0
;	.line	1229; meter_logger.c	T0CONbits.T0PS1 = 0;
	BCF	_T0CONbits, 1
;	.line	1230; meter_logger.c	T0CONbits.T0PS2 = 0;		// prescaler 1:2
	BCF	_T0CONbits, 2
;	.line	1231; meter_logger.c	T0CONbits.T08BIT = 0;		// use timer0 16-bit counter
	BCF	_T0CONbits, 6
;	.line	1232; meter_logger.c	T0CONbits.T0CS = 0;			// internal clock source
	BCF	_T0CONbits, 5
;	.line	1233; meter_logger.c	T0CONbits.PSA = 1;			// disable timer0 prescaler
	BSF	_T0CONbits, 3
;	.line	1234; meter_logger.c	INTCON2bits.TMR0IP = 1;		// high priority
	BSF	_INTCON2bits, 2
;	.line	1235; meter_logger.c	INTCONbits.TMR0IE = 1;		// Enable TMR0 Interrupt
	BSF	_INTCONbits, 5
;	.line	1236; meter_logger.c	INTCONbits.TMR0IF = 0;
	BCF	_INTCONbits, 2
;	.line	1238; meter_logger.c	INTCONbits.INT0IE = 1;		// enable ext int
	BSF	_INTCONbits, 4
;	.line	1239; meter_logger.c	INTCON2bits.INTEDG0 = 1;	// rising edge
	BSF	_INTCON2bits, 6
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__testo_valid_err_corr	code
_testo_valid_err_corr:
;	.line	1163; meter_logger.c	unsigned char testo_valid_err_corr(unsigned int c) {
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
;	.line	1170; meter_logger.c	calculated_err_corr_bit = 0;
	CLRF	r0x02
;	.line	1171; meter_logger.c	for (i = 0; i < 8; i++) {
	MOVLW	0x78
	ANDWF	r0x00, W
	MOVWF	r0x03
	CLRF	r0x04
	CLRF	r0x05
_00913_DS_:
;	.line	1172; meter_logger.c	calculated_err_corr_bit ^= (((c & 0x78) & (1 << i)) != 0);   // 0b01111000
	MOVLW	0x01
	MOVWF	r0x06
	MOVLW	0x00
	MOVWF	r0x07
	MOVF	r0x05, W
	BZ	_00955_DS_
	NEGF	WREG
	BCF	STATUS, 0
_00956_DS_:
	RLCF	r0x06, F
	RLCF	r0x07, F
	ADDLW	0x01
	BNC	_00956_DS_
_00955_DS_:
	MOVF	r0x03, W
	ANDWF	r0x06, F
	MOVF	r0x04, W
	ANDWF	r0x07, F
	MOVF	r0x06, W
	BNZ	_00958_DS_
	MOVF	r0x07, W
	BNZ	_00958_DS_
	CLRF	r0x06
	INCF	r0x06, F
	BRA	_00959_DS_
_00958_DS_:
	CLRF	r0x06
_00959_DS_:
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
;	.line	1171; meter_logger.c	for (i = 0; i < 8; i++) {
	INCF	r0x05, F
	MOVLW	0x08
	SUBWF	r0x05, W
	BNC	_00913_DS_
;	.line	1175; meter_logger.c	calculated_err_corr = calculated_err_corr << 1;
	RLNCF	r0x02, W
	ANDLW	0xfe
	MOVWF	r0x03
;	.line	1178; meter_logger.c	calculated_err_corr_bit = 0;
	CLRF	r0x02
;	.line	1179; meter_logger.c	for (i = 0; i < 8; i++) {
	MOVLW	0xe6
	ANDWF	r0x00, W
	MOVWF	r0x04
	CLRF	r0x05
	CLRF	r0x06
_00915_DS_:
;	.line	1180; meter_logger.c	calculated_err_corr_bit ^= (((c & 0xe6) & (1 << i)) != 0);   // 0b11100110
	MOVLW	0x01
	MOVWF	r0x07
	MOVLW	0x00
	MOVWF	r0x08
	MOVF	r0x06, W
	BZ	_00963_DS_
	NEGF	WREG
	BCF	STATUS, 0
_00964_DS_:
	RLCF	r0x07, F
	RLCF	r0x08, F
	ADDLW	0x01
	BNC	_00964_DS_
_00963_DS_:
	MOVF	r0x04, W
	ANDWF	r0x07, F
	MOVF	r0x05, W
	ANDWF	r0x08, F
	MOVF	r0x07, W
	BNZ	_00966_DS_
	MOVF	r0x08, W
	BNZ	_00966_DS_
	CLRF	r0x07
	INCF	r0x07, F
	BRA	_00967_DS_
_00966_DS_:
	CLRF	r0x07
_00967_DS_:
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
;	.line	1179; meter_logger.c	for (i = 0; i < 8; i++) {
	INCF	r0x06, F
	MOVLW	0x08
	SUBWF	r0x06, W
	BNC	_00915_DS_
;	.line	1182; meter_logger.c	calculated_err_corr |= calculated_err_corr_bit;
	MOVF	r0x02, W
	IORWF	r0x03, F
;	.line	1183; meter_logger.c	calculated_err_corr = calculated_err_corr << 1;
	BCF	STATUS, 0
	RLCF	r0x03, F
;	.line	1186; meter_logger.c	calculated_err_corr_bit = 0;
	CLRF	r0x02
;	.line	1187; meter_logger.c	for (i = 0; i < 8; i++) {
	MOVLW	0xd5
	ANDWF	r0x00, W
	MOVWF	r0x04
	CLRF	r0x05
	CLRF	r0x06
_00917_DS_:
;	.line	1188; meter_logger.c	calculated_err_corr_bit ^= (((c & 0xd5) & (1 << i)) != 0);   // 0b11010101
	MOVLW	0x01
	MOVWF	r0x07
	MOVLW	0x00
	MOVWF	r0x08
	MOVF	r0x06, W
	BZ	_00972_DS_
	NEGF	WREG
	BCF	STATUS, 0
_00973_DS_:
	RLCF	r0x07, F
	RLCF	r0x08, F
	ADDLW	0x01
	BNC	_00973_DS_
_00972_DS_:
	MOVF	r0x04, W
	ANDWF	r0x07, F
	MOVF	r0x05, W
	ANDWF	r0x08, F
	MOVF	r0x07, W
	BNZ	_00975_DS_
	MOVF	r0x08, W
	BNZ	_00975_DS_
	CLRF	r0x07
	INCF	r0x07, F
	BRA	_00976_DS_
_00975_DS_:
	CLRF	r0x07
_00976_DS_:
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
;	.line	1187; meter_logger.c	for (i = 0; i < 8; i++) {
	INCF	r0x06, F
	MOVLW	0x08
	SUBWF	r0x06, W
	BNC	_00917_DS_
;	.line	1190; meter_logger.c	calculated_err_corr |= calculated_err_corr_bit;
	MOVF	r0x02, W
	IORWF	r0x03, F
;	.line	1191; meter_logger.c	calculated_err_corr = calculated_err_corr << 1;
	BCF	STATUS, 0
	RLCF	r0x03, F
;	.line	1194; meter_logger.c	calculated_err_corr_bit = 0;
	CLRF	r0x02
;	.line	1195; meter_logger.c	for (i = 0; i < 8; i++) {
	MOVLW	0x8b
	ANDWF	r0x00, W
	MOVWF	r0x04
	CLRF	r0x05
	CLRF	r0x06
_00919_DS_:
;	.line	1196; meter_logger.c	calculated_err_corr_bit ^= (((c & 0x8b) & (1 << i)) != 0);   // 0b10001011
	MOVLW	0x01
	MOVWF	r0x07
	MOVLW	0x00
	MOVWF	r0x08
	MOVF	r0x06, W
	BZ	_00981_DS_
	NEGF	WREG
	BCF	STATUS, 0
_00982_DS_:
	RLCF	r0x07, F
	RLCF	r0x08, F
	ADDLW	0x01
	BNC	_00982_DS_
_00981_DS_:
	MOVF	r0x04, W
	ANDWF	r0x07, F
	MOVF	r0x05, W
	ANDWF	r0x08, F
	MOVF	r0x07, W
	BNZ	_00984_DS_
	MOVF	r0x08, W
	BNZ	_00984_DS_
	CLRF	r0x07
	INCF	r0x07, F
	BRA	_00985_DS_
_00984_DS_:
	CLRF	r0x07
_00985_DS_:
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
;	.line	1195; meter_logger.c	for (i = 0; i < 8; i++) {
	INCF	r0x06, F
	MOVLW	0x08
	SUBWF	r0x06, W
	BNC	_00919_DS_
;	.line	1198; meter_logger.c	calculated_err_corr |= calculated_err_corr_bit;
	MOVF	r0x02, W
	IORWF	r0x03, F
;	.line	1209; meter_logger.c	if ((c >> 8) == calculated_err_corr) {
	MOVF	r0x01, W
	MOVWF	r0x00
	CLRF	r0x01
	CLRF	r0x02
	MOVF	r0x00, W
	XORWF	r0x03, W
	BNZ	_00989_DS_
	MOVF	r0x01, W
	XORWF	r0x02, W
	BZ	_00990_DS_
_00989_DS_:
	BRA	_00911_DS_
_00990_DS_:
;	.line	1210; meter_logger.c	return 1;
	MOVLW	0x01
	BRA	_00921_DS_
_00911_DS_:
;	.line	1213; meter_logger.c	return 0;
	CLRF	WREG
_00921_DS_:
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
;	.line	1155; meter_logger.c	unsigned char reverse(unsigned char b) {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVFF	r0x01, POSTDEC1
	MOVFF	r0x02, POSTDEC1
	MOVFF	r0x03, POSTDEC1
	MOVLW	0x02
	MOVFF	PLUSW2, r0x00
;	.line	1157; meter_logger.c	c  = ((b >>  1) & 0x55) | ((b <<  1) & 0xaa);
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
;	.line	1158; meter_logger.c	c |= ((b >>  2) & 0x33) | ((b <<  2) & 0xcc);
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
;	.line	1159; meter_logger.c	c |= ((b >>  4) & 0x0f) | ((b <<  4) & 0xf0);
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
;	.line	1160; meter_logger.c	return(c);
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
;	.line	1125; meter_logger.c	void my_usart_open() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
;	.line	1126; meter_logger.c	SPBRG = 103;					// 8MHz => 19230 baud
	MOVLW	0x67
	MOVWF	_SPBRG
;	.line	1127; meter_logger.c	TXSTAbits.BRGH = 1;	// (0 = low speed)
	BSF	_TXSTAbits, 2
;	.line	1128; meter_logger.c	TXSTAbits.SYNC = 0;	// (0 = asynchronous)
	BCF	_TXSTAbits, 4
;	.line	1129; meter_logger.c	BAUDCONbits.BRG16 = 1;
	BSF	_BAUDCONbits, 3
;	.line	1132; meter_logger.c	RCSTAbits.SPEN = 1; // (1 = serial port enabled)
	BSF	_RCSTAbits, 7
;	.line	1135; meter_logger.c	PIE1bits.TXIE = 0; // (1 = enabled)
	BCF	_PIE1bits, 4
;	.line	1136; meter_logger.c	IPR1bits.TXIP = 0; // USART Tx on low priority interrupt
	BCF	_IPR1bits, 4
;	.line	1139; meter_logger.c	PIE1bits.RCIE = 1; // (1 = enabled)
	BSF	_PIE1bits, 5
;	.line	1140; meter_logger.c	IPR1bits.RCIP = 0; // USART Rx on low priority interrupt
	BCF	_IPR1bits, 5
;	.line	1143; meter_logger.c	TXSTAbits.TX9 = 0; // (0 = 8-bit transmit)
	BCF	_TXSTAbits, 6
;	.line	1146; meter_logger.c	RCSTAbits.RX9 = 0; // (0 = 8-bit reception)
	BCF	_RCSTAbits, 6
;	.line	1149; meter_logger.c	RCSTAbits.CREN = 1; // (1 = Enables receiver)
	BSF	_RCSTAbits, 4
;	.line	1152; meter_logger.c	TXSTAbits.TXEN = 1; // (1 = transmit enabled)
	BSF	_TXSTAbits, 5
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__get_dev_id	code
_get_dev_id:
;	.line	1108; meter_logger.c	unsigned int get_dev_id() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
	MOVFF	r0x00, POSTDEC1
	MOVFF	r0x01, POSTDEC1
	MOVFF	r0x02, POSTDEC1
	MOVFF	r0x03, POSTDEC1
	MOVFF	r0x04, POSTDEC1
	MOVFF	r0x05, POSTDEC1
;	.line	1111; meter_logger.c	TBLPTRU = __DEVID1 >> 16;
	MOVLW	0x3f
	MOVWF	_TBLPTRU
;	.line	1112; meter_logger.c	TBLPTRH = __DEVID1 >> 8;
	MOVLW	0xff
	MOVWF	_TBLPTRH
;	.line	1113; meter_logger.c	TBLPTRL = __DEVID1;
	MOVLW	0xfe
	MOVWF	_TBLPTRL
	tblrd*+
	
;	.line	1117; meter_logger.c	dev_id_low = TABLAT;
	MOVFF	_TABLAT, r0x00
	tblrd*+
	
;	.line	1121; meter_logger.c	dev_id_high = TABLAT;
	MOVFF	_TABLAT, r0x01
;	.line	1122; meter_logger.c	return ((dev_id_high << 8) + dev_id_low) & 0xffe0;	// dont return revision
	CLRF	r0x02
	MOVF	r0x01, W
	MOVWF	r0x04
	CLRF	r0x03
	CLRF	r0x05
	MOVF	r0x00, W
	ADDWF	r0x03, F
	MOVF	r0x05, W
	ADDWFC	r0x04, F
	MOVLW	0xe0
	ANDWF	r0x03, F
	MOVFF	r0x04, PRODL
	MOVF	r0x03, W
	MOVFF	PREINC1, r0x05
	MOVFF	PREINC1, r0x04
	MOVFF	PREINC1, r0x03
	MOVFF	PREINC1, r0x02
	MOVFF	PREINC1, r0x01
	MOVFF	PREINC1, r0x00
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__init_system	code
_init_system:
;	.line	1007; meter_logger.c	void init_system() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
;	.line	1009; meter_logger.c	TRIS_COMP1 = INPUT_STATE;		// as input
	BSF	_TRISAbits, 0
;	.line	1010; meter_logger.c	TRIS_COMP2 = INPUT_STATE;		// as input
	BSF	_TRISAbits, 1
;	.line	1012; meter_logger.c	TRIS_IR_PIN = INPUT_STATE;		// as input
	BSF	_TRISBbits, 0
;	.line	1014; meter_logger.c	TRIS_LED_PIN = OUTPUT_STATE;	// as output
	BCF	_TRISBbits, 4
;	.line	1015; meter_logger.c	LED_PIN = 0;					// and clear
	BCF	_PORTBbits, 4
;	.line	1017; meter_logger.c	TRIS_IR_LED_PIN = OUTPUT_STATE;	// as output
	BCF	_TRISBbits, 1
;	.line	1018; meter_logger.c	IR_LED_PIN = 0;					// and clear
	BCF	_PORTBbits, 1
;	.line	1020; meter_logger.c	TRIS_V_SENSE = INPUT_STATE;		// as input
	BSF	_TRISAbits, 5
;	.line	1022; meter_logger.c	TRIS_DEBUG_PIN = OUTPUT_STATE;	// as output
	BCF	_TRISBbits, 2
;	.line	1023; meter_logger.c	DEBUG_PIN = 0;					// and clear
	BCF	_PORTBbits, 2
;	.line	1025; meter_logger.c	TRIS_DEBUG2_PIN = OUTPUT_STATE;	// as output
	BCF	_TRISBbits, 3
;	.line	1026; meter_logger.c	DEBUG2_PIN = 0;					// and clear
	BCF	_PORTBbits, 3
;	.line	1028; meter_logger.c	TRIS_DEBUG3_PIN = OUTPUT_STATE;	// as output
	BCF	_TRISBbits, 4
;	.line	1029; meter_logger.c	DEBUG3_PIN = 0;					// and clear
	BCF	_PORTBbits, 4
;	.line	1033; meter_logger.c	TRIS_PWM_PIN = OUTPUT_STATE;	// enable output from pwm module at init
	BCF	_TRISCbits, 1
;	.line	1034; meter_logger.c	PWM_PIN = 0;					// and clear
	BCF	_PORTCbits, 1
;	.line	1037; meter_logger.c	TRIS_RX_PIN = INPUT_STATE;		// as input
	BSF	_TRISCbits, 7
;	.line	1038; meter_logger.c	TRIS_TX_PIN = OUTPUT_STATE;		// as input
	BCF	_TRISCbits, 6
;	.line	1043; meter_logger.c	T1CONbits.TMR1ON = 1;
	BSF	_T1CONbits, 0
;	.line	1044; meter_logger.c	T1CONbits.RD16 = 1;
	BSF	_T1CONbits, 7
;	.line	1045; meter_logger.c	T1CONbits.TMR1CS = 0;   // internal clock source
	BCF	_T1CONbits, 1
;	.line	1046; meter_logger.c	T1CONbits.T1OSCEN = 0;  // dont put t1 on pin
	BCF	_T1CONbits, 3
;	.line	1047; meter_logger.c	T1CONbits.T1CKPS0 = 0;
	BCF	_T1CONbits, 4
;	.line	1048; meter_logger.c	T1CONbits.T1CKPS1 = 0;
	BCF	_T1CONbits, 5
;	.line	1049; meter_logger.c	IPR1bits.TMR1IP = 0;	// low priority
	BCF	_IPR1bits, 0
;	.line	1050; meter_logger.c	PIE1bits.TMR1IE = 1;	// Ensure that TMR1 Interrupt is enabled
	BSF	_PIE1bits, 0
;	.line	1051; meter_logger.c	PIR1bits.TMR1IF = 1;	// Force Instant entry to Timer 1 Interrupt
	BSF	_PIR1bits, 0
;	.line	1082; meter_logger.c	RCONbits.IPEN = 1;
	BSF	_RCONbits, 7
;	.line	1084; meter_logger.c	INTCONbits.INT0IE = 0;		// disable ext int, enabled when ir demodulator is started
	BCF	_INTCONbits, 4
;	.line	1085; meter_logger.c	INTCON2bits.INTEDG0 = 1;	// rising edge
	BSF	_INTCON2bits, 6
;	.line	1087; meter_logger.c	INTCONbits.PEIE = 1;
	BSF	_INTCONbits, 6
;	.line	1088; meter_logger.c	INTCONbits.GIE = 1;	/* Enable Global interrupts   */	
	BSF	_INTCONbits, 7
;	.line	1093; meter_logger.c	IPR1bits.RCIP = 0;
	BCF	_IPR1bits, 5
;	.line	1094; meter_logger.c	IPR1bits.TXIP = 0;
	BCF	_IPR1bits, 4
;	.line	1105; meter_logger.c	my_usart_open();
	CALL	_my_usart_open
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_meter_logger__sleep_ms	code
_sleep_ms:
;	.line	987; meter_logger.c	void sleep_ms(unsigned int ms) {
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
;	.line	990; meter_logger.c	start_timer_1_ms = timer_1_ms;	
	MOVFF	_timer_1_ms, r0x02
	MOVFF	(_timer_1_ms + 1), r0x03
;	.line	993; meter_logger.c	do {
	MOVF	r0x02, W
	SUBLW	0xff
	MOVWF	r0x04
	MOVLW	0xff
	SUBFWB	r0x03, W
	MOVWF	r0x05
_00869_DS_:
;	.line	994; meter_logger.c	if (start_timer_1_ms <= timer_1_ms) {
	MOVF	r0x03, W
	BANKSEL	(_timer_1_ms + 1)
	SUBWF	(_timer_1_ms + 1), W, B
	BNZ	_00880_DS_
	MOVF	r0x02, W
; removed redundant BANKSEL
	SUBWF	_timer_1_ms, W, B
_00880_DS_:
	BNC	_00867_DS_
;	.line	995; meter_logger.c	diff = timer_1_ms - start_timer_1_ms;
	MOVF	r0x02, W
	BANKSEL	_timer_1_ms
	SUBWF	_timer_1_ms, W, B
	MOVWF	r0x06
	MOVF	r0x03, W
; removed redundant BANKSEL
	SUBWFB	(_timer_1_ms + 1), W, B
	MOVWF	r0x07
	BRA	_00870_DS_
_00867_DS_:
	BANKSEL	_timer_1_ms
;	.line	999; meter_logger.c	diff = 0xffff - start_timer_1_ms + timer_1_ms;
	MOVF	_timer_1_ms, W, B
	ADDWF	r0x04, W
	MOVWF	r0x06
; removed redundant BANKSEL
	MOVF	(_timer_1_ms + 1), W, B
	ADDWFC	r0x05, W
	MOVWF	r0x07
_00870_DS_:
;	.line	1001; meter_logger.c	} while (diff < ms);
	MOVF	r0x01, W
	SUBWF	r0x07, W
	BNZ	_00881_DS_
	MOVF	r0x00, W
	SUBWF	r0x06, W
_00881_DS_:
	BNC	_00869_DS_
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
;	.line	955; meter_logger.c	static void isr_low_prio(void) __interrupt 2 {
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
;	.line	958; meter_logger.c	if (PIR1bits.TMR1IF) {
	BTFSS	_PIR1bits, 0
	BRA	_00838_DS_
;	.line	959; meter_logger.c	TMR1H = (unsigned char)(TIMER1_RELOAD >> 8);    // 262,158ms @ 8MHz
	MOVLW	0xf8
	MOVWF	_TMR1H
;	.line	960; meter_logger.c	TMR1L = (unsigned char)TIMER1_RELOAD;
	MOVLW	0x53
	MOVWF	_TMR1L
;	.line	962; meter_logger.c	switch (led_flash.state) {
	MOVFF	_led_flash, r0x00
	MOVF	r0x00, W
	MOVWF	r0x01
	MOVF	r0x01, W
	BZ	_00832_DS_
	MOVF	r0x00, W
	XORLW	0x01
	BZ	_00833_DS_
	BRA	_00836_DS_
_00832_DS_:
;	.line	964; meter_logger.c	LED_PIN = 1;
	BSF	_PORTBbits, 4
;	.line	965; meter_logger.c	led_flash.state = LED_FLASH_RUNNING;
	MOVLW	0x01
	BANKSEL	_led_flash
	MOVWF	_led_flash, B
;	.line	966; meter_logger.c	break;
	BRA	_00836_DS_
_00833_DS_:
;	.line	968; meter_logger.c	if (led_flash.timer-- == 0) {
	MOVFF	(_led_flash + 1), r0x00
	DECF	r0x00, W
	MOVWF	r0x01
	MOVF	r0x01, W
	BANKSEL	(_led_flash + 1)
	MOVWF	(_led_flash + 1), B
	MOVF	r0x00, W
	BNZ	_00836_DS_
;	.line	969; meter_logger.c	LED_PIN = 0;
	BCF	_PORTBbits, 4
;	.line	970; meter_logger.c	led_flash.state = LED_FLASH_STOPPED;
	MOVLW	0x02
; removed redundant BANKSEL
	MOVWF	_led_flash, B
_00836_DS_:
	BANKSEL	_timer_1_ms
;	.line	974; meter_logger.c	timer_1_ms++;
	INCFSZ	_timer_1_ms, F, B
	BRA	_31238_DS_
; removed redundant BANKSEL
	INCF	(_timer_1_ms + 1), F, B
_31238_DS_:
;	.line	975; meter_logger.c	PIR1bits.TMR1IF = 0;    /* Clear the Timer Flag  */
	BCF	_PIR1bits, 0
_00838_DS_:
;	.line	979; meter_logger.c	if (usart_drdy()) {
	CALL	_usart_drdy
	MOVWF	r0x00
	MOVF	r0x00, W
	BZ	_00841_DS_
;	.line	981; meter_logger.c	c = usart_getc();
	CALL	_usart_getc
	MOVWF	r0x00
;	.line	982; meter_logger.c	usart_putc(c);
	MOVF	r0x00, W
	CALL	_usart_putc
_00841_DS_:
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
;	.line	477; meter_logger.c	static void isr_high_prio(void) __interrupt 1 {
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
;	.line	479; meter_logger.c	if (INTCONbits.INT0IF && INTCONbits.INT0IE) {
	BTFSS	_INTCONbits, 1
	BRA	_00398_DS_
	BTFSS	_INTCONbits, 4
	BRA	_00398_DS_
;	.line	480; meter_logger.c	timer_0 = (unsigned int)(TMR0L) | ((unsigned int)(TMR0H) << 8);
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
;	.line	481; meter_logger.c	TMR0H = (unsigned char)(timer0_reload >> 8);
	MOVF	(_timer0_reload + 1), W, B
	MOVWF	r0x00
	CLRF	r0x01
	MOVF	r0x00, W
	MOVWF	_TMR0H
; removed redundant BANKSEL
;	.line	482; meter_logger.c	TMR0L = (unsigned char)timer0_reload;
	MOVF	_timer0_reload, W, B
	MOVWF	_TMR0L
	BANKSEL	_codec_type
;	.line	484; meter_logger.c	switch (codec_type) {
	MOVF	_codec_type, W, B
	XORLW	0x01
	BZ	_00356_DS_
_00716_DS_:
	BANKSEL	_codec_type
	MOVF	_codec_type, W, B
	XORLW	0x02
	BNZ	_00718_DS_
	BRA	_00390_DS_
_00718_DS_:
	BANKSEL	_codec_type
	MOVF	_codec_type, W, B
	XORLW	0x04
	BNZ	_00720_DS_
	BRA	_00393_DS_
_00720_DS_:
	BRA	_00396_DS_
_00356_DS_:
;	.line	486; meter_logger.c	flash_led(100);
	MOVLW	0x64
	MOVWF	POSTDEC1
	CALL	_flash_led
	MOVF	POSTINC1, F
;	.line	487; meter_logger.c	switch (testo_ir_proto.state) {
	MOVFF	_testo_ir_proto, r0x00
	MOVF	r0x00, W
	MOVWF	r0x01
	MOVF	r0x01, W
	BZ	_00357_DS_
	MOVF	r0x00, W
	XORLW	0x02
	BZ	_00358_DS_
	MOVF	r0x00, W
	XORLW	0x04
	BNZ	_00726_DS_
	BRA	_00366_DS_
_00726_DS_:
	BRA	_00396_DS_
_00357_DS_:
;	.line	489; meter_logger.c	T0CONbits.TMR0ON = 1;		// Start TMR0
	BSF	_T0CONbits, 7
;	.line	490; meter_logger.c	testo_ir_proto.start_bit_len = 1;
	MOVLW	0x01
	BANKSEL	(_testo_ir_proto + 2)
	MOVWF	(_testo_ir_proto + 2), B
;	.line	491; meter_logger.c	testo_ir_proto.state = START_BIT_WAIT;
	MOVLW	0x02
; removed redundant BANKSEL
	MOVWF	_testo_ir_proto, B
;	.line	492; meter_logger.c	break;
	BRA	_00396_DS_
_00358_DS_:
	BANKSEL	_timer0_reload
;	.line	494; meter_logger.c	if ((TICK + timer0_reload - TICK_ADJ < timer_0) && (timer_0 < TICK + timer0_reload + TICK_ADJ)) {
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
	BNZ	_00727_DS_
; removed redundant BANKSEL
	MOVF	_timer_0, W, B
	SUBWF	r0x00, W
_00727_DS_:
	BC	_00363_DS_
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
	BNZ	_00728_DS_
	MOVF	r0x00, W
; removed redundant BANKSEL
	SUBWF	_timer_0, W, B
_00728_DS_:
	BC	_00363_DS_
;	.line	495; meter_logger.c	if (testo_ir_proto.start_bit_len < 2) {
	MOVLW	0x02
	BANKSEL	(_testo_ir_proto + 2)
	SUBWF	(_testo_ir_proto + 2), W, B
	BC	_00360_DS_
;	.line	496; meter_logger.c	testo_ir_proto.start_bit_len++;
	MOVFF	(_testo_ir_proto + 2), r0x00
	INCF	r0x00, F
	MOVF	r0x00, W
; removed redundant BANKSEL
	MOVWF	(_testo_ir_proto + 2), B
	BRA	_00396_DS_
_00360_DS_:
	BANKSEL	(_testo_ir_proto + 3)
;	.line	500; meter_logger.c	testo_ir_proto.data = 0;
	CLRF	(_testo_ir_proto + 3), B
; removed redundant BANKSEL
	CLRF	(_testo_ir_proto + 4), B
; removed redundant BANKSEL
;	.line	501; meter_logger.c	testo_ir_proto.data_len = 0;
	CLRF	(_testo_ir_proto + 5), B
;	.line	502; meter_logger.c	testo_ir_proto.state = DATA_WAIT;
	MOVLW	0x04
; removed redundant BANKSEL
	MOVWF	_testo_ir_proto, B
	BRA	_00396_DS_
_00363_DS_:
;	.line	507; meter_logger.c	testo_ir_proto.start_bit_len = 1;
	MOVLW	0x01
	BANKSEL	(_testo_ir_proto + 2)
	MOVWF	(_testo_ir_proto + 2), B
;	.line	508; meter_logger.c	testo_ir_proto.state = START_BIT_WAIT;
	MOVLW	0x02
; removed redundant BANKSEL
	MOVWF	_testo_ir_proto, B
;	.line	510; meter_logger.c	break;
	BRA	_00396_DS_
_00366_DS_:
;	.line	512; meter_logger.c	if (testo_ir_proto.data_len <= 12) {
	MOVLW	0x0d
	BANKSEL	(_testo_ir_proto + 5)
	SUBWF	(_testo_ir_proto + 5), W, B
	BTFSC	STATUS, 0
	BRA	_00396_DS_
	BANKSEL	_timer0_reload
;	.line	513; meter_logger.c	if (((TICK + timer0_reload - TICK_ADJ < timer_0) && (timer_0 < TICK + timer0_reload + TICK_ADJ)) || ((3 * TICK + timer0_reload - TICK_ADJ < timer_0) && (timer_0 < 3 * TICK + timer0_reload + TICK_ADJ))) {
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
	BNZ	_00731_DS_
; removed redundant BANKSEL
	MOVF	_timer_0, W, B
	SUBWF	r0x00, W
_00731_DS_:
	BC	_00382_DS_
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
	BNZ	_00732_DS_
	MOVF	r0x00, W
; removed redundant BANKSEL
	SUBWF	_timer_0, W, B
_00732_DS_:
	BNC	_00377_DS_
_00382_DS_:
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
	BNZ	_00733_DS_
; removed redundant BANKSEL
	MOVF	_timer_0, W, B
	SUBWF	r0x00, W
_00733_DS_:
	BC	_00378_DS_
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
	BNZ	_00734_DS_
	MOVF	r0x00, W
; removed redundant BANKSEL
	SUBWF	_timer_0, W, B
_00734_DS_:
	BC	_00378_DS_
_00377_DS_:
	BANKSEL	(_testo_ir_proto + 3)
;	.line	515; meter_logger.c	if ((testo_ir_proto.data & 1) != 0) {
	BTFSS	(_testo_ir_proto + 3), 0
	BRA	_00368_DS_
; removed redundant BANKSEL
;	.line	517; meter_logger.c	testo_ir_proto.data <<= 1;		// bitshift once to left
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
	BRA	_00369_DS_
_00368_DS_:
	BANKSEL	(_testo_ir_proto + 3)
;	.line	521; meter_logger.c	testo_ir_proto.data <<= 1;		// bitshift once to left
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
;	.line	522; meter_logger.c	testo_ir_proto.data |= 1;	// and set bit 0
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
_00369_DS_:
;	.line	524; meter_logger.c	testo_ir_proto.data_len++;
	MOVFF	(_testo_ir_proto + 5), r0x00
	INCF	r0x00, F
	MOVF	r0x00, W
	BANKSEL	(_testo_ir_proto + 5)
	MOVWF	(_testo_ir_proto + 5), B
	BRA	_00379_DS_
_00378_DS_:
	BANKSEL	_timer0_reload
;	.line	526; meter_logger.c	else if ((2 * TICK + timer0_reload - TICK_ADJ < timer_0) && (timer_0 < 2 * TICK + timer0_reload + TICK_ADJ)) {
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
	BNZ	_00737_DS_
; removed redundant BANKSEL
	MOVF	_timer_0, W, B
	SUBWF	r0x00, W
_00737_DS_:
	BC	_00374_DS_
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
	BNZ	_00738_DS_
	MOVF	r0x00, W
; removed redundant BANKSEL
	SUBWF	_timer_0, W, B
_00738_DS_:
	BC	_00374_DS_
	BANKSEL	(_testo_ir_proto + 3)
;	.line	528; meter_logger.c	if ((testo_ir_proto.data & 1) != 0) {
	BTFSS	(_testo_ir_proto + 3), 0
	BRA	_00371_DS_
; removed redundant BANKSEL
;	.line	530; meter_logger.c	testo_ir_proto.data <<= 1;		// bitshift once to left
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
;	.line	531; meter_logger.c	testo_ir_proto.data |= 1;	// and set bit 0
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
	BRA	_00372_DS_
_00371_DS_:
	BANKSEL	(_testo_ir_proto + 3)
;	.line	535; meter_logger.c	testo_ir_proto.data <<= 1;		// bitshift once to left
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
_00372_DS_:
;	.line	537; meter_logger.c	testo_ir_proto.data_len++;
	MOVFF	(_testo_ir_proto + 5), r0x00
	INCF	r0x00, F
	MOVF	r0x00, W
	BANKSEL	(_testo_ir_proto + 5)
	MOVWF	(_testo_ir_proto + 5), B
	BRA	_00379_DS_
_00374_DS_:
;	.line	542; meter_logger.c	testo_ir_proto.start_bit_len = 1;
	MOVLW	0x01
	BANKSEL	(_testo_ir_proto + 2)
	MOVWF	(_testo_ir_proto + 2), B
;	.line	543; meter_logger.c	testo_ir_proto.state = START_BIT_WAIT;
	MOVLW	0x02
; removed redundant BANKSEL
	MOVWF	_testo_ir_proto, B
_00379_DS_:
	BANKSEL	(_testo_ir_proto + 5)
;	.line	545; meter_logger.c	if (testo_ir_proto.data_len == 12) {
	MOVF	(_testo_ir_proto + 5), W, B
	XORLW	0x0c
	BZ	_00742_DS_
	BRA	_00396_DS_
_00742_DS_:
	BANKSEL	(_testo_ir_proto + 4)
;	.line	548; meter_logger.c	if (testo_valid_err_corr(testo_ir_proto.data & 0xffff)) {
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
	BZ	_00384_DS_
	BANKSEL	(_testo_ir_proto + 3)
;	.line	550; meter_logger.c	fifo_put(testo_ir_proto.data & 0xff);
	MOVF	(_testo_ir_proto + 3), W, B
	MOVWF	r0x00
	CLRF	r0x01
	MOVF	r0x00, W
	MOVWF	POSTDEC1
	CALL	_fifo_put
	MOVF	POSTINC1, F
;	.line	551; meter_logger.c	LED_PIN = 1;
	BSF	_PORTBbits, 4
_00384_DS_:
	BANKSEL	_testo_ir_proto
;	.line	553; meter_logger.c	testo_ir_proto.state = INIT_STATE;
	CLRF	_testo_ir_proto, B
;	.line	558; meter_logger.c	break;
	BRA	_00396_DS_
_00390_DS_:
	BANKSEL	_rs232_proto
;	.line	560; meter_logger.c	switch (rs232_proto.state) {
	MOVF	_rs232_proto, W, B
	XORLW	0x02
	BZ	_00745_DS_
	BRA	_00396_DS_
_00745_DS_:
	BANKSEL	_timer0_reload
;	.line	571; meter_logger.c	TMR0H = (unsigned char)((timer0_reload - ((0xffff - timer0_reload) >> 1)) >> 8);
	MOVF	_timer0_reload, W, B
	SUBLW	0xff
	MOVWF	r0x00
	MOVLW	0xff
; removed redundant BANKSEL
	SUBFWB	(_timer0_reload + 1), W, B
	MOVWF	r0x01
	BCF	STATUS, 0
	RRCF	r0x01, F
	RRCF	r0x00, F
	MOVF	r0x00, W
; removed redundant BANKSEL
	SUBWF	_timer0_reload, W, B
	MOVWF	r0x00
	MOVF	r0x01, W
; removed redundant BANKSEL
	SUBWFB	(_timer0_reload + 1), W, B
	MOVWF	r0x01
	MOVF	r0x01, W
	MOVWF	r0x00
	CLRF	r0x01
	MOVF	r0x00, W
	MOVWF	_TMR0H
; removed redundant BANKSEL
;	.line	572; meter_logger.c	TMR0L = (unsigned char)timer0_reload - ((0xffff - timer0_reload) >> 1);
	MOVF	_timer0_reload, W, B
	MOVWF	r0x00
; removed redundant BANKSEL
	MOVF	_timer0_reload, W, B
	SUBLW	0xff
	MOVWF	r0x01
	MOVLW	0xff
; removed redundant BANKSEL
	SUBFWB	(_timer0_reload + 1), W, B
	MOVWF	r0x02
	BCF	STATUS, 0
	RRCF	r0x02, F
	RRCF	r0x01, F
	MOVF	r0x01, W
	SUBWF	r0x00, W
	MOVWF	_TMR0L
;	.line	573; meter_logger.c	INTCONbits.INT0IE = 0;		// disable ext int while we are using timer to receive data bits
	BCF	_INTCONbits, 4
;	.line	574; meter_logger.c	T0CONbits.TMR0ON = 1;		// Start TMR0
	BSF	_T0CONbits, 7
;	.line	575; meter_logger.c	rs232_proto.state = DATA_WAIT;
	MOVLW	0x04
	BANKSEL	_rs232_proto
	MOVWF	_rs232_proto, B
;	.line	578; meter_logger.c	break;
	BRA	_00396_DS_
_00393_DS_:
	BANKSEL	_rs232_proto
;	.line	580; meter_logger.c	switch (rs232_proto.state) {
	MOVF	_rs232_proto, W, B
	XORLW	0x02
	BNZ	_00396_DS_
_00747_DS_:
	BANKSEL	_timer0_reload
;	.line	591; meter_logger.c	TMR0H = (unsigned char)((timer0_reload - ((0xffff - timer0_reload) >> 1)) >> 8);
	MOVF	_timer0_reload, W, B
	SUBLW	0xff
	MOVWF	r0x00
	MOVLW	0xff
; removed redundant BANKSEL
	SUBFWB	(_timer0_reload + 1), W, B
	MOVWF	r0x01
	BCF	STATUS, 0
	RRCF	r0x01, F
	RRCF	r0x00, F
	MOVF	r0x00, W
; removed redundant BANKSEL
	SUBWF	_timer0_reload, W, B
	MOVWF	r0x00
	MOVF	r0x01, W
; removed redundant BANKSEL
	SUBWFB	(_timer0_reload + 1), W, B
	MOVWF	r0x01
	MOVF	r0x01, W
	MOVWF	r0x00
	CLRF	r0x01
	MOVF	r0x00, W
	MOVWF	_TMR0H
; removed redundant BANKSEL
;	.line	592; meter_logger.c	TMR0L = (unsigned char)timer0_reload - ((0xffff - timer0_reload) >> 1);
	MOVF	_timer0_reload, W, B
	MOVWF	r0x00
; removed redundant BANKSEL
	MOVF	_timer0_reload, W, B
	SUBLW	0xff
	MOVWF	r0x01
	MOVLW	0xff
; removed redundant BANKSEL
	SUBFWB	(_timer0_reload + 1), W, B
	MOVWF	r0x02
	BCF	STATUS, 0
	RRCF	r0x02, F
	RRCF	r0x01, F
	MOVF	r0x01, W
	SUBWF	r0x00, W
	MOVWF	_TMR0L
;	.line	593; meter_logger.c	INTCONbits.INT0IE = 0;		// disable ext int while we are using timer to receive data bits
	BCF	_INTCONbits, 4
;	.line	594; meter_logger.c	T0CONbits.TMR0ON = 1;		// Start TMR0
	BSF	_T0CONbits, 7
;	.line	595; meter_logger.c	rs232_proto.data &= 0x7f;	// 7-bit data
	MOVLW	0x7f
	BANKSEL	(_rs232_proto + 2)
	ANDWF	(_rs232_proto + 2), W, B
	MOVWF	r0x00
	MOVF	r0x00, W
; removed redundant BANKSEL
	MOVWF	(_rs232_proto + 2), B
; removed redundant BANKSEL
;	.line	596; meter_logger.c	rs232_proto.calculated_parity = 0;
	CLRF	(_rs232_proto + 5), B
;	.line	598; meter_logger.c	rs232_proto.state = DATA_WAIT;
	MOVLW	0x04
; removed redundant BANKSEL
	MOVWF	_rs232_proto, B
_00396_DS_:
;	.line	603; meter_logger.c	INTCONbits.INT0IF = 0;	/* Clear Interrupt Flag */
	BCF	_INTCONbits, 1
_00398_DS_:
;	.line	607; meter_logger.c	if (INTCONbits.TMR0IF && INTCONbits.TMR0IE) {
	BTFSS	_INTCONbits, 2
	GOTO	_00475_DS_
	BTFSS	_INTCONbits, 5
	GOTO	_00475_DS_
	BANKSEL	(_timer0_reload + 1)
;	.line	609; meter_logger.c	TMR0H = (unsigned char)(timer0_reload >> 8);
	MOVF	(_timer0_reload + 1), W, B
	MOVWF	r0x00
	CLRF	r0x01
	MOVF	r0x00, W
	MOVWF	_TMR0H
; removed redundant BANKSEL
;	.line	610; meter_logger.c	TMR0L = (unsigned char)timer0_reload;
	MOVF	_timer0_reload, W, B
	MOVWF	_TMR0L
;	.line	612; meter_logger.c	switch (codec_type) {
	MOVLW	0x01
	BANKSEL	_codec_type
	SUBWF	_codec_type, W, B
	BTFSS	STATUS, 0
	GOTO	_00473_DS_
	MOVLW	0x08
; removed redundant BANKSEL
	SUBWF	_codec_type, W, B
	BTFSC	STATUS, 0
	GOTO	_00473_DS_
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
	ADDLW	LOW(_00751_DS_)
	MOVWF	POSTDEC1
	MOVLW	HIGH(_00751_DS_)
	ADDWFC	PCLATH, F
	MOVLW	UPPER(_00751_DS_)
	ADDWFC	PCLATU, F
	MOVF	PREINC1, W
	MOVWF	PCL
_00751_DS_:
	GOTO	_00400_DS_
	GOTO	_00412_DS_
	GOTO	_00401_DS_
	GOTO	_00433_DS_
	GOTO	_00422_DS_
	GOTO	_00446_DS_
	GOTO	_00457_DS_
_00400_DS_:
;	.line	614; meter_logger.c	T0CONbits.TMR0ON = 0;			// Stop TMR0
	BCF	_T0CONbits, 7
	BANKSEL	_testo_ir_proto
;	.line	615; meter_logger.c	testo_ir_proto.state = INIT_STATE;
	CLRF	_testo_ir_proto, B
	sleep 
;	.line	617; meter_logger.c	break;
	GOTO	_00473_DS_
_00401_DS_:
;	.line	619; meter_logger.c	switch (rs232_proto.state) {
	MOVFF	_rs232_proto, r0x00
	MOVF	r0x00, W
	MOVWF	r0x01
	MOVF	r0x01, W
	BZ	_00402_DS_
	MOVF	r0x00, W
	XORLW	0x03
	BZ	_00405_DS_
	MOVF	r0x00, W
	XORLW	0x0a
	BZ	_00409_DS_
	MOVF	r0x00, W
	XORLW	0x0b
	BZ	_00410_DS_
	GOTO	_00473_DS_
_00402_DS_:
	BANKSEL	(_rs232_proto + 3)
;	.line	621; meter_logger.c	if (rs232_proto.data_len == 8) {
	MOVF	(_rs232_proto + 3), W, B
	XORLW	0x08
	BZ	_00761_DS_
	BRA	_00473_DS_
_00761_DS_:
;	.line	622; meter_logger.c	IR_LED_PIN = 1;		// inverted rs232 output on ir, start bit = ir light
	BSF	_PORTBbits, 1
;	.line	623; meter_logger.c	rs232_proto.state = START_BIT_SENT;
	MOVLW	0x03
	BANKSEL	_rs232_proto
	MOVWF	_rs232_proto, B
;	.line	625; meter_logger.c	break;
	BRA	_00473_DS_
_00405_DS_:
;	.line	627; meter_logger.c	if (rs232_proto.data_len >= 1) {
	MOVLW	0x01
	BANKSEL	(_rs232_proto + 3)
	SUBWF	(_rs232_proto + 3), W, B
	BNC	_00407_DS_
;	.line	628; meter_logger.c	IR_LED_PIN = (rs232_proto.data & 1) == 0;	// inverted rs232 output on ir
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
;	.line	629; meter_logger.c	rs232_proto.data = rs232_proto.data >> 1;
	RRNCF	(_rs232_proto + 2), W, B
	ANDLW	0x7f
	MOVWF	r0x00
	MOVF	r0x00, W
; removed redundant BANKSEL
	MOVWF	(_rs232_proto + 2), B
;	.line	630; meter_logger.c	rs232_proto.data_len--;
	MOVFF	(_rs232_proto + 3), r0x00
	DECF	r0x00, F
	MOVF	r0x00, W
; removed redundant BANKSEL
	MOVWF	(_rs232_proto + 3), B
	BRA	_00473_DS_
_00407_DS_:
;	.line	633; meter_logger.c	IR_LED_PIN = 0;								// inverted rs232 output on ir					
	BCF	_PORTBbits, 1
;	.line	634; meter_logger.c	rs232_proto.state = STOP_BIT_SENT;
	MOVLW	0x0a
	BANKSEL	_rs232_proto
	MOVWF	_rs232_proto, B
;	.line	636; meter_logger.c	break;
	BRA	_00473_DS_
_00409_DS_:
;	.line	638; meter_logger.c	IR_LED_PIN = 0;									// inverted rs232 output on ir
	BCF	_PORTBbits, 1
;	.line	639; meter_logger.c	rs232_proto.state = STOP_BIT2_SENT;
	MOVLW	0x0b
	BANKSEL	_rs232_proto
	MOVWF	_rs232_proto, B
;	.line	640; meter_logger.c	break;
	BRA	_00473_DS_
_00410_DS_:
;	.line	642; meter_logger.c	IR_LED_PIN = 0;									// inverted rs232 output on ir
	BCF	_PORTBbits, 1
	BANKSEL	_rs232_proto
;	.line	643; meter_logger.c	rs232_proto.state = INIT_STATE;
	CLRF	_rs232_proto, B
;	.line	644; meter_logger.c	T0CONbits.TMR0ON = 0;							// stop timer 0
	BCF	_T0CONbits, 7
;	.line	647; meter_logger.c	break;
	BRA	_00473_DS_
_00412_DS_:
;	.line	649; meter_logger.c	switch (rs232_proto.state) {
	MOVFF	_rs232_proto, r0x00
	MOVF	r0x00, W
	XORLW	0x04
	BZ	_00413_DS_
	MOVF	r0x00, W
	XORLW	0x08
	BZ	_00419_DS_
	MOVF	r0x00, W
	XORLW	0x09
	BZ	_00420_DS_
	BRA	_00473_DS_
_00413_DS_:
;	.line	651; meter_logger.c	rs232_proto.data_len++;
	MOVFF	(_rs232_proto + 3), r0x00
	INCF	r0x00, F
	MOVF	r0x00, W
	BANKSEL	(_rs232_proto + 3)
	MOVWF	(_rs232_proto + 3), B
;	.line	652; meter_logger.c	if (IR_PIN) {		
	BTFSS	_PORTBbits, 0
	BRA	_00415_DS_
; removed redundant BANKSEL
;	.line	654; meter_logger.c	rs232_proto.data >>= 1;
	RRNCF	(_rs232_proto + 2), W, B
	ANDLW	0x7f
	MOVWF	r0x00
	MOVF	r0x00, W
; removed redundant BANKSEL
	MOVWF	(_rs232_proto + 2), B
	BRA	_00416_DS_
_00415_DS_:
	BANKSEL	(_rs232_proto + 2)
;	.line	666; meter_logger.c	rs232_proto.data >>= 1;
	RRNCF	(_rs232_proto + 2), W, B
	ANDLW	0x7f
	MOVWF	r0x00
	MOVF	r0x00, W
; removed redundant BANKSEL
	MOVWF	(_rs232_proto + 2), B
;	.line	667; meter_logger.c	rs232_proto.data |= 0x80;
	MOVLW	0x80
; removed redundant BANKSEL
	IORWF	(_rs232_proto + 2), W, B
	MOVWF	r0x00
	MOVF	r0x00, W
; removed redundant BANKSEL
	MOVWF	(_rs232_proto + 2), B
_00416_DS_:
;	.line	687; meter_logger.c	if (rs232_proto.data_len >= 8) {
	MOVLW	0x08
	BANKSEL	(_rs232_proto + 3)
	SUBWF	(_rs232_proto + 3), W, B
	BTFSS	STATUS, 0
	BRA	_00473_DS_
;	.line	689; meter_logger.c	rs232_proto.state = STOP_BIT_WAIT;
	MOVLW	0x08
; removed redundant BANKSEL
	MOVWF	_rs232_proto, B
;	.line	691; meter_logger.c	break;
	BRA	_00473_DS_
_00419_DS_:
;	.line	693; meter_logger.c	rs232_proto.state = STOP_BIT2_WAIT;
	MOVLW	0x09
	BANKSEL	_rs232_proto
	MOVWF	_rs232_proto, B
;	.line	694; meter_logger.c	break;
	BRA	_00473_DS_
_00420_DS_:
	BANKSEL	(_rs232_proto + 2)
;	.line	696; meter_logger.c	fifo_put(rs232_proto.data);
	MOVF	(_rs232_proto + 2), W, B
	MOVWF	POSTDEC1
	CALL	_fifo_put
	MOVF	POSTINC1, F
	BANKSEL	(_rs232_proto + 2)
;	.line	697; meter_logger.c	rs232_proto.data = 0;
	CLRF	(_rs232_proto + 2), B
; removed redundant BANKSEL
;	.line	698; meter_logger.c	rs232_proto.data_len = 0;
	CLRF	(_rs232_proto + 3), B
;	.line	699; meter_logger.c	rs232_proto.state = START_BIT_WAIT;
	MOVLW	0x02
; removed redundant BANKSEL
	MOVWF	_rs232_proto, B
;	.line	700; meter_logger.c	T0CONbits.TMR0ON = 0;
	BCF	_T0CONbits, 7
;	.line	701; meter_logger.c	INTCONbits.INT0IF = 0;		// dont enter ext int now
	BCF	_INTCONbits, 1
;	.line	702; meter_logger.c	INTCONbits.INT0IE = 1;		// enable ext int again
	BSF	_INTCONbits, 4
;	.line	705; meter_logger.c	break;
	BRA	_00473_DS_
_00422_DS_:
;	.line	707; meter_logger.c	switch (rs232_proto.state) {
	MOVFF	_rs232_proto, r0x00
	MOVF	r0x00, W
	MOVWF	r0x01
	MOVF	r0x01, W
	BZ	_00423_DS_
	MOVF	r0x00, W
	XORLW	0x03
	BZ	_00426_DS_
	MOVF	r0x00, W
	XORLW	0x06
	BNZ	_00777_DS_
	BRA	_00430_DS_
_00777_DS_:
	MOVF	r0x00, W
	XORLW	0x0a
	BNZ	_00779_DS_
	BRA	_00431_DS_
_00779_DS_:
	BRA	_00473_DS_
_00423_DS_:
	BANKSEL	(_rs232_proto + 3)
;	.line	709; meter_logger.c	if (rs232_proto.data_len == 7) {
	MOVF	(_rs232_proto + 3), W, B
	XORLW	0x07
	BZ	_00781_DS_
	BRA	_00473_DS_
_00781_DS_:
;	.line	710; meter_logger.c	IR_LED_PIN = 1;		// inverted rs232 output on ir, start bit = ir light
	BSF	_PORTBbits, 1
;	.line	711; meter_logger.c	rs232_proto.parity = rs232_proto.data & 1;
	MOVLW	0x01
	BANKSEL	(_rs232_proto + 2)
	ANDWF	(_rs232_proto + 2), W, B
	MOVWF	r0x00
	MOVF	r0x00, W
; removed redundant BANKSEL
	MOVWF	(_rs232_proto + 4), B
;	.line	712; meter_logger.c	rs232_proto.state = START_BIT_SENT;
	MOVLW	0x03
; removed redundant BANKSEL
	MOVWF	_rs232_proto, B
;	.line	714; meter_logger.c	break;
	BRA	_00473_DS_
_00426_DS_:
;	.line	716; meter_logger.c	if (rs232_proto.data_len >= 1) {
	MOVLW	0x01
	BANKSEL	(_rs232_proto + 3)
	SUBWF	(_rs232_proto + 3), W, B
	BNC	_00428_DS_
;	.line	717; meter_logger.c	IR_LED_PIN = (rs232_proto.data & 1) == 0;	// inverted rs232 output on ir
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
;	.line	718; meter_logger.c	rs232_proto.parity ^= ((rs232_proto.data >> 1) & 1);
	MOVF	(_rs232_proto + 2), W, B
	ANDLW	0x02
	RRNCF	WREG, W
	MOVWF	r0x00
; removed redundant BANKSEL
	MOVF	(_rs232_proto + 4), W, B
	MOVWF	r0x01
	MOVF	r0x01, W
	XORWF	r0x00, F
	MOVF	r0x00, W
; removed redundant BANKSEL
	MOVWF	(_rs232_proto + 4), B
; removed redundant BANKSEL
;	.line	719; meter_logger.c	rs232_proto.data = rs232_proto.data >> 1;
	RRNCF	(_rs232_proto + 2), W, B
	ANDLW	0x7f
	MOVWF	r0x00
	MOVF	r0x00, W
; removed redundant BANKSEL
	MOVWF	(_rs232_proto + 2), B
;	.line	720; meter_logger.c	rs232_proto.data_len--;
	MOVFF	(_rs232_proto + 3), r0x00
	DECF	r0x00, F
	MOVF	r0x00, W
; removed redundant BANKSEL
	MOVWF	(_rs232_proto + 3), B
	BRA	_00473_DS_
_00428_DS_:
;	.line	723; meter_logger.c	IR_LED_PIN = (rs232_proto.parity & 1) == 0;		// inverted rs232 output on ir					
	MOVLW	0x01
	BANKSEL	(_rs232_proto + 4)
	ANDWF	(_rs232_proto + 4), W, B
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
;	.line	724; meter_logger.c	rs232_proto.state = PARITY_BIT_SENT;
	MOVLW	0x06
; removed redundant BANKSEL
	MOVWF	_rs232_proto, B
;	.line	726; meter_logger.c	break;
	BRA	_00473_DS_
_00430_DS_:
;	.line	728; meter_logger.c	IR_LED_PIN = 0;									// inverted rs232 output on ir
	BCF	_PORTBbits, 1
;	.line	729; meter_logger.c	rs232_proto.state = STOP_BIT_SENT;
	MOVLW	0x0a
	BANKSEL	_rs232_proto
	MOVWF	_rs232_proto, B
;	.line	730; meter_logger.c	break;
	BRA	_00473_DS_
_00431_DS_:
;	.line	732; meter_logger.c	IR_LED_PIN = 0;									// inverted rs232 output on ir
	BCF	_PORTBbits, 1
	BANKSEL	_rs232_proto
;	.line	733; meter_logger.c	rs232_proto.state = INIT_STATE;
	CLRF	_rs232_proto, B
;	.line	734; meter_logger.c	T0CONbits.TMR0ON = 0;							// stop timer 0
	BCF	_T0CONbits, 7
;	.line	737; meter_logger.c	break;
	BRA	_00473_DS_
_00433_DS_:
;	.line	739; meter_logger.c	switch (rs232_proto.state) {
	MOVFF	_rs232_proto, r0x00
	MOVF	r0x00, W
	XORLW	0x04
	BZ	_00434_DS_
	MOVF	r0x00, W
	XORLW	0x07
	BZ	_00440_DS_
	MOVF	r0x00, W
	XORLW	0x08
	BZ	_00441_DS_
	BRA	_00473_DS_
_00434_DS_:
;	.line	741; meter_logger.c	rs232_proto.data_len++;
	MOVFF	(_rs232_proto + 3), r0x00
	INCF	r0x00, F
	MOVF	r0x00, W
	BANKSEL	(_rs232_proto + 3)
	MOVWF	(_rs232_proto + 3), B
;	.line	742; meter_logger.c	if (IR_PIN) {
	BTFSS	_PORTBbits, 0
	BRA	_00436_DS_
;	.line	744; meter_logger.c	rs232_proto.calculated_parity ^= 0;
	MOVFF	(_rs232_proto + 5), r0x00
	MOVF	r0x00, W
; removed redundant BANKSEL
	MOVWF	(_rs232_proto + 5), B
; removed redundant BANKSEL
;	.line	745; meter_logger.c	rs232_proto.data >>= 1;
	RRNCF	(_rs232_proto + 2), W, B
	ANDLW	0x7f
	MOVWF	r0x00
	MOVF	r0x00, W
; removed redundant BANKSEL
	MOVWF	(_rs232_proto + 2), B
	BRA	_00437_DS_
_00436_DS_:
;	.line	757; meter_logger.c	rs232_proto.calculated_parity ^= 1;
	MOVLW	0x01
	BANKSEL	(_rs232_proto + 5)
	XORWF	(_rs232_proto + 5), W, B
	MOVWF	r0x00
	MOVF	r0x00, W
; removed redundant BANKSEL
	MOVWF	(_rs232_proto + 5), B
; removed redundant BANKSEL
;	.line	758; meter_logger.c	rs232_proto.data >>= 1;
	RRNCF	(_rs232_proto + 2), W, B
	ANDLW	0x7f
	MOVWF	r0x00
	MOVF	r0x00, W
; removed redundant BANKSEL
	MOVWF	(_rs232_proto + 2), B
;	.line	759; meter_logger.c	rs232_proto.data |= 0x40;
	MOVLW	0x40
; removed redundant BANKSEL
	IORWF	(_rs232_proto + 2), W, B
	MOVWF	r0x00
	MOVF	r0x00, W
; removed redundant BANKSEL
	MOVWF	(_rs232_proto + 2), B
_00437_DS_:
;	.line	779; meter_logger.c	if (rs232_proto.data_len >= 7) {
	MOVLW	0x07
	BANKSEL	(_rs232_proto + 3)
	SUBWF	(_rs232_proto + 3), W, B
	BTFSS	STATUS, 0
	BRA	_00473_DS_
;	.line	781; meter_logger.c	rs232_proto.state = PARITY_BIT_WAIT;
	MOVLW	0x07
; removed redundant BANKSEL
	MOVWF	_rs232_proto, B
;	.line	783; meter_logger.c	break;
	BRA	_00473_DS_
_00440_DS_:
;	.line	785; meter_logger.c	rs232_proto.parity = IR_PIN ? 0 : 1;
	BTFSS	_PORTBbits, 0
	BRA	_00496_DS_
	CLRF	r0x00
	BRA	_00497_DS_
_00496_DS_:
	MOVLW	0x01
	MOVWF	r0x00
_00497_DS_:
	MOVF	r0x00, W
	BANKSEL	(_rs232_proto + 4)
	MOVWF	(_rs232_proto + 4), B
;	.line	786; meter_logger.c	rs232_proto.state = STOP_BIT_WAIT;
	MOVLW	0x08
; removed redundant BANKSEL
	MOVWF	_rs232_proto, B
;	.line	787; meter_logger.c	break;
	BRA	_00473_DS_
_00441_DS_:
	BANKSEL	(_rs232_proto + 5)
;	.line	789; meter_logger.c	if (rs232_proto.calculated_parity == rs232_proto.parity) {
	MOVF	(_rs232_proto + 5), W, B
; removed redundant BANKSEL
	XORWF	(_rs232_proto + 4), W, B
	BNZ	_00443_DS_
_00797_DS_:
	BANKSEL	(_rs232_proto + 2)
;	.line	790; meter_logger.c	fifo_put(rs232_proto.data);
	MOVF	(_rs232_proto + 2), W, B
	MOVWF	POSTDEC1
	CALL	_fifo_put
	MOVF	POSTINC1, F
	BRA	_00444_DS_
_00443_DS_:
	BANKSEL	(_rs232_proto + 2)
;	.line	795; meter_logger.c	fifo_put(rs232_proto.data);
	MOVF	(_rs232_proto + 2), W, B
	MOVWF	POSTDEC1
	CALL	_fifo_put
	MOVF	POSTINC1, F
_00444_DS_:
	BANKSEL	(_rs232_proto + 2)
;	.line	797; meter_logger.c	rs232_proto.data = 0;
	CLRF	(_rs232_proto + 2), B
; removed redundant BANKSEL
;	.line	798; meter_logger.c	rs232_proto.data_len = 0;
	CLRF	(_rs232_proto + 3), B
;	.line	799; meter_logger.c	rs232_proto.state = START_BIT_WAIT;
	MOVLW	0x02
; removed redundant BANKSEL
	MOVWF	_rs232_proto, B
;	.line	800; meter_logger.c	T0CONbits.TMR0ON = 0;
	BCF	_T0CONbits, 7
;	.line	801; meter_logger.c	INTCONbits.INT0IF = 0;		// dont enter ext int now
	BCF	_INTCONbits, 1
;	.line	802; meter_logger.c	INTCONbits.INT0IE = 1;		// enable ext int again
	BSF	_INTCONbits, 4
;	.line	805; meter_logger.c	break;
	BRA	_00473_DS_
_00446_DS_:
;	.line	807; meter_logger.c	switch (fsk_proto.state) {
	MOVFF	_fsk_proto, r0x00
	MOVF	r0x00, W
	XORLW	0x04
	BZ	_00447_DS_
	MOVF	r0x00, W
	XORLW	0x08
	BZ	_00455_DS_
	BRA	_00473_DS_
_00447_DS_:
;	.line	809; meter_logger.c	fsk_proto.data_len++;						
	MOVFF	(_fsk_proto + 13), r0x00
	INCF	r0x00, F
	MOVF	r0x00, W
	BANKSEL	(_fsk_proto + 13)
	MOVWF	(_fsk_proto + 13), B
;	.line	810; meter_logger.c	if (fsk_proto.data_len <= 8) {
	MOVLW	0x09
; removed redundant BANKSEL
	SUBWF	(_fsk_proto + 13), W, B
	BC	_00453_DS_
;	.line	811; meter_logger.c	if ((fsk_proto.diff > 340) && (fsk_proto.diff < 476)) {
	MOVLW	0x01
; removed redundant BANKSEL
	SUBWF	(_fsk_proto + 2), W, B
	BNZ	_00803_DS_
	MOVLW	0x55
; removed redundant BANKSEL
	SUBWF	(_fsk_proto + 1), W, B
_00803_DS_:
	BNC	_00449_DS_
	MOVLW	0x01
	BANKSEL	(_fsk_proto + 2)
	SUBWF	(_fsk_proto + 2), W, B
	BNZ	_00804_DS_
	MOVLW	0xdc
; removed redundant BANKSEL
	SUBWF	(_fsk_proto + 1), W, B
_00804_DS_:
	BC	_00449_DS_
	BANKSEL	(_fsk_proto + 12)
;	.line	814; meter_logger.c	fsk_proto.data >>= 1;
	RRNCF	(_fsk_proto + 12), W, B
	ANDLW	0x7f
	MOVWF	r0x00
	MOVF	r0x00, W
; removed redundant BANKSEL
	MOVWF	(_fsk_proto + 12), B
	BRA	_00473_DS_
_00449_DS_:
	BANKSEL	(_fsk_proto + 12)
;	.line	819; meter_logger.c	fsk_proto.data >>= 1;
	RRNCF	(_fsk_proto + 12), W, B
	ANDLW	0x7f
	MOVWF	r0x00
	MOVF	r0x00, W
; removed redundant BANKSEL
	MOVWF	(_fsk_proto + 12), B
;	.line	820; meter_logger.c	fsk_proto.data |= 0x80;
	MOVLW	0x80
; removed redundant BANKSEL
	IORWF	(_fsk_proto + 12), W, B
	MOVWF	r0x00
	MOVF	r0x00, W
; removed redundant BANKSEL
	MOVWF	(_fsk_proto + 12), B
	BRA	_00473_DS_
_00453_DS_:
;	.line	828; meter_logger.c	fsk_proto.state = STOP_BIT_WAIT;
	MOVLW	0x08
	BANKSEL	_fsk_proto
	MOVWF	_fsk_proto, B
;	.line	830; meter_logger.c	break;
	BRA	_00473_DS_
_00455_DS_:
	BANKSEL	(_fsk_proto + 12)
;	.line	833; meter_logger.c	fifo_put(fsk_proto.data);
	MOVF	(_fsk_proto + 12), W, B
	MOVWF	POSTDEC1
	CALL	_fifo_put
	MOVF	POSTINC1, F
	BANKSEL	(_fsk_proto + 12)
;	.line	834; meter_logger.c	fsk_proto.data = 0;
	CLRF	(_fsk_proto + 12), B
;	.line	835; meter_logger.c	fsk_proto.state = START_BIT_WAIT;
	MOVLW	0x02
; removed redundant BANKSEL
	MOVWF	_fsk_proto, B
;	.line	837; meter_logger.c	INTCONbits.TMR0IE = 0;						
	BCF	_INTCONbits, 5
;	.line	840; meter_logger.c	break;
	BRA	_00473_DS_
_00457_DS_:
;	.line	842; meter_logger.c	switch (fsk_proto.state) {
	MOVFF	_fsk_proto, r0x00
	MOVF	r0x00, W
	MOVWF	r0x01
	MOVF	r0x01, W
	BZ	_00458_DS_
	MOVF	r0x00, W
	XORLW	0x01
	BZ	_00461_DS_
	MOVF	r0x00, W
	XORLW	0x03
	BZ	_00462_DS_
	MOVF	r0x00, W
	XORLW	0x05
	BNZ	_00813_DS_
	BRA	_00470_DS_
_00813_DS_:
	MOVF	r0x00, W
	XORLW	0x0a
	BNZ	_00815_DS_
	BRA	_00471_DS_
_00815_DS_:
	BRA	_00473_DS_
_00458_DS_:
	BANKSEL	(_fsk_proto + 13)
;	.line	845; meter_logger.c	if (fsk_proto.data_len == 8) {
	MOVF	(_fsk_proto + 13), W, B
	XORLW	0x08
	BNZ	_00460_DS_
;	.line	846; meter_logger.c	fsk_proto.state = IDLE;
	MOVLW	0x01
	BANKSEL	_fsk_proto
	MOVWF	_fsk_proto, B
_00460_DS_:
;	.line	849; meter_logger.c	DEBUG_PIN = 0;
	BCF	_PORTBbits, 2
;	.line	851; meter_logger.c	break;
	BRA	_00473_DS_
_00461_DS_:
;	.line	853; meter_logger.c	send_fsk_low();
	CALL	_send_fsk_low
;	.line	854; meter_logger.c	fsk_proto.state = START_BIT_SENT;
	MOVLW	0x03
	BANKSEL	_fsk_proto
	MOVWF	_fsk_proto, B
;	.line	856; meter_logger.c	DEBUG_PIN = 1;
	BSF	_PORTBbits, 2
;	.line	858; meter_logger.c	break;
	BRA	_00473_DS_
_00462_DS_:
;	.line	860; meter_logger.c	if (fsk_proto.data_len--) {
	MOVFF	(_fsk_proto + 13), r0x00
	DECF	r0x00, W
	MOVWF	r0x01
	MOVF	r0x01, W
	BANKSEL	(_fsk_proto + 13)
	MOVWF	(_fsk_proto + 13), B
	MOVF	r0x00, W
	BZ	_00467_DS_
;	.line	861; meter_logger.c	if (fsk_proto.data & (0x80 >> fsk_proto.data_len)) {
	MOVLW	0x80
	MOVWF	r0x00
; removed redundant BANKSEL
	MOVF	(_fsk_proto + 13), W, B
	BZ	_00818_DS_
	NEGF	WREG
	BCF	STATUS, 0
_00819_DS_:
	RRCF	r0x00, F
	ADDLW	0x01
	BNC	_00819_DS_
_00818_DS_:
	BANKSEL	(_fsk_proto + 12)
	MOVF	(_fsk_proto + 12), W, B
	ANDWF	r0x00, F
	MOVF	r0x00, W
	BZ	_00464_DS_
;	.line	862; meter_logger.c	send_fsk_high();
	CALL	_send_fsk_high
;	.line	864; meter_logger.c	DEBUG_PIN = 0;
	BCF	_PORTBbits, 2
	BRA	_00467_DS_
_00464_DS_:
;	.line	868; meter_logger.c	send_fsk_low();
	CALL	_send_fsk_low
;	.line	870; meter_logger.c	DEBUG_PIN = 1;
	BSF	_PORTBbits, 2
_00467_DS_:
	BANKSEL	(_fsk_proto + 13)
;	.line	874; meter_logger.c	if (fsk_proto.data_len == 0) {
	MOVF	(_fsk_proto + 13), W, B
	BNZ	_00473_DS_
;	.line	875; meter_logger.c	fsk_proto.state = DATA_SENT;
	MOVLW	0x05
; removed redundant BANKSEL
	MOVWF	_fsk_proto, B
;	.line	877; meter_logger.c	break;
	BRA	_00473_DS_
_00470_DS_:
;	.line	879; meter_logger.c	send_fsk_high();
	CALL	_send_fsk_high
;	.line	880; meter_logger.c	fsk_proto.state = STOP_BIT_SENT;
	MOVLW	0x0a
	BANKSEL	_fsk_proto
	MOVWF	_fsk_proto, B
;	.line	882; meter_logger.c	DEBUG_PIN = 0;
	BCF	_PORTBbits, 2
;	.line	884; meter_logger.c	break;
	BRA	_00473_DS_
_00471_DS_:
;	.line	886; meter_logger.c	send_fsk_high();
	CALL	_send_fsk_high
	BANKSEL	_fsk_proto
;	.line	887; meter_logger.c	fsk_proto.state = INIT_STATE;
	CLRF	_fsk_proto, B
;	.line	889; meter_logger.c	DEBUG_PIN = 0;
	BCF	_PORTBbits, 2
_00473_DS_:
;	.line	896; meter_logger.c	INTCONbits.TMR0IF = 0;
	BCF	_INTCONbits, 2
_00475_DS_:
;	.line	899; meter_logger.c	if (PIR2bits.CMIF && PIE2bits.CMIE) {
	BTFSS	_PIR2bits, 6
	BRA	_00494_DS_
	BTFSS	_PIE2bits, 6
	BRA	_00494_DS_
;	.line	901; meter_logger.c	if (CMCONbits.C1OUT) {		// rising edge
	BTFSS	_CMCONbits, 6
	BRA	_00489_DS_
;	.line	902; meter_logger.c	timer_0 = (unsigned int)(TMR0L) | ((unsigned int)(TMR0H) << 8);
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
;	.line	907; meter_logger.c	DEBUG_PIN = 1;
	BSF	_PORTBbits, 2
	BANKSEL	_last_timer_0
;	.line	909; meter_logger.c	fsk_proto.diff = timer_0 - last_timer_0;
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
;	.line	910; meter_logger.c	last_timer_0 = timer_0;
	MOVFF	_timer_0, _last_timer_0
	MOVFF	(_timer_0 + 1), (_last_timer_0 + 1)
;	.line	912; meter_logger.c	if ((fsk_proto.diff > 340) && (fsk_proto.diff < 476)) {
	MOVLW	0x01
; removed redundant BANKSEL
	SUBWF	(_fsk_proto + 2), W, B
	BNZ	_00821_DS_
	MOVLW	0x55
; removed redundant BANKSEL
	SUBWF	(_fsk_proto + 1), W, B
_00821_DS_:
	BNC	_00485_DS_
	MOVLW	0x01
	BANKSEL	(_fsk_proto + 2)
	SUBWF	(_fsk_proto + 2), W, B
	BNZ	_00822_DS_
	MOVLW	0xdc
; removed redundant BANKSEL
	SUBWF	(_fsk_proto + 1), W, B
_00822_DS_:
	BC	_00485_DS_
	BANKSEL	(_fsk_proto + 1)
;	.line	913; meter_logger.c	fsk_proto.low_count += fsk_proto.diff;
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
;	.line	914; meter_logger.c	if (fsk_proto.state == START_BIT_WAIT) {
	MOVF	_fsk_proto, W, B
	XORLW	0x02
	BNZ	_00490_DS_
;	.line	915; meter_logger.c	if (fsk_proto.low_count >= 800) {								// start bit received
	MOVLW	0x03
	BANKSEL	(_fsk_proto + 6)
	SUBWF	(_fsk_proto + 6), W, B
	BNZ	_00825_DS_
	MOVLW	0x20
; removed redundant BANKSEL
	SUBWF	(_fsk_proto + 5), W, B
_00825_DS_:
	BNC	_00490_DS_
	BANKSEL	(_timer0_reload + 1)
;	.line	917; meter_logger.c	TMR0H = (unsigned char)(timer0_reload >> 8);
	MOVF	(_timer0_reload + 1), W, B
	MOVWF	r0x00
	CLRF	r0x01
	MOVF	r0x00, W
	MOVWF	_TMR0H
; removed redundant BANKSEL
;	.line	918; meter_logger.c	TMR0L = (unsigned char)timer0_reload;
	MOVF	_timer0_reload, W, B
	MOVWF	_TMR0L
	BANKSEL	(_fsk_proto + 5)
;	.line	919; meter_logger.c	fsk_proto.low_count = 0;
	CLRF	(_fsk_proto + 5), B
; removed redundant BANKSEL
	CLRF	(_fsk_proto + 6), B
; removed redundant BANKSEL
;	.line	920; meter_logger.c	fsk_proto.high_count = 0;
	CLRF	(_fsk_proto + 7), B
; removed redundant BANKSEL
	CLRF	(_fsk_proto + 8), B
; removed redundant BANKSEL
;	.line	922; meter_logger.c	fsk_proto.data_len = 0;
	CLRF	(_fsk_proto + 13), B
; removed redundant BANKSEL
;	.line	923; meter_logger.c	fsk_proto.data = 0;
	CLRF	(_fsk_proto + 12), B
;	.line	924; meter_logger.c	fsk_proto.state = DATA_WAIT;
	MOVLW	0x04
; removed redundant BANKSEL
	MOVWF	_fsk_proto, B
;	.line	925; meter_logger.c	INTCONbits.TMR0IF = 0;		// clear flag so it dont enter isr now
	BCF	_INTCONbits, 2
;	.line	926; meter_logger.c	INTCONbits.TMR0IE = 1;		// Enable TMR0 Interrupt
	BSF	_INTCONbits, 5
	BRA	_00490_DS_
_00485_DS_:
	BANKSEL	_fsk_proto
;	.line	932; meter_logger.c	if (fsk_proto.state == START_BIT_WAIT) {
	MOVF	_fsk_proto, W, B
	XORLW	0x02
	BNZ	_00482_DS_
_00827_DS_:
	BANKSEL	(_fsk_proto + 5)
;	.line	933; meter_logger.c	fsk_proto.low_count = 0;
	CLRF	(_fsk_proto + 5), B
; removed redundant BANKSEL
	CLRF	(_fsk_proto + 6), B
; removed redundant BANKSEL
;	.line	934; meter_logger.c	fsk_proto.high_count = 0;
	CLRF	(_fsk_proto + 7), B
; removed redundant BANKSEL
	CLRF	(_fsk_proto + 8), B
	BRA	_00490_DS_
_00482_DS_:
	BANKSEL	(_fsk_proto + 1)
;	.line	937; meter_logger.c	fsk_proto.high_count += fsk_proto.diff;
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
	BRA	_00490_DS_
_00489_DS_:
;	.line	943; meter_logger.c	DEBUG_PIN = 0;
	BCF	_PORTBbits, 2
_00490_DS_:
;	.line	947; meter_logger.c	PIR2bits.CMIF = 0;
	BCF	_PIR2bits, 6
_00494_DS_:
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
__str_0:
	DB	0x50, 0x72, 0x6f, 0x63, 0x65, 0x73, 0x73, 0x6f, 0x72, 0x3a, 0x20, 0x70
	DB	0x69, 0x63, 0x31, 0x38, 0x66, 0x32, 0x35, 0x35, 0x30, 0x0a, 0x0d, 0x00
; ; Starting pCode block
__str_1:
	DB	0x50, 0x72, 0x6f, 0x63, 0x65, 0x73, 0x73, 0x6f, 0x72, 0x3a, 0x20, 0x70
	DB	0x69, 0x63, 0x31, 0x38, 0x66, 0x32, 0x35, 0x35, 0x33, 0x0a, 0x0d, 0x00
; ; Starting pCode block
__str_2:
	DB	0x50, 0x72, 0x6f, 0x63, 0x65, 0x73, 0x73, 0x6f, 0x72, 0x3a, 0x20, 0x75
	DB	0x6e, 0x73, 0x75, 0x70, 0x70, 0x6f, 0x72, 0x74, 0x65, 0x64, 0x2c, 0x20
	DB	0x64, 0x65, 0x76, 0x69, 0x63, 0x65, 0x20, 0x69, 0x64, 0x3a, 0x20, 0x30
	DB	0x78, 0x25, 0x30, 0x34, 0x78, 0x0a, 0x0d, 0x00
; ; Starting pCode block
__str_3:
	DB	0x0a, 0x0d, 0x70, 0x72, 0x65, 0x73, 0x73, 0x20, 0x70, 0x72, 0x69, 0x6e
	DB	0x74, 0x20, 0x6f, 0x6e, 0x20, 0x74, 0x65, 0x73, 0x74, 0x6f, 0x0a, 0x0d
	DB	0x00
; ; Starting pCode block
__str_4:
	DB	0x3c, 0x2d, 0x20, 0x00
; ; Starting pCode block
__str_5:
	DB	0x25, 0x64, 0x20, 0x00
; ; Starting pCode block
__str_6:
	DB	0x0a, 0x0d, 0x00
; ; Starting pCode block
__str_7:
	DB	0x2d, 0x3e, 0x20, 0x00
; ; Starting pCode block
__str_8:
	DB	0x42, 0x61, 0x74, 0x74, 0x65, 0x72, 0x79, 0x3a, 0x20, 0x25, 0x64, 0x6d
	DB	0x56, 0x0a, 0x0d, 0x00


; Statistics:
; code size:	16230 (0x3f66) bytes (12.38%)
;           	 8115 (0x1fb3) words
; udata size:	 1199 (0x04af) bytes (66.91%)
; access size:	   14 (0x000e) bytes


	end
