;--------------------------------------------------------
; File Created by SDCC : free open source ANSI-C Compiler
; Version 3.3.0 #8604 (Oct 27 2013) (Mac OS X x86_64)
; This file was generated Fri Mar 28 13:51:48 2014
;--------------------------------------------------------
; PIC16 port for the Microchip 16-bit core micros
;--------------------------------------------------------
	list	p=18f4620
	radix	dec
	CONFIG	OSC=INTIO67
	CONFIG	FCMEN=OFF
	CONFIG	IESO=OFF
	CONFIG	PWRT=OFF
	CONFIG	BOREN=SBORDIS
	CONFIG	BORV=3
	CONFIG	WDT=OFF
	CONFIG	WDTPS=32768
	CONFIG	CCP2MX=PORTC
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
	global	_fifo_buffer
	global	_fifo_tail
	global	_fifo_head
	global	_i
	global	_sleep_ms
	global	_sleep_tick
	global	_init_system
	global	_my_usart_open
	global	__debug
	global	_timer_0_ms
	global	_timer_1
	global	_main

;--------------------------------------------------------
; extern variables in this module
;--------------------------------------------------------
	extern	_PORTAbits
	extern	_PORTBbits
	extern	_PORTCbits
	extern	_PORTDbits
	extern	_PORTEbits
	extern	_LATAbits
	extern	_LATBbits
	extern	_LATCbits
	extern	_LATDbits
	extern	_LATEbits
	extern	_DDRAbits
	extern	_TRISAbits
	extern	_DDRBbits
	extern	_TRISBbits
	extern	_DDRCbits
	extern	_TRISCbits
	extern	_DDRDbits
	extern	_TRISDbits
	extern	_DDREbits
	extern	_TRISEbits
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
	extern	_ECCP1ASbits
	extern	_PWM1CONbits
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
	extern	_PORTA
	extern	_PORTB
	extern	_PORTC
	extern	_PORTD
	extern	_PORTE
	extern	_LATA
	extern	_LATB
	extern	_LATC
	extern	_LATD
	extern	_LATE
	extern	_DDRA
	extern	_TRISA
	extern	_DDRB
	extern	_TRISB
	extern	_DDRC
	extern	_TRISC
	extern	_DDRD
	extern	_TRISD
	extern	_DDRE
	extern	_TRISE
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
	extern	_EEADRH
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
	extern	_ECCP1AS
	extern	_PWM1CON
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
PCLATH	equ	0xffa
PCLATU	equ	0xffb
BSR	equ	0xfe0
FSR0L	equ	0xfe9
FSR0H	equ	0xfea
FSR1L	equ	0xfe1
FSR2L	equ	0xfd9
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

udata_testo_printer_emulator_0	udata
_i	res	1

udata_testo_printer_emulator_1	udata
_fifo_head	res	2

udata_testo_printer_emulator_2	udata
_fifo_tail	res	2

udata_testo_printer_emulator_3	udata
_fifo_buffer	res	100

udata_testo_printer_emulator_4	udata
_timer_0_ms	res	4

udata_testo_printer_emulator_5	udata
_timer_1	res	4

;--------------------------------------------------------
; interrupt vector
;--------------------------------------------------------

;--------------------------------------------------------
; global & static initialisations
;--------------------------------------------------------
; ; Starting pCode block for absolute section
; ;-----------------------------------------
S_testo_printer_emulator_ivec_0x1_isr_high_prio	code	0X000008
ivec_0x1_isr_high_prio:
	GOTO	_isr_high_prio

; ; Starting pCode block for absolute section
; ;-----------------------------------------
S_testo_printer_emulator_ivec_0x2_isr_low_prio	code	0X000018
ivec_0x2_isr_low_prio:
	GOTO	_isr_low_prio

; I code from now on!
; ; Starting pCode block
S_testo_printer_emulator__main	code
_main:
;	.line	22; testo_printer_emulator.c	OSCCONbits.SCS = 0x10;
	MOVF	_OSCCONbits, W
	ANDLW	0xfc
	MOVWF	_OSCCONbits
;	.line	23; testo_printer_emulator.c	OSCCONbits.IRCF = 0x7;	// 8 MHz
	MOVF	_OSCCONbits, W
	ANDLW	0x8f
	IORLW	0x70
	MOVWF	_OSCCONbits
	BANKSEL	_timer_0_ms
;	.line	26; testo_printer_emulator.c	timer_0_ms = 0;
	CLRF	_timer_0_ms, B
	BANKSEL	(_timer_0_ms + 1)
	CLRF	(_timer_0_ms + 1), B
	BANKSEL	(_timer_0_ms + 2)
	CLRF	(_timer_0_ms + 2), B
	BANKSEL	(_timer_0_ms + 3)
	CLRF	(_timer_0_ms + 3), B
;	.line	28; testo_printer_emulator.c	init_system();
	CALL	_init_system
;	.line	31; testo_printer_emulator.c	IPR1bits.RCIP = 0;
	BCF	_IPR1bits, 5
;	.line	32; testo_printer_emulator.c	IPR1bits.TXIP = 0;
	BCF	_IPR1bits, 4
;	.line	43; testo_printer_emulator.c	my_usart_open();
	CALL	_my_usart_open
;	.line	45; testo_printer_emulator.c	sleep_ms(1000);	// let stuff settle...
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
;	.line	46; testo_printer_emulator.c	usart_puts("OpenStoker starting... serial working\n\r");
	MOVLW	UPPER(__str_0)
	MOVWF	r0x02
	MOVLW	HIGH(__str_0)
	MOVWF	r0x01
	MOVLW	LOW(__str_0)
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
;	.line	48; testo_printer_emulator.c	TRISBbits.RB0 = 0x1;	// input
	BSF	_TRISBbits, 0
;	.line	49; testo_printer_emulator.c	TRISCbits.RC0 = 0x1;	// input
	BSF	_TRISCbits, 0
;	.line	50; testo_printer_emulator.c	TRISDbits.RD4 = 0x0;	// output
	BCF	_TRISDbits, 4
;	.line	51; testo_printer_emulator.c	PORTDbits.RD4 = 0;		// clear output
	BCF	_PORTDbits, 4
_00106_DS_:
;	.line	53; testo_printer_emulator.c	while (1) {
	BRA	_00106_DS_
	RETURN	

; ; Starting pCode block
S_testo_printer_emulator___debug	code
__debug:
;	.line	249; testo_printer_emulator.c	void _debug() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
;	.line	254; testo_printer_emulator.c	}
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_testo_printer_emulator__my_usart_open	code
_my_usart_open:
;	.line	219; testo_printer_emulator.c	void my_usart_open() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
;	.line	220; testo_printer_emulator.c	SPBRG = 103;					// 8MHz => 19230 baud
	MOVLW	0x67
	MOVWF	_SPBRG
;	.line	221; testo_printer_emulator.c	TXSTAbits.BRGH = 1;	// (1 = high speed)
	BSF	_TXSTAbits, 2
;	.line	222; testo_printer_emulator.c	TXSTAbits.SYNC = 0;	// (0 = asynchronous)
	BCF	_TXSTAbits, 4
;	.line	223; testo_printer_emulator.c	BAUDCONbits.BRG16 = 1;
	BSF	_BAUDCONbits, 3
;	.line	226; testo_printer_emulator.c	RCSTAbits.SPEN = 1; // (1 = serial port enabled)
	BSF	_RCSTAbits, 7
;	.line	229; testo_printer_emulator.c	PIE1bits.TXIE = 0; // (1 = enabled)
	BCF	_PIE1bits, 4
;	.line	230; testo_printer_emulator.c	IPR1bits.TXIP = 0; // USART Tx on low priority interrupt
	BCF	_IPR1bits, 4
;	.line	233; testo_printer_emulator.c	PIE1bits.RCIE = 1; // (1 = enabled)
	BSF	_PIE1bits, 5
;	.line	234; testo_printer_emulator.c	IPR1bits.RCIP = 0; // USART Rx on low priority interrupt
	BCF	_IPR1bits, 5
;	.line	237; testo_printer_emulator.c	TXSTAbits.TX9 = 0; // (0 = 8-bit transmit)
	BCF	_TXSTAbits, 6
;	.line	240; testo_printer_emulator.c	RCSTAbits.RX9 = 0; // (0 = 8-bit reception)
	BCF	_RCSTAbits, 6
;	.line	243; testo_printer_emulator.c	RCSTAbits.CREN = 1; // (1 = Enables receiver)
	BSF	_RCSTAbits, 4
;	.line	246; testo_printer_emulator.c	TXSTAbits.TXEN = 1; // (1 = transmit enabled)
	BSF	_TXSTAbits, 5
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_testo_printer_emulator__init_system	code
_init_system:
;	.line	159; testo_printer_emulator.c	void init_system() {
	MOVFF	FSR2L, POSTDEC1
	MOVFF	FSR1L, FSR2L
;	.line	161; testo_printer_emulator.c	T0CONbits.TMR0ON = 1;
	BSF	_T0CONbits, 7
;	.line	162; testo_printer_emulator.c	T0CONbits.T0PS0 = 0;
	BCF	_T0CONbits, 0
;	.line	163; testo_printer_emulator.c	T0CONbits.T0PS1 = 0;
	BCF	_T0CONbits, 1
;	.line	164; testo_printer_emulator.c	T0CONbits.T0PS2 = 0;	// prescaler 1:2
	BCF	_T0CONbits, 2
;	.line	165; testo_printer_emulator.c	T0CONbits.T08BIT = 0;   // use timer0 16-bit counter
	BCF	_T0CONbits, 6
;	.line	166; testo_printer_emulator.c	T0CONbits.T0CS = 0;             // internal clock source
	BCF	_T0CONbits, 5
;	.line	167; testo_printer_emulator.c	T0CONbits.PSA = 1;              // disable timer0 prescaler
	BSF	_T0CONbits, 3
;	.line	168; testo_printer_emulator.c	INTCON2bits.TMR0IP = 1; // high priority
	BSF	_INTCON2bits, 2
;	.line	169; testo_printer_emulator.c	INTCONbits.T0IE = 1;    // Ensure that TMR0 Interrupt is enabled
	BSF	_INTCONbits, 5
;	.line	170; testo_printer_emulator.c	INTCONbits.TMR0IF = 1;  // Force Instant entry to Timer 0 Interrupt
	BSF	_INTCONbits, 2
;	.line	173; testo_printer_emulator.c	T1CONbits.TMR1ON = 1;
	BSF	_T1CONbits, 0
;	.line	174; testo_printer_emulator.c	T1CONbits.RD16 = 1;
	BSF	_T1CONbits, 7
;	.line	175; testo_printer_emulator.c	T1CONbits.TMR1CS = 0;   // internal clock source
	BCF	_T1CONbits, 1
;	.line	176; testo_printer_emulator.c	T1CONbits.T1OSCEN = 0;  // dont put t1 on pin
	BCF	_T1CONbits, 3
;	.line	177; testo_printer_emulator.c	T1CONbits.T1CKPS0 = 0;
	BCF	_T1CONbits, 4
;	.line	178; testo_printer_emulator.c	T1CONbits.T1CKPS1 = 0;
	BCF	_T1CONbits, 5
;	.line	179; testo_printer_emulator.c	IPR1bits.TMR1IP = 0;	// low priority
	BCF	_IPR1bits, 0
;	.line	180; testo_printer_emulator.c	PIE1bits.TMR1IE = 1;	// Ensure that TMR1 Interrupt is enabled
	BSF	_PIE1bits, 0
;	.line	181; testo_printer_emulator.c	PIR1bits.TMR1IF = 1;	// Force Instant entry to Timer 1 Interrupt
	BSF	_PIR1bits, 0
;	.line	210; testo_printer_emulator.c	RCONbits.IPEN = 1;
	BSF	_RCONbits, 7
;	.line	212; testo_printer_emulator.c	INTCONbits.INT0IE = 1;		// enable ext int
	BSF	_INTCONbits, 4
;	.line	213; testo_printer_emulator.c	INTCON2bits.INTEDG0 = 0;	// on falling edge
	BCF	_INTCON2bits, 6
;	.line	215; testo_printer_emulator.c	INTCONbits.PEIE = 1;
	BSF	_INTCONbits, 6
;	.line	216; testo_printer_emulator.c	INTCONbits.GIE = 1;	/* Enable Global interrupts   */	
	BSF	_INTCONbits, 7
	MOVFF	PREINC1, FSR2L
	RETURN	

; ; Starting pCode block
S_testo_printer_emulator__sleep_tick	code
_sleep_tick:
;	.line	150; testo_printer_emulator.c	void sleep_tick(unsigned long ms) {
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
;	.line	152; testo_printer_emulator.c	start_timer_1 = timer_1;	
	MOVFF	_timer_1, r0x04
	MOVFF	(_timer_1 + 1), r0x05
	MOVFF	(_timer_1 + 2), r0x06
	MOVFF	(_timer_1 + 3), r0x07
_00157_DS_:
;	.line	155; testo_printer_emulator.c	while ( (((signed long)(timer_1 - start_timer_1) < 0) ? (-1 * (timer_1 - start_timer_1)) : (timer_1 - start_timer_1)) < ms) {
	MOVF	r0x04, W
	BANKSEL	_timer_1
	SUBWF	_timer_1, W, B
	MOVWF	r0x08
	MOVF	r0x05, W
	BANKSEL	(_timer_1 + 1)
	SUBWFB	(_timer_1 + 1), W, B
	MOVWF	r0x09
	MOVF	r0x06, W
	BANKSEL	(_timer_1 + 2)
	SUBWFB	(_timer_1 + 2), W, B
	MOVWF	r0x0a
	MOVF	r0x07, W
	BANKSEL	(_timer_1 + 3)
	SUBWFB	(_timer_1 + 3), W, B
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
	BNC	_00162_DS_
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
	BRA	_00163_DS_
_00162_DS_:
	MOVFF	r0x08, r0x0c
	MOVFF	r0x09, r0x0d
	MOVFF	r0x0a, r0x0e
	MOVFF	r0x0b, r0x0f
_00163_DS_:
	MOVF	r0x03, W
	SUBWF	r0x0f, W
	BNZ	_00170_DS_
	MOVF	r0x02, W
	SUBWF	r0x0e, W
	BNZ	_00170_DS_
	MOVF	r0x01, W
	SUBWF	r0x0d, W
	BNZ	_00170_DS_
	MOVF	r0x00, W
	SUBWF	r0x0c, W
_00170_DS_:
	BTFSS	STATUS, 0
	BRA	_00157_DS_
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
S_testo_printer_emulator__sleep_ms	code
_sleep_ms:
;	.line	140; testo_printer_emulator.c	void sleep_ms(unsigned long ms) {
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
;	.line	142; testo_printer_emulator.c	start_timer_0_ms = timer_0_ms;	
	MOVFF	_timer_0_ms, r0x04
	MOVFF	(_timer_0_ms + 1), r0x05
	MOVFF	(_timer_0_ms + 2), r0x06
	MOVFF	(_timer_0_ms + 3), r0x07
_00139_DS_:
;	.line	145; testo_printer_emulator.c	while ( (((signed long)(timer_0_ms - start_timer_0_ms) < 0) ? (-1 * (timer_0_ms - start_timer_0_ms)) : (timer_0_ms - start_timer_0_ms)) < ms) {
	MOVF	r0x04, W
	BANKSEL	_timer_0_ms
	SUBWF	_timer_0_ms, W, B
	MOVWF	r0x08
	MOVF	r0x05, W
	BANKSEL	(_timer_0_ms + 1)
	SUBWFB	(_timer_0_ms + 1), W, B
	MOVWF	r0x09
	MOVF	r0x06, W
	BANKSEL	(_timer_0_ms + 2)
	SUBWFB	(_timer_0_ms + 2), W, B
	MOVWF	r0x0a
	MOVF	r0x07, W
	BANKSEL	(_timer_0_ms + 3)
	SUBWFB	(_timer_0_ms + 3), W, B
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
	BNC	_00144_DS_
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
	BRA	_00145_DS_
_00144_DS_:
	MOVFF	r0x08, r0x0c
	MOVFF	r0x09, r0x0d
	MOVFF	r0x0a, r0x0e
	MOVFF	r0x0b, r0x0f
_00145_DS_:
	MOVF	r0x03, W
	SUBWF	r0x0f, W
	BNZ	_00152_DS_
	MOVF	r0x02, W
	SUBWF	r0x0e, W
	BNZ	_00152_DS_
	MOVF	r0x01, W
	SUBWF	r0x0d, W
	BNZ	_00152_DS_
	MOVF	r0x00, W
	SUBWF	r0x0c, W
_00152_DS_:
	BTFSS	STATUS, 0
	BRA	_00139_DS_
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
S_testo_printer_emulator__isr_low_prio	code
_isr_low_prio:
;	.line	121; testo_printer_emulator.c	static void isr_low_prio(void) __interrupt 2 {
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
;	.line	124; testo_printer_emulator.c	if (PIR1bits.TMR1IF) {
	BTFSS	_PIR1bits, 0
	BRA	_00123_DS_
;	.line	125; testo_printer_emulator.c	TMR1H = (unsigned char)(TIMER1_RELOAD >> 8);    // 262,158ms @ 8MHz
	MOVLW	0xf8
	MOVWF	_TMR1H
;	.line	126; testo_printer_emulator.c	TMR1L = (unsigned char)TIMER1_RELOAD;
	MOVLW	0x53
	MOVWF	_TMR1L
;	.line	127; testo_printer_emulator.c	PIR1bits.TMR1IF = 0;    /* Clear the Timer Flag  */
	BCF	_PIR1bits, 0
	BANKSEL	_timer_0_ms
;	.line	128; testo_printer_emulator.c	timer_0_ms++;
	INCF	_timer_0_ms, F, B
	BNC	_00123_DS_
	BANKSEL	(_timer_0_ms + 1)
	INCF	(_timer_0_ms + 1), F, B
	BNC	_00123_DS_
	BANKSEL	(_timer_0_ms + 2)
	INCFSZ	(_timer_0_ms + 2), F, B
	BRA	_10185_DS_
	BANKSEL	(_timer_0_ms + 3)
	INCF	(_timer_0_ms + 3), F, B
_10185_DS_:
_00134_DS_:
_00123_DS_:
;	.line	132; testo_printer_emulator.c	if (usart_drdy()) {
	CALL	_usart_drdy
	MOVWF	r0x00
	MOVF	r0x00, W
	BZ	_00126_DS_
;	.line	134; testo_printer_emulator.c	c = usart_getc();
	CALL	_usart_getc
	MOVWF	r0x00
;	.line	135; testo_printer_emulator.c	usart_putc(c);
	MOVF	r0x00, W
	CALL	_usart_putc
_00126_DS_:
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
S_testo_printer_emulator__isr_high_prio	code
_isr_high_prio:
;	.line	65; testo_printer_emulator.c	static void isr_high_prio(void) __interrupt 1 {
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
;	.line	66; testo_printer_emulator.c	if (INTCONbits.INT0IF) {
	BTFSS	_INTCONbits, 1
	BRA	_00114_DS_
;	.line	67; testo_printer_emulator.c	INTCONbits.INT0IF = 0;	/* Clear Interrupt Flag */
	BCF	_INTCONbits, 1
;	.line	73; testo_printer_emulator.c	PORTDbits.RD4 = 0x1;
	BSF	_PORTDbits, 4
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	
;	.line	111; testo_printer_emulator.c	PORTDbits.RD4 = 0x0;
	BCF	_PORTDbits, 4
_00114_DS_:
;	.line	114; testo_printer_emulator.c	if (INTCONbits.TMR0IF) {
	BTFSS	_INTCONbits, 2
	BRA	_00117_DS_
;	.line	115; testo_printer_emulator.c	TMR0H = (unsigned char)(TIMER0_RELOAD >> 8);
	MOVLW	0xfc
	MOVWF	_TMR0H
;	.line	116; testo_printer_emulator.c	TMR0L = (unsigned char)TIMER0_RELOAD;   /* Reload the Timer ASAP */
	MOVLW	0xc8
	MOVWF	_TMR0L
;	.line	117; testo_printer_emulator.c	INTCONbits.TMR0IF = 0;  /* Clear the Timer Flag  */
	BCF	_INTCONbits, 2
_00117_DS_:
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
	DB	0x4f, 0x70, 0x65, 0x6e, 0x53, 0x74, 0x6f, 0x6b, 0x65, 0x72, 0x20, 0x73
	DB	0x74, 0x61, 0x72, 0x74, 0x69, 0x6e, 0x67, 0x2e, 0x2e, 0x2e, 0x20, 0x73
	DB	0x65, 0x72, 0x69, 0x61, 0x6c, 0x20, 0x77, 0x6f, 0x72, 0x6b, 0x69, 0x6e
	DB	0x67, 0x0a, 0x0d, 0x00


; Statistics:
; code size:	 1232 (0x04d0) bytes ( 0.94%)
;           	  616 (0x0268) words
; udata size:	  113 (0x0071) bytes ( 2.94%)
; access size:	   16 (0x0010) bytes


	end
