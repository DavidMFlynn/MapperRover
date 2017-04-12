;=========================================================================================
;
;   Filename:	RoverFriver.asm
;   Date:	4/11/2017
;   File Version:	1.0d2
;
;    Author:	David M. Flynn
;    Company:	Oxford V.U.E., Inc.
;    E-Mail:	dflynn@oxfordvue.com
;    Web Site:	http://www.oxfordvue.com/
;=========================================================================================
;   Tracked Rover Drive w/ closed loop position and torque
;    acts as I2C Slave to control a tracked rover.
;
;    History:
;
; 1.0d2  4/11/2017	Working on a motor test.
; 1.0d1  2/13/2016	First code
;=========================================================================================
; Options
;
I2C_ADDRESS	EQU	0x30	; Slave default address
DefaultMinSpd	EQU	.20	;0..DefaultMaxSpd-1
DefaultMaxSpd	EQU	.255	;DefaultMinSpd+1..255
DefaultFlags	EQU	b'00000000'
;
;=========================================================================================
;=========================================================================================
; What happens next:
;
;   The system LED blinks once per second
;
;
;=========================================================================================
;
;  Communication protocols:
;    Slave to Master
;	Position Left/Right SInt32
;	Speed Left/Right SInt8
;	Torque Left/Right SInt8
;
;    Master to Slave
;	Speed Left/Right SInt8
;
;  Register addresses
;   00:3F	Device  (Read Only)
;   01:01	Revision (Read Only)
;   02..05:	M1 Position LSB..MSB   (Read/Write)
;   06..09:	M2 Position LSB..MSB   (Read/Write)
;   0A:	M1 Speed  (Read/Write)
;   0B:	M2 Speed  (Read/Write)
;   0C:	M1 Torque (Read Only)
;   0D:	M2 Torque (Read Only)
;
;=========================================================================================
;
;   Pin 1 (RA2/AN2)		n/c
;   Pin 2 (RA3/AN3) 		Left Motor Direction
;   Pin 3 (RA4/AN4)		Right Motor Direction
;   Pin 4 (RA5/MCLR*)		n/c
;   Pin 5 (GND) 		Ground
;   Pin 6 (RB0) 		n/c
;   Pin 7 (RB1/AN11/SDA1) 	SDA1 I2C Data
;   Pin 8 (RB2/AN10/RX)	Right Encoder A
;   Pin 9 (RB3/CCP1)		Left PWM CCP1
;
;   Pin 10 (RB4/AN8/SCL1)	SCL1 I2C Clock
;   Pin 11 (RB5/AN7) 		Right Encoder B
;   Pin 12 (RB6/AN5/CCP2) 	Left Encoder A
;   Pin 13 (RB7/AN6) 		Left Encoder B
;   Pin 14 (Vcc)		+5 volts
;   Pin 15 (RA6) 		System LED (active low output)
;   Pin 16 (RA7/CCP2) 		Right PWM CCP2
;   Pin 17 (RA0/AN0) 		Left Motor Current AN0
;   Pin 18 (RA1/AN1)		Right Motor Current AN1
;
;====================================================================================================
;
;
	list	p=16f1847,r=hex,w=1	; list directive to define processor
;
	nolist
	include	p16f1847.inc	; processor specific variable definitions
	list
;
	__CONFIG _CONFIG1,_FOSC_INTOSC & _WDTE_ON & _MCLRE_OFF & _IESO_OFF
;
;
; INTOSC oscillator: I/O function on CLKIN pin
; WatchDogTimerEnable enabled
; PWRT disabled
; MCLR/VPP pin function is digital input
; Program memory code protection is disabled
; Data memory code protection is disabled
; Brown-out Reset enabled
; CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin
; Internal/External Switchover mode is disabled
; Fail-Safe Clock Monitor is enabled
;
	__CONFIG _CONFIG2,_WRT_OFF & _PLLEN_ON & _LVP_OFF
;
; Write protection off
; 4x PLL disabled
; Stack Overflow or Underflow will cause a Reset
; Brown-out Reset Voltage (Vbor), low trip point selected.
; Low-voltage programming enabled
;
; '__CONFIG' directive is used to embed configuration data within .asm file.
; The lables following the directive are located in the respective .inc file.
; See respective data sheet for additional information on configuration word.
;
;
	constant	oldCode=0
	constant	useRS232=0
	constant	useI2CWDT=0
	constant	UseEEParams=1
;
#Define	_C	STATUS,C
#Define	_Z	STATUS,Z
;
;
;====================================================================================================
	nolist
	include	F1847_Macros.inc
	list
;
;    Port A bits
PortADDRBits	EQU	b'11100111'
#Define	RA0_IN	PORTA,0	;Left Motor Current AN0
#Define	RA1_IN	PORTA,1	;Right Motor Current AN1
#Define	RA2_IN	PORTA,2	;not used
#Define	M1_Dir	LATA,3	;Left Motor Direction
#Define	M2_Dir	LATA,4	;Right Motor Direction
#Define	RA5_IN	PORTA,5	;not used
LED1_Bit	EQU	6	;LED1 (Active Low Output)
#Define	SystemLED	LATA,LED1_Bit	;Output: 0=LED ON, Input: 0 = Switch Pressed
#Define	LED1_Tris	TRISA,LED1_Bit
#Define	RA7_IN	PORTA,7	;Right PWM CCP2
PortAValue	EQU	b'00000000'
;
; The only thing used on port B is I2C 1 and 2
PortBDDRBits	EQU	b'11111111'
M1EncA	EQU	6
M1EncB	EQU	7
M2EncA	EQU	2
M2EncB	EQU	5
#Define	RB0_IN	PORTB,0	;not used
#Define	RB1_IN	PORTB,1	;SDA1 I2C Data
#Define	M2_EncA	PORTB,M2EncA	;Right Encoder A
#Define	M1_PWM	LATB,3	;Left PWM CCP1
#Define	RB4_IN	PORTB,4	;SLC1 I2C Clock
#Define	M2_EncB	PORTB,M2EncB	;Right Encoder B
#Define	M1_EncA	PORTB,M1EncA	;Left Encoder A
#Define	M1_EncB	PORTB,M1EncB	;Left Encoder A
PortBValue	EQU	b'00000000'
;====================================================================================================
;====================================================================================================
;
;Constants
All_In	EQU	0xFF
All_Out	EQU	0x00
;
;OSCCON_Value	EQU	b'01110010'	;8MHz
OSCCON_Value	EQU	b'11110000'	;32MHz
;T2CON_Value	EQU	b'01001110'	;T2 On, /16 pre, /10 post
T2CON_Value	EQU	b'01001111'	;T2 On, /64 pre, /10 post
PR2_Value	EQU	.125
;
; 0.5uS res counter from 8MHz OSC
CCP1CON_Run	EQU	b'00001010'	;interupt but don't change the pin
CCP1CON_Clr	EQU	b'00001001'	;Clear output on match
CCP1CON_Set	EQU	b'00001000'	;Set output on match
;T1CON_Val	EQU	b'00000001'	;PreScale=1,Fosc/4,Timer ON
T1CON_Val	EQU	b'00100001'	;PreScale=4,Fosc/4,Timer ON
;
LEDTIME	EQU	d'100'	;1.00 seconds
LEDErrorTime	EQU	d'10'
;
;================================================================================================
;***** VARIABLE DEFINITIONS
; there are 128 bytes of ram, Bank0 0x20..0x7F, Bank1 0xA0..0xBF
; there are 256 bytes of EEPROM starting at 0x00 the EEPROM is not mapped into memory but
;  accessed through the EEADR and EEDATA registers
;================================================================================================
;  Bank0 Ram 020h-06Fh 80 Bytes
;
	cblock	0x20
;
	LED_Time	
	LED_Count		;part of tickcount timmer
	ISRScratch		;Timer tick count
	OnTheHalfCount
	M1Ticks
	M2Ticks
;
	EEAddrTemp		;EEProm address to read or write
	EEDataTemp		;Data to be writen to EEProm
;
	Timer1Lo		;1st 16 bit timer
	Timer1Hi		; one second RX timeiout
	Timer2Lo		;2nd 16 bit timer
	Timer2Hi		;
	Timer3Lo		;3rd 16 bit timer
	Timer3Hi		;GP wait timer
	Timer4Lo		;4th 16 bit timer
	Timer4Hi		; debounce timer
	I2C_Reg
	M1EncABPrev
	M1EncABCur
	M2EncABPrev
	M2EncABCur
	M1Dist:4		;SInt32
	M1CurSpeed		;100th per count
	M2Dist:4
	M2CurSpeed
	EncFlags
	abCurr
	abPrev
	Switches
;
	SysFlags
	I2CAddr
	MinSpd
	MaxSpd
;
	endc
;
;EncFlags
#Define	EncPhaseZero	EncFlags,0
;
	if useI2CWDT
TimerI2C	EQU	Timer1Lo
	endif
;
#Define	FirstRAMParam	I2CAddr
#Define	LastRAMParam	MaxSpd
;
;================================================================================================
;  Bank2 Ram 120h-16Fh 80 Bytes
;
; I2C Stuff is here
;Note: only upper 7 bits of address are used
RX_ELEMENTS	EQU	.14	; number of allowable array elements, in this case 16
TX_ELEMENTS	EQU	.14	; MotorFlagsX..CurrentPositionX+1
I2C_TX_Init_Val	EQU	0xAA	; value to load into transmit array to send to master
I2C_RX_Init_Val	EQU	0xAB	; value to load into received data array
;
	cblock	0x120   
	I2C_ARRAY_TX:TX_ELEMENTS	; array to transmit to master
	I2C_ARRAY_RX:RX_ELEMENTS 	; array to receive from master
	endc
;
;
;=======================================================================================================
;  Common Ram 70-7F same for all banks
;      except for ISR_W_Temp these are used for paramiter passing and temp vars
;=======================================================================================================
;
	cblock	0x70
	Param70
	Param71
	Param72
	Param73
	Param74
	Param75
	Param76
	Param77
	Param78
	Param79
	Param7A
	Param7B
	Param7C
	Param7D
	Param7E
	Param7F
	endc
;
#Define	INDEX_I2C	Param70	;I2C Data Pointer
#Define	TX_DataSize	Param71
#Define	GFlags	Param72
#Define	I2C_TXLocked	Param72,0	; Set/cleared by ISR, data is being sent
#Define	I2C_RXLocked	Param72,1	; Set/cleared by ISR, data is being received
#Define	I2C_NewRXData	Param72,2	; Set by ISR, The new data is here!
;
;=========================================================================================
;Conditionals
HasISR	EQU	0x80	;used to enable interupts 0x80=true 0x00=false
;
;=========================================================================================
;==============================================================================================
; ID Locations
	__idlocs	0x10d1
;
;==============================================================================================
; EEPROM locations (NV-RAM) 0x00..0x7F (offsets)
	ORG	0xF000
	de	I2C_ADDRESS	;nvI2CAddr
	de	DefaultMinSpd	;nvMinSpd
	de	DefaultMaxSpd	;nvMaxSpd
;
	cblock	0x0000
;
	nvI2CAddr
	nvMinSpd
	nvMaxSpd
;
	endc
;
#Define	nvFirstParamByte	nvI2CAddr
#Define	nvLastParamByte	nvMaxSpd
;
;
;==============================================================================================
;============================================================================================
;
;
	ORG	0x000	; processor reset vector
	CLRF	STATUS
	CLRF	PCLATH
  	goto	start	; go to beginning of program
;
;===============================================================================================
; Interupt Service Routine
;
; we loop through the interupt service routing every 0.01 seconds
;
;
	ORG	0x004	; interrupt vector location
	CLRF	BSR	; bank0
; Just in case we got here from another segment
	clrf	PCLATH	
;
;Timer 2
	btfss	PIR1,TMR2IF
	bra	TMR2_End
;
;Decrement timers until they are zero
; 
	clrf	FSR0H
	call	DecTimer1	;if timer 1 is not zero decrement
	call	DecTimer2
	call	DecTimer3
	call	DecTimer4
;
	btfss	M1Ticks,7
	incf	M1Ticks,F
	btfss	M2Ticks,7
	incf	M2Ticks,F
;--------------------------------------------------------------------
	movlb	0
	movf	OnTheHalfCount,F
	SKPNZ
	bra	OTH_Now
	decf	OnTheHalfCount,F
	bra	OTH_End
;	
OTH_Now	movlw	.50
	movwf	OnTheHalfCount
	call	OnTheHalf
OTH_End:
;-----------------------------------------------------------------
; blink LEDs
	BankSel	TRISA
	BSF	LED1_Tris	;LED off
	movlb	0
;
; Read SW1
;	BCF	SW1_Flag
;	BTFSS	SW1_In	;Button pressed?
;	BSF	SW1_Flag	; Yes, it's active low
;
;
	DECFSZ	LED_Count,F
	bra	TMR2_Done
	MOVF	LED_Time,W
	MOVWF	LED_Count
	BANKSEL	TRISA
	BCF	LED1_Tris	;Output=LED on
;
;
	MOVLB	0
TMR2_Done	BCF	PIR1,TMR2IF
TMR2_End:	
;
;==============================================================================================
;
	retfie		; return from interrupt
;
;==============================================================================================
; Things that happen every 1/2 second go here.
;
OnTheHalf:
	return
;
;==============================================================================================
;==============================================================================================
;
	include	F1847_Common.inc
	include	I2C_SLAVE.inc
;
;==============================================================================================
;**********************************************************************************************
;==============================================================================================
;
start	MOVLB	0x01	; select bank 1
	bsf	OPTION_REG,NOT_WPUEN	; disable pullups on port B
	bcf	OPTION_REG,TMR0CS	; TMR0 clock Fosc/4
	bcf	OPTION_REG,PSA	; prescaler assigned to TMR0
	bsf	OPTION_REG,PS0	;111 8mhz/4/256=7812.5hz=128uS/Ct=0.032768S/ISR
	bsf	OPTION_REG,PS1	;101 8mhz/4/64=31250hz=32uS/Ct=0.008192S/ISR
	bsf	OPTION_REG,PS2
;
	MOVLW	OSCCON_Value
	MOVWF	OSCCON
	movlw	b'00010111'	; WDT prescaler 1:65536 period is 2 sec (RESET value)
	movwf	WDTCON 	
;	
	MOVLB	0x03	; bank 3
	movlw	0x03
	movwf	ANSELA
	CLRF	ANSELB	;Digital I/O
;	
;
; setup timer 1 for 0.5uS/count
;
	movlb	0	; bank 0
	MOVLW	T1CON_Val
	MOVWF	T1CON
	bcf	T1GCON,TMR1GE	;always count
;
; Setup timer 2 for 0.01S/Interupt
	MOVLW	T2CON_Value	;Setup T2 for 100/s
	MOVWF	T2CON
	BANKSEL	PR2
	MOVLW	PR2_Value
	MOVWF	PR2
;
; setup data ports
	movlw	PortBValue
	movwf	PORTB	;init port B
	movlw	PortAValue
	movwf	PORTA
	MOVLB	0x01	; bank 1
	movlw	PortADDRBits
	movwf	TRISA
	movlw	PortBDDRBits	;setup for programer
	movwf	TRISB
;
;==========================
; Setup PWM's
;  Use timer 4 Prescale 16, PR4 0xFF
	BANKSEL	CCP1CON
	movlw	0x0C
	movwf	CCP1CON
	movwf	CCP2CON
	movlw	0x05
	movwf	CCPTMRS
	BANKSEL	TMR4
	movlw	0xFF
	movwf	PR4
	movlw	0x06	;post=0, tmr on,pre=16
	movwf	T4CON
;
; clear memory to zero
	CALL	ClearRam	;bank 0
;
;
	CLRWDT
;
	MOVLW	LEDTIME
	MOVWF	LED_Time
;
	MOVLW	0x01
	MOVWF	LED_Count
;
;
	call	CopyToRam
;
; init motor variables
;
; test for a bad I2C address
	movf	I2CAddr,W
	btfsc	WREG,0	;LSb most be clr
	MOVLW	I2C_ADDRESS
	movf	WREG,F
	SKPNZ
	MOVLW	I2C_ADDRESS
	movwf	I2CAddr
;
	CLRWDT
	call	Init_I2C	;setup I2C
;
	bsf	INTCON,PEIE	; enable periferal interupts
	bsf	INTCON,GIE	; enable interupts
;
;=========================================================================================
;*****************************************************************************************
;=========================================================================================
;  Main Loop
;
MainLoop	CLRWDT
;
	CALL	ReadEncorders
;
	CALL	I2C_Idle
	CALL	I2C_DataInturp
;
	CALL	I2C_DataSender
;
	goto	MainLoop
;
;
;=========================================================================================
;=========================================================================================
;
I2C_DataInturp	BTFSC	I2C_RXLocked	;Data is locked?
	RETURN		; Yes
	BTFSS	I2C_NewRXData	;Data is new?
	RETURN		; No
	BCF	I2C_NewRXData
;
	return
;
;==============================================================
;
; Send all stepper data to master (9 bytes)
;
I2C_DataSender	movlb	0	; bank 0
	BTFSC	I2C_TXLocked	;Locked?
	RETURN		; Yes
;
	movlw	low I2C_ARRAY_TX
	movwf	FSR0L
	movlw	high I2C_ARRAY_TX
	movwf	FSR0H
;
	movlw	low MotorFlagsX
	movwf	FSR1L
	movlw	high MotorFlagsX
	movwf	FSR1H
;
	movlw	0x09
	movwf	Param78
I2C_DataSender_L1	moviw	FSR1++
	movwi	FSR0++
	decfsz	Param78,F
	goto	I2C_DataSender_L1
;
	RETURN
;
;=========================================================================================
;=========================================================================================
;
ReadEncorders:
	movlb	0
;Stalled?
	btfsc	M1Ticks,7
	bsf	M1CurSpeed,7	;Stopped or dead slow >1.27s/count
;
	movf	PORTB,W
	movwf	Switches
	movf	M1EncABCur,W
	movwf	M1EncABPrev
	clrf	M1EncABCur
	btfsc	Switches,M1EncA
	BSF	M1EncABCur,0
	btfsc	Switches,M1EncB
	BSF	M1EncABCur,1
	movwf	M1EncABCur
	subwf	M1EncABPrev,W	;Changed?
	SKPNZ
	bra	ReadEncorders_1
	movf	M1EncABCur,W
	movwf	M1EncABPrev
	movlw	low M1Dist
	movwf	FSR0L
	movlw	high M1Dist
	movwf	FSR0H
	call	QuadCount
; Find speed
	btfss	EncPhaseZero
	bra	ReadEncorders_1
	movf	M1Ticks,W
	clrf	M1Ticks
	movwf	M1CurSpeed
;
ReadEncorders_1:
;Stalled?
	btfsc	M2Ticks,7
	bsf	M2CurSpeed,7	;Stopped or dead slow >1.27s/count
;
	movf	M2EncABCur,W
	movwf	M2EncABPrev
	clrf	M2EncABCur
	btfsc	Switches,M2EncA
	BSF	M2EncABCur,0
	btfsc	Switches,M2EncB
	BSF	M2EncABCur,1
	movwf	M2EncABCur
	subwf	M2EncABPrev,W	;Changed?
	SKPNZ
	return
	movf	M2EncABCur,W
	movwf	M2EncABPrev
	movlw	low M2Dist
	movwf	FSR0L
	movlw	high M2Dist
	movwf	FSR0H
	call	QuadCount
	btfss	EncPhaseZero
	bra	ReadEncorders_2
	movf	M2Ticks,W
	clrf	M2Ticks
	movwf	M2CurSpeed
;
ReadEncorders_2	return
;
QuadCount:
	movf	abCurr,W
	bcf	EncPhaseZero
	brw
	bra	Q_Enc0	;0
	bra	Q_Enc1	;1
	bra	Q_Enc2	;2
	movf	abPrev,W	;3
	brw
	return		;3 >> 0 : not valid
	bra	Q_Enc31	;3 >> 1 : rev
	bra	Q_Enc32	;3 >> 2 : for
;
Q_Enc0	bsf	EncPhaseZero
	movf	abPrev,W
	brw
	return
	bra	Q_Enc01	;0 >> 1 : for
	bra	Q_Enc02	;0 >> 2 : rev
	return		;0 >> 3 : not valid
;
Q_Enc1	movf	abPrev,W
	brw
	bra	Q_Enc10	;1 >> 0 : rev
	return		;1 >> 1
	return		;1 >> 0 : not valid
	bra	Q_Enc13	;1 >> 0 : for
;
Q_Enc2	movf	abPrev,W
	brw
	bra	Q_Enc20	;2 >> 0 : for
	return		;2 >> 0 : not valid
	return		;2 >> 2
	bra	Q_Enc23	;2 >> 0 : rev
;
Q_Enc20:
Q_Enc13:		
Q_Enc01:	
Q_Enc32	movlw	0x01
	subwf	INDF0,F	;M1Dist
	clrw
	incf	FSR0,F
	subwfb	INDF0,F
	incf	FSR0,F
	subwfb	INDF0,F
	incf	FSR0,F
	subwfb	INDF0,F
	return
Q_Enc23:
Q_Enc10:
Q_Enc02:
Q_Enc31	movlw	0x01
	addwf	INDF0,F
	clrw
	incf	FSR0,F
	addwfc	INDF0,F
	incf	FSR0,F
	addwfc	INDF0,F
	incf	FSR0,F
	addwfc	INDF0,F	
	return
;
;
	END
;
;