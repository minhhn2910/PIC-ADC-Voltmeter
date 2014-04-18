			#include	p18f4520.inc
			radix		dec

			#define		LCD_D 		PORTD
			#define		LCD_D4 		LATD,RD0
			#define		LCD_D5 		LATD,RD1
			#define		LCD_D6 		LATD,RD2
			#define		LCD_D7 		LATD,RD3
			
			#define		LCD_D4_DIR 	TRISD,RD0
			#define		LCD_D5_DIR 	TRISD,RD1
			#define		LCD_D6_DIR 	TRISD,RD2
			#define		LCD_D7_DIR 	TRISD,RD3	; input and output

			#define		LCD_ON 		LATD,RD7
			#define		LCD_E 		LATD,RD6
			#define		LCD_RW 		LATD,RD5
			#define		LCD_RS 		LATD,RD4

			#define		LCD_ON_DIR 	TRISD,RD7
			#define		LCD_E_DIR 	TRISD,RD6
			#define		LCD_RW_DIR 	TRISD,RD5
			#define		LCD_RS_DIR 	TRISD,RD4


			code		0
			goto		MAIN

			org			0x08
			goto		ISR_INTERRUPT0

			org			0x18
			goto		ISR_TMR0

			udata
delay		res			1
delay_1		res			1
command		res			1
data_4bit	res			1
row			res			1
col			res			1
addr		res			1
char		res			1
decode_var	res			1

milisecond res 1
second res 1
minute res 1

W_temp res 1

status res 1

bcd_convert res 1

PRG			code
MAIN		
			movlw 0x30
			daw
			call INIT
			BSF	T0CON,TMR0ON	;start Timer0
			goto		$

ISR_INTERRUPT0
	bcf INTCON, INT0IF
	movlw 0x00
	cpfseq status
	goto STATUS1
	goto STATUS0
END_INT0
	retfie

STATUS1
	clrf status
	bsf T0CON, TMR0ON
	bra END_INT0

STATUS0
	incf status
	bcf T0CON, TMR0ON
	bra END_INT0

INIT
	movlw 0x00
	movwf status
	movlw .0	;59 - (50 DEC)
	movwf milisecond
	movlw .0	;89 - (59 DEC)
	movwf second
	movlw .0	;35 - (23 DEC)
	movwf minute

	call INIT_TIMER0
	call INIT_LCD
	bsf TRISA, RA4

	movlw 0x0e
	movwf ADCON1
	bsf TRISB, RB0

	BSF	INTCON,GIEH		;set the global interrupt enable bits
	BSF INTCON,GIEL		;enable priorities interrupts
	BCF INTCON2,TMR0IP	;set timer0 as low prority
	BSF RCON, IPEN		;enable priorities interrupts
	
	BSF INTCON, INT0IE
	BCF INTCON, INT0IF
	BCF INTCON2, INTEDG0	;falling edge RB0

	return

INIT_LCD
			call		init_lcd			; Khoi dong LCD
			movlw		'L'
			movwf		char
			call		lcd_print_char		; Xuat ra chu 'LCD'
			movlw		'C'
			movwf		char
			call		lcd_print_char
			movlw		'D'
			movwf		char
			call		lcd_print_char
			movlw 0x01
			movwf row
			movlw 0x00
			movwf col
			call lcd_goto_xy
			return

INIT_TIMER0	;10ms
	BCF	INTCON,TMR0IF	;clear Timer0 overflow flag
	BSF	INTCON,TMR0IE	;set interrupt by Timer0 enable

	CLRF	T0CON	;prescale 2:1
	MOVLW	0xEC	;move MSBs 0xEC77 to TMR0H
	MOVWF	TMR0H
	MOVLW	0x77	;move LSBs 0xEC77 to TMR0L
	MOVWF	TMR0L
	
	RETURN


ISR_TMR0
	MOVWF W_temp
	BCF INTCON, TMR0IF
	BCF INTCON, TMR0ON
	MOVLW	0xEC	;move MSBs 0xEC77 to TMR0H
	MOVWF	TMR0H
	MOVLW	0x77	;move LSBs 0xEC77 to TMR0L
	MOVWF	TMR0L
	BSF INTCON, TMR0ON

	call PRINT_LCD
NEXT
	incf milisecond
	movf milisecond, W
	daw
	movwf milisecond

CHECK_MILISECOND
	movlw .0
	cpfseq milisecond
	;retfie
	bra FINISH_CHECK
	call UPDATE_SECOND
CHECK_SECOND
	movlw .96
	cpfseq second
	;retfie
	bra FINISH_CHECK
	call UPDATE_MINUTE
CHECK_MINUTE
	movlw .96
	cpfseq minute
	;retfie
	bra FINISH_CHECK
	call RESTART
FINISH_CHECK
	movf W_temp, W
	retfie

UPDATE_SECOND
	movlw 0x00
	movwf milisecond
	incf second
	movf second, W
	daw
	movwf second
	return

UPDATE_MINUTE
	movlw 0x00
	movwf second
	incf minute
	movf minute, W
	daw
	movwf minute
	return

RESTART
	movlw 0x00
	movwf minute
	return

PRINT_LCD
	call SUPPORT_PRINT_LCD
	movlw 0x01
	movwf row
	movlw 0x00
	movwf col
	call lcd_goto_xy
	return

SUPPORT_PRINT_LCD
	movf minute,W
	movwf decode_var
	;movff minute, decode_var
	swapf decode_var
	call decode_ascii
	call lcd_print_char
	;movff minute, decode_var
	movf minute,W
	movwf decode_var
	call decode_ascii
	call lcd_print_char
	movlw ':'
	movwf char
	call lcd_print_char
	
	movf second,W
	movwf decode_var
	swapf decode_var
	call decode_ascii
	call lcd_print_char
	movf second,W
	movwf decode_var
	call decode_ascii
	call lcd_print_char
	movlw ':'
	movwf char
	call lcd_print_char

	movf milisecond,W
	movwf decode_var
	swapf decode_var
	call decode_ascii
	call lcd_print_char
	
	movf milisecond,W
	movwf decode_var
	call decode_ascii
	call lcd_print_char

	return

decode_ascii ;----- Giai ma 4bit du lieu trong decode_var ra char -----;		
	movlw	0x0f
	andwf	decode_var
	movlw	0x0a
	cpfslt	decode_var
	bra	decode_1	; Lon hon 9
	movlw	0x30	; Nho hon hay bang 9
decode_2	addwf	decode_var
	movff	decode_var,char
	return
decode_1	movlw	0x37
	bra	decode_2


debug		movlw		0x0f
			movwf		ADCON1				; Port digital
			clrf		LATB				; PortB xuat
			bcf			TRISB,RB1
debug1		btg			LATB,RB1			; Phat xung BR1			
			bra			debug1

lcd_print_char;--- trong char
			movlw		.0
			cpfseq		row
			bra			print1
			movlw		.16					; current_row = 0
			cpfseq		col
			bra			print1
			movlw		.1					; current_col = 16
			movwf		row				
			movlw		.0
			movwf		col
			call		lcd_goto_xy			; Chuyen cursor ve 1,0
print1		movlw		.1
			cpfseq		row
			bra			print2
			movlw		.16					; current_row = 1
			cpfseq		col
			bra			print2
			movlw		.0					; current_col = 16
			movwf		row				
			movwf		col
			call		lcd_goto_xy			; Chuyen cursor ve 0,0
print2		call		lcd_write_data		; Xuat data ra LCD
			movlw		.1	
			addwf		row					; Tang row len 1
			return

lcd_write_data ;----- trong char
			bsf			LCD_RS				; Cho phep xuat data
			movff		char,data_4bit		; Xuat 4 bit data cao ra
			swapf		data_4bit
			call		lcd_write_4bits	
			movff		char,data_4bit		; Xuat 4 bit data thap ra
			call		lcd_write_4bits
			return

lcd_clear	movlw		0x01
			movwf		command
			call		lcd_write_cmd		; Xoa man hinh LCD
			call 		delay15ms			; Giu cham 15msec
			movlw		0x00
			movwf		row
			movlw		0x00
			movwf		col
			call		lcd_goto_xy			; Cursor ve toa do 0,0
			return

lcd_goto_xy	;---- Thiet lap cursor toi toa do x.y
			movlw		.20
			cpfslt		col
			bra			xy3					; Qua 19 cot
			movlw		.4
			cpfslt		row
			bra			xy3					; Qua 3 hang
			movlw		.2
			cpfslt		row
			bra			xy1					
			movlw		0x40				; Hang nho hon 2
			mulwf		row
			movff		PRODL,addr
			movf		col,w
			addwf		addr
xy2			bsf			addr,7				; Thiet lap bit7 cua lenh
			movff		addr,command
			call		lcd_write_cmd		; Phat lenh
xy3			return
xy1			movlw		0x40				; Hang lon hon hay bang 2
			mulwf		row
			movff		PRODL,addr
			movf		col,w
			addwf		addr
			bsf			addr,4
			bsf			addr,2
			bra			xy2		

init_lcd	call		init_portD
			movlw		.0
			movwf		col
			movwf		row
			call 		delay15ms			; Giu cham 15msec
			bcf			LCD_RS				; Cho phep gui lenh
			movlw		0x03
			movwf		data_4bit
			call		lcd_write_4bits		; 
			call		delay5ms
			call		lcd_write_4bits		;
			call		delay100us
			call		lcd_write_4bits		;
			movlw		0x02
			movwf		data_4bit
			call		lcd_write_4bits		; Lenh bat dau 4bit
			movlw		0x28
			movwf		command
			call		lcd_write_cmd		; Hien thi 2 hang, ma tran 5x7
			movlw		0x0c
			movwf		command
			call		lcd_write_cmd		; Display = on
			movlw		0x06
			movwf		command
			call		lcd_write_cmd		; Set mode : dich phai tang dan
			call		lcd_clear
			return

init_portD	clrf		LATD
			bcf			LCD_D4_DIR			; Data Output
			bcf			LCD_D5_DIR
			bcf			LCD_D6_DIR
			bcf			LCD_D7_DIR			
			bcf			LCD_E_DIR			; Control output
			bcf			LCD_RW_DIR
			bcf			LCD_RS_DIR
			bcf			LCD_ON_DIR
			bsf			LCD_ON				; Cap nguon cho LCD
			return

lcd_write_cmd ;----- trong command
			;call		lcd_wait_busy		; LCD san sang
			call delay5ms					;MODIFIED
			bcf			LCD_RS				; Cho phep lenh
			movff		command,data_4bit		
			swapf		data_4bit
			call		lcd_write_4bits		; Xuat 4bit lenh cao 
			movff		command,data_4bit
			call		lcd_write_4bits		; Xuat 4bit lenh thap
			return

lcd_wait_busy;----- Cho LCD san sang
			bsf			LCD_D7_DIR			; RD3 input
			bsf			LCD_D6_DIR			; RD2 input
			bsf			LCD_D5_DIR			; RD1 input
			bsf			LCD_D4_DIR			; RD0 input
			bcf			LCD_RS				; Cho phep lenh
			bsf			LCD_RW				; Cho phep doc
			call		delay1us			; Cho xong
			bsf			LCD_E				; Cho phep LCD hoat dong
			call		delay1us			; Giu cham phan cung
wait1		btfsc		LCD_D,3				; Kiem tra LCD xong chua ?
			bra			wait1
			bcf			LCD_E				; Xong, cam LCD
			bcf			LCD_D7_DIR			; RD3 output
			bcf			LCD_D6_DIR			; RD2 output
			bcf			LCD_D5_DIR			; RD1 output
			bcf			LCD_D4_DIR			; RD0 output
			return

lcd_write_4bits ;----- Lay 4bit thap trong data_4bit
			bcf			LCD_RW				; Cho phep viet
			bsf			LCD_E				; Cho phep LCD hoat dong
			call		write_4bit_to_portD	; Xuat 4bit data
			call		delay10us			; Giu cham 10 usec
			bcf			LCD_E				; CAM LCD hoat dong
			call		delay10us			; Giu cham 10 usec
			return

;---Kiem tra 4bit thap trong data_4bit va thiet lap 4bit cao trong portD
write_4bit_to_portD
			btfsc		data_4bit,0
			bra			set_bit_0
			bcf			LCD_D4
tt1			btfsc		data_4bit,1
			bra			set_bit_1
			bcf			LCD_D5
tt2			btfsc		data_4bit,2
			bra			set_bit_2
			bcf			LCD_D6
tt3			btfsc		data_4bit,3
			bra			set_bit_3
			bcf			LCD_D7
tt4			return
set_bit_0	bsf			LCD_D4
			bra			tt1
set_bit_1	bsf			LCD_D5
			bra			tt2
set_bit_2	bsf			LCD_D6
			bra			tt3
set_bit_3	bsf			LCD_D7
			bra			tt4

delay15ms	movlw		.15
			movwf		delay_1
dl_15ms		call		delay1ms
			decfsz		delay_1
			bra			dl_15ms
			return

delay5ms	movlw		.5
			movwf		delay_1
dl_5ms		call		delay1ms
			decfsz		delay_1
			bra			dl_5ms
			return


delay1ms	movlw		.248
			movwf		delay
dl_1ms		call		delay1us
			decfsz		delay
			bra			dl_1ms
			return

delay100us	movlw		.98
			movwf		delay
dl100us		call		delay1us
			decfsz		delay
			bra			dl100us
			return

delay10us	movlw		.9
			movwf		delay
dl10us		call		delay1us
			decfsz		delay
			bra			dl10us
			return


delay1us	nop			; Tan so 4MHz
			nop			; 1 chu ky
			return		; 2 chu ky


isr_high	retfie

isr_low		retfie

			end