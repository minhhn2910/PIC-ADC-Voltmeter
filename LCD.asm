			#include	p18f4520.inc
			radix		dec

			#define		LCD_D 		PORTD
			#define		LCD_D4 		LATD,RD0
			#define		LCD_D5 		LATD,RD1
			#define		LCD_D6 		LATD,RD2
			#define		LCD_D7 		LATD,RD3
			
		;	#define		LCD_D4_DIR 	TRISD,RD0
		;	#define		LCD_D5_DIR 	TRISD,RD1
		;	#define		LCD_D6_DIR 	TRISD,RD2
		;	#define		LCD_D7_DIR 	TRISD,RD3

			#define		LCD_E 		LATD,RD6
			#define		LCD_RW 		LATD,RD5
			#define		LCD_RS 		LATD,RD4

		;	#define		LCD_E_DIR 	TRISD,RD6
		;	#define		LCD_RW_DIR 	TRISD,RD5
		;	#define		LCD_RS_DIR 	TRISD,RD4

		;	#define		LCD_INS 	0
		;	#define		LCD_DATA 	1

			code		0
			goto		start
			org			08h
			goto		isr_high
			org			18h
			goto		isr_low

			udata
delay		res			1
delay_1		res			1
temp		res			1
data_4bit	res			1
x			res			1
y			res			1
addr		res			1
;current_row	res			1
;current_col	res			1
char		res			1

PRG			code
start		call		init_lcd
			
finish		
			goto		$

lcd_print_char;--- trong char
			movlw		.0
			cpfseq		x
			bra			print1
			movlw		.16
			cpfseq		y
			bra			print1
			movlw		.1
			movwf		x
			movlw		.0
			movwf		y
			call		lcd_goto_xy
print1		movlw		.1
			cpfseq		x
			bra			print2
			movlw		.16
			cpfseq		y
			bra			print2
			movlw		.0
			movwf		x
			movwf		y
			call		lcd_goto_xy
print2		call		lcd_write_data
			movlw		.1
			addwf		x
			return

lcd_write_data ;----- trong temp
			bsf			LCD_RS
			movff		temp,data_4bit
			swapf		data_4bit
			call		lcd_write_4bits	
			movff		temp,data_4bit
			call		lcd_write_4bits
			return

lcd_clear	movlw		0x01
			movwf		temp
			call		lcd_write_cmd
			movlw		0x00
			movwf		x
			movwf		y
			call		lcd_goto_xy
			return

lcd_goto_xy	;---- Toi toa do x.y
			movlw		.20
			cpfslt		y
			bra			kt
			movlw		.4
			cpfslt		x
			bra			kt
			movlw		.2
			cpfslt		x
			bra			xy1
			movlw		0x40
			mulwf		x
			addwf		y
			movwf		addr
xy2			bsf			addr,7
			movff		addr,temp
			call		lcd_write_cmd
			;movff		x,current_row
			;movff		y,current_col
kt			return
xy1			movlw		0x40
			mulwf		x
			addwf		y
			movwf		addr
			bsf			addr,4
			bsf			addr,2
			bra			xy2		

init_lcd	call 		delay15ms
			clrf		LATD
			bcf			LCD_RS
			movlw		0x03
			movwf		data_4bit
			call		lcd_write_4bits
			call		delay5ms
			call		lcd_write_4bits
			call		delay1ms
			call		lcd_write_4bits
			movlw		0x02
			movwf		data_4bit
			call		lcd_write_4bits
			movlw		0x28
			movwf		temp
			call		lcd_write_cmd
			movlw		0x0c
			movwf		temp
			call		lcd_write_cmd
			movlw		0x06
			movwf		temp
			call		lcd_write_cmd
			return

lcd_write_cmd ;----- trong temp
			bcf			LCD_RS
			movff		temp,data_4bit
			swapf		data_4bit
			call		lcd_write_4bits	
			movff		temp,data_4bit
			call		lcd_write_4bits
			return

lcd_write_4bits ;----- trong data_4bit
			call		write_4bit_to_portD
			bsf			LCD_E
			bcf			LCD_RW
			call		delay1ms
			bsf			LCD_RW
			bcf			LCD_E
			call		delay1ms
			return

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

delay5ms	movlw		.5
			movwf		delay_1
dl_3		call		delay1ms
			decfsz		delay_1
			bra			dl_3
			return

delay15ms	movlw		.15
			movwf		delay_1
dl_2		call		delay1ms
			decfsz		delay_1
			bra			dl_2
			return

delay1ms	clrf		delay
dl_1		nop
			decfsz		delay
			bra			dl_1
			return


isr_high	retfie

isr_low		retfie

			end
