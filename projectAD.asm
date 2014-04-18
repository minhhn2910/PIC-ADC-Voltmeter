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
			goto		start
			org			08h
			goto		isr_high
			org			18h
			goto		isr_low

			udata
;--------------bien cho convert--------------

;bien dung trong phep chia
a_var res 1
b_var res 1		;perform a/b = result + mod   ; fixed b = 10
result res 1
mod res 1
out1 res 1  ; phan nguyen
out2 res 1	; 2 so phan thap phan 
out3 res 1 ; 2 so cuoi
in_high res 1
in_low res 1
; => ket qua 1.2345
; ket qua se co do tin cay 5*10^-3
; buoc nhay don vi = 5/1023
temp1 res 1 
temp2 res 1
temp3 res 1
; bien tam dung trong luc cong


; ------------- bien tam -----------
high_temp res 1
low_temp res 1

;-----bien lcd
delay		res			1
delay_1		res			1
command		res			1
data_4bit	res			1
row			res			1
col			res			1
addr		res			1
char		res			1
dl_1sec		res			1
dem_BCD		res			1
decode_var	res			1


;---------------------CODE-------------
PRG			code
start	

			call		init_lcd			; Khoi dong LCD
			call		init_AD				; Khoi dong A/D 

main			
			bsf	ADCON0,GO		;start conversion
			btfsc	ADCON0,GO
			bra	$-2
			movff	ADRESH,in_high
			movff	ADRESL,in_low
;check xem input co thay doi k, neu k thay doi thi quay ve main
			movf high_temp,w 
			cpfseq in_high
			bra main_continue
			movf low_temp,w
			cpfseq in_low
			bra main_continue
			bra main
main_continue
			movff in_high,high_temp
			movff in_low,low_temp

			call init_convert		; reset convert
			call convert
			call round_number		

			clrf		col					; Tro ve toa do 0,0
			clrf		row
			call		lcd_goto_xy
;xuat gia tri ADC
			movlw		'A'
			movwf		char
			call		lcd_print_char
			movlw		'D'
			movwf		char
			call		lcd_print_char
			movlw		'C'
			movwf		char
			call		lcd_print_char
			movlw		'='
			movwf		char
			call		lcd_print_char
			movff		in_high,decode_var 
			call		decode_ascii		; Giai ma binary sang ma ASCII
			call		lcd_print_char		; 
			movff		in_low,decode_var 
			call		decode_ascii		; Giai ma binary sang ma ASCII
			call		lcd_print_char		; 

; di chuyen den hang thu 2	
			clrf		col
			movlw		.1
			movwf 		row
			call		lcd_goto_xy

			movlw		'V'
			movwf		char
			call		lcd_print_char
			movlw		'o'
			movwf		char
			call		lcd_print_char
			movlw		'l'
			movwf		char
			call		lcd_print_char
			movlw		't'
			movwf		char
			call		lcd_print_char
			movlw		'='
			movwf		char
			call		lcd_print_char		; Xuat ra chu 'LCD'
			
			;out1
			movff		out1,decode_var	; Xuat du lieu ra LCD
			call		decode_ascii		; Giai ma binary sang ma ASCII
			call		lcd_print_char		; Xuat ra LCD		
			movlw		'.'
			movwf		char
			call		lcd_print_char		
			
			;out2
			movff		out2,a_var	; chuan bi cho phep chia
			call		devide8
			movff		result,decode_var 
			call		decode_ascii		; Giai ma binary sang ma ASCII
			call		lcd_print_char		; 
			movff		mod,decode_var	; 
			call		decode_ascii		; Giai ma binary sang ma ASCII
			call		lcd_print_char		;
			
	;		call		delay1s				; giu cham 1sec

			bra			main

	;-------- Test  --------;
		
debug		movlw		0x0f
			movwf		ADCON1				; Port digital
			clrf		LATB				; PortB xuat
			bcf			TRISB,RB1
debug1		btg			LATB,RB1			; Phat xung BR1			
			bra			debug1

			
	;-----------------Init-------------------
init_convert
	; buoc nhay = 0.004888
	clrf out1
	clrf out2
	clrf out3
return	

init_AD
	movlw	b'10001110'		;RA0 = analog input
	movwf	ADCON1
	
	movlw		b'11000001'			; Chon RC osc, AN0 input
	movwf		ADCON0				; Cho phep A_D
	
	movlw	0x01
	movwf	ADCON2

	bsf		ADCON2,7
	
	call		delay100us			; Giu cham > 15 cycles
	
return

	;----------------Convert---------------		
sum_product			; cong temp vao out sau khi nhan xong;
	movf temp3,WREG
	addwf out3
	call adjust
	movf temp2,WREG
	addwf out2
	call adjust	
	movf temp1,WREG
	addwf out1
return

adjust ; dieu chinh lai cac bien out < 100
	movlw .99
	cpfsgt out3
	bra continue3
	movlw .100
	subwf out3
	incf out2
continue3
	movlw .99
	cpfsgt out2
	bra continue2
	movlw .100
	subwf out2
	incf out1
continue2	
return

convert ; ham tinh toan chinh : xet tung bit cua 2 thanh ghi, cong voi gia tri tuong ung
	btfss in_high,1
	bra con_9
	movlw .9	; bit so 9 duoc set
	call table
	call sum_product
con_9
	btfss in_high,0
	bra con_8
	movlw .8	; bit so 8 duoc set
	call table
	call sum_product
con_8
	btfss in_low,7
	bra con_7
	movlw .7	; bit so 7 duoc set
	call table
	call sum_product
con_7
	btfss in_low,6
	bra con_6
	movlw .6	; bit so 6 duoc set
	call table
	call sum_product
con_6
	btfss in_low,5
	bra con_5
	movlw .5	; bit so 5 duoc set
	call table
	call sum_product
con_5
	btfss in_low,4
	bra con_4
	movlw .4	; bit so 4 duoc set
	call table
	call sum_product
con_4
	btfss in_low,3
	bra con_3
	movlw .3	; bit so 3 duoc set
	call table
	call sum_product
con_3
	btfss in_low,2
	bra con_2
	movlw .2	; bit so 2 duoc set
	call table
	call sum_product
con_2
	btfss in_low,1
	bra con_1
	movlw .1	; bit so 1 duoc set
	call table
	call sum_product
con_1
	btfss in_low,0
	bra con_0
	movlw .0 ; bit so 0 duoc set
	call table
	call sum_product
con_0
	return	
	
;-------------------------------ham chia so 8 bit---------------------------
devide8	;a_var = b_var * result + mod
; init
	movlw .10
	movwf b_var
	clrf result
;end init
devide8_next
	movf a_var,WREG
	cpfsgt b_var
	bra continue_devide	;a_var >= b
	movff a_var,mod	;finish (a<b)
	return
continue_devide
	movf b_var,WREG			
	subwf a_var		;a = a-b
	incf result
	bra devide8_next

round_number
	movff		out3,a_var	; chuan bi cho phep chia
	call		devide8
	movlw 		.4
	cpfsgt		result
	bra finish_round
	incf out2
	call adjust
finish_round
return
	




org 0x500

table 
	; xu ly w + pc
addwf WREG,0	; wreg * 2
addwf PCL,1
	bra bit0
	bra bit1
	bra bit2
	bra bit3
	bra bit4
	bra bit5
	bra bit6
	bra bit7
	bra bit8
	bra bit9	
	bra bit8
	bra bit9	
	bra bit8
	bra bit9	
	bra bit8
	bra bit9	
;	bat dau nap gia tri cho temp
;bit = 0	0.0049
bit0
	clrf temp1
	clrf temp2
	movlw .49
	movwf temp3
	return
;bit = 1	0.0098
bit1
	clrf temp1
	clrf temp2
	movlw .98
	movwf temp3
	return
;bit = 2	0.0196
bit2
	clrf temp1
	movlw .01
	movwf temp2
	movlw .96
	movwf temp3
	return
;bit = 3	0.0391
bit3
	clrf temp1
	movlw .03
	movwf temp2
	movlw .91
	movwf temp3
	return
;bit = 4	0.0782
bit4
	clrf temp1
	movlw .07
	movwf temp2
	movlw .82
	movwf temp3
	return
;bit = 5	0.1564
bit5
	clrf temp1
	movlw .15
	movwf temp2
	movlw .64
	movwf temp3
	return
;bit = 6	0.3128
bit6
	clrf temp1
	movlw .31
	movwf temp2
	movlw .28
	movwf temp3
	return
;bit = 7	0.6256
bit7
	clrf temp1
	movlw .62
	movwf temp2
	movlw .56
	movwf temp3
	return
;bit = 8	1.2512
bit8
	movlw .1
	movwf temp1
	movlw .25
	movwf temp2
	movlw .12
	movwf temp3
	return
;bit = 9	2.5024
bit9
	movlw .2
	movwf temp1
	movlw .50
	movwf temp2
	movlw .24
	movwf temp3
	return








;-----------------------------------------------------------------
	;--------- Chuong trinh LCD  ----------;

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