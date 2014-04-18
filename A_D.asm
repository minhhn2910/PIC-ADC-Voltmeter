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
	

			udata
;------- Bien cho LCD ---------;
delay		res			1
delay_1		res			1
command		res			1
data_4bit	res			1
row			res			1
col			res			1
addr		res			1
char		res			1
;------- Bien cho A_D ---------;
;decode_var	res			1
;A_D_var	res				1
in_high equ 0x50
in_low equ 0x51
out1 equ 0x52  ; phan nguyen
out2 equ 0x53	; 2 so phan thap phan 
out3 equ 0x54  ; 2 so tiep theo
out4 equ 0x55 ; 2 so cuoi
; => ket qua 1.234567
; ket qua se co do tin cay 10^-3
; buoc nhay don vi = 5/1023
temp1 equ 0x56 
temp2 equ 0x57
temp3 equ 0x58
temp4 equ 0x59
; bien tam dung trong luc cong

PRG			code
start		call		init_lcd			; Khoi dong LCD
			rcall		init_A_D			; Khoi dong A_D
			rcall		A_D_isr				; Doc bien doi A_D
			movff		A_D_var,decode_var	; Xuat du lieu ra LCD
			swapf		decode_var			; Lay byte cao
			rcall		decode_ascii		; Giai ma binary sang ma ASCII
			call		lcd_print_char		; Xuat byte1 ra LCD
			movff		A_D_var,decode_var	; Lay byte thap
			rcall		decode_ascii		; Giai ma binary sang ma ASCII
			call		lcd_print_char		; Xuat byte2 ra LCD
call debug
			bra			$

;------ Dung cho DEBUG -------;

debug		movlw		0x0f
			movwf		ADCON1						; Port digital
			clrf		LATB						; PortB xuat
			bcf			TRISB,RB1
debug1		btg			LATB,RB1					; Phat xung BR1
			bra			debug1

;------ Chuong trinh cua A_D -------;

init_A_D	movlw		b'00000100'			; RA0,RA1,RA4 analog input
			movwf		ADCON1
			movlw		b'11000001'			; Chon RC osc, AN0 input
			movwf		ADCON0				; Cho phep A_D
			movlw		0x01
			movwf		ADCON2
			call		delay100us			; Giu cham > 15 cycles
			bsf			ADCON0,GO			; Bat dau bien doi A_D
			return

A_D_isr		bsf			ADCON0,GO			; Bat dau bien doi
			btfsc		ADCON0,GO
			bra			$ - 2
			movff		ADRESH,A_D_var
			return

;------- Chuong trinh cua LCD --------;

decode_ascii;---- chuyen 4bit thap trong decode_var thanh 8bit ASCII trong char ----;
			movlw		0x0f
			andwf		decode_var
			movlw		0x0a
			cpfslt		decode_var
			bra			decode_1			; Lon hon 9
			movlw		0x30				; Nho hon hay bang 9
decode_2	addwf		decode_var
			movff		decode_var,char
			return
decode_1	movlw		0x37
			bra			decode_2

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
			rcall		lcd_goto_xy			; Chuyen cursor ve 1,0
print1		movlw		.1
			cpfseq		row
			bra			print2
			movlw		.16					; current_row = 1
			cpfseq		col
			bra			print2
			movlw		.0					; current_col = 16
			movwf		row				
			movwf		col					; Xoa LCD
			rcall		lcd_clear
print2		rcall		lcd_write_data		; Xuat data ra LCD
			movlw		.1	
			addwf		col					; Tang col len 1
			return

lcd_write_data ;----- trong char
			bsf			LCD_RS				; Cho phep xuat data
			movff		char,data_4bit		; Xuat 4 bit data cao ra
			swapf		data_4bit
			rcall		lcd_write_4bits	
			movff		char,data_4bit		; Xuat 4 bit data thap ra
			rcall		lcd_write_4bits
			return

lcd_clear	movlw		0x01
			movwf		command
			rcall		lcd_write_cmd		; Xoa man hinh LCD
			call 		delay5ms			; Giu cham 5msec
			movlw		.0
			movwf		row
			movlw		.0
			movwf		col
			rcall		lcd_goto_xy			; Cursor ve toa do 0,0
			return

lcd_goto_xy	;---- Thiet lap cursor toi toa do x.y
			movlw		.0
			movwf		addr
			movlw		.20
			cpfslt		col
			bra			xy3					; Qua 16 cot
			movlw		.4
			cpfslt		row
			bra			xy3					; Qua 2 hang
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
			rcall		lcd_write_cmd		; Phat lenh
			call 		delay5ms			; Giu cham 5msec
xy3			return
xy1			movlw		0x40				; Hang lon hon hay bang 2
			mulwf		row
			movff		PRODL,addr
			movf		col,w
			addwf		addr
			bsf			addr,4
			bsf			addr,2
			bra			xy2		

init_lcd	rcall		init_portD
			movlw		.0
			movwf		col
			movwf		row
			call 		delay15ms			; Giu cham 15msec
			bcf			LCD_RS				; Cho phep gui lenh
			movlw		0x03
			movwf		data_4bit
			rcall		lcd_write_4bits		; 
			call		delay5ms
			rcall		lcd_write_4bits		;
			call		delay100us
			rcall		lcd_write_4bits		;
			movlw		0x02
			movwf		data_4bit
			rcall		lcd_write_4bits		; Lenh bat dau 4bit
			movlw		0x28
			movwf		command
			rcall		lcd_write_cmd		; Hien thi 2 hang, ma tran 5x7
			movlw		0x0c
			movwf		command
			rcall		lcd_write_cmd		; Display = on
			movlw		0x06
			movwf		command
			rcall		lcd_write_cmd		; Set mode : dich phai tang dan
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
		;	rcall		lcd_wait_busy		; LCD san sang
			bcf			LCD_RS				; Cho phep lenh
			movff		command,data_4bit		
			swapf		data_4bit
			rcall		lcd_write_4bits		; Xuat 4bit lenh cao 
			movff		command,data_4bit
			rcall		lcd_write_4bits		; Xuat 4bit lenh thap
			return

lcd_wait_busy;----- Cho LCD san sang
			bsf			LCD_D7_DIR			; RD3 input
			bsf			LCD_D6_DIR			; RD2 input
			bsf			LCD_D5_DIR			; RD1 input
			bsf			LCD_D4_DIR			; RD0 input
			bcf			LCD_RS				; Cho phep lenh
			bsf			LCD_RW				; Cho phep doc
			rcall		delay1us			; Cho xong
			bsf			LCD_E				; Cho phep LCD hoat dong
			rcall		delay1us			; Giu cham phan cung
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
			rcall		write_4bit_to_portD	; Xuat 4bit data
			rcall		delay10us			; Giu cham 10 usec
			bcf			LCD_E				; CAM LCD hoat dong
			rcall		delay10us			; Giu cham 10 usec
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
dl_15ms		rcall		delay1ms
			decfsz		delay_1
			bra			dl_15ms
			return

delay5ms	movlw		.5
			movwf		delay_1
dl_5ms		rcall		delay1ms
			decfsz		delay_1
			bra			dl_5ms
			return


delay1ms	movlw		.248
			movwf		delay
dl_1ms		rcall		delay1us
			decfsz		delay
			bra			dl_1ms
			return

delay100us	movlw		.98
			movwf		delay
dl100us		rcall		delay1us
			decfsz		delay
			bra			dl100us
			return

delay10us	movlw		.9
			movwf		delay
dl10us		rcall		delay1us
			decfsz		delay
			bra			dl10us
			return

delay1us	nop			; Tan so 4MHz
			nop			; 1 chu ky
			return		; 2 chu ky


			end
