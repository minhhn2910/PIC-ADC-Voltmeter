processor = p18f4520
#include P18F4520.inc
udata
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
code 0x00
goto main
org 0x100
main
	call init
	call convert
	bra $
init
	; buoc nhay = 0.004888
	clrf out1
	clrf out2
	clrf out3
	clrf out4
	;nap gia tri test 0x136 => out = 1.515152
	movlw 0x01 
	movwf in_high
	movlw 0x36
	movwf in_low
return

sum_product			; cong temp vao out sau khi nhan xong;
	movf temp4,WREG
	addwf out4
	call adjust
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
	cpfsgt out4
	bra continue4
	movlw .100
	subwf out4
	incf out3
continue4
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
;bit = 0	0.004888
bit0
	clrf temp1
	clrf temp2
	movlw .48
	movwf temp3
	movlw .88
	movwf temp4
	return
;bit = 1	0.009775
bit1
	clrf temp1
	clrf temp2
	movlw .97
	movwf temp3
	movlw .75
	movwf temp4
	return
;bit = 2	0.019550
bit2
	clrf temp1
	movlw .01
	movwf temp2
	movlw .95
	movwf temp3
	movlw .50
	movwf temp4
	return
;bit = 3	0.039101
bit3
	clrf temp1
	movlw .03
	movwf temp2
	movlw .91
	movwf temp3
	movlw .01
	movwf temp4
	return
;bit = 4	0.078201
bit4
	clrf temp1
	movlw .07
	movwf temp2
	movlw .82
	movwf temp3
	movlw .01
	movwf temp4
	return
;bit = 5	0.156403
bit5
	clrf temp1
	movlw .15
	movwf temp2
	movlw .64
	movwf temp3
	movlw .03
	movwf temp4
	return
;bit = 6	0.312805
bit6
	clrf temp1
	movlw .31
	movwf temp2
	movlw .28
	movwf temp3
	movlw .05
	movwf temp4
	return
;bit = 7	0.625611
bit7
	clrf temp1
	movlw .62
	movwf temp2
	movlw .56
	movwf temp3
	movlw .11
	movwf temp4
	return
;bit = 8	1.251222
bit8
	movlw .1
	movwf temp1
	movlw .25
	movwf temp2
	movlw .12
	movwf temp3
	movlw .22
	movwf temp4
	return
;bit = 9	2.502444
bit9
	movlw .2
	movwf temp1
	movlw .50
	movwf temp2
	movlw .24
	movwf temp3
	movlw .44
	movwf temp4
	return

end


