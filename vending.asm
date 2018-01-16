;-----------------------------------------------------------------------------
;	--------------------------------
;         Vending Machine Controller
;	--------------------------------
                LIST	p=16F628, R=DEC		;tell assembler what chip we are using
                include "P16F628.inc"		;include the defaults for the chip
                ERRORLEVEL	0,	-302	;suppress bank selection messages
                __CONFIG     _CP_OFF & _LVP_OFF & _BODEN_OFF  & _PWRTE_ON & _WDT_OFF & _HS_OSC  & _MCLRE_OFF
;
; Var definitions...
;
                cblock	0x20                    ;start of general purpose registers
                        count1			;used in delay routine
			                  counta			;used in delay routine
			                  countb			;used in delay routine
                        templcd			;temp store for 4 bit mode
			                  templcd2
                        Xmit_Byte        	;holds byte to xmit
            	        	Rcv_Byte        	;holds received byte
            		        Bit_Cntr        	;bit counter for RS232
            		        Delay_Count        	;delay loop counter
                        tmp1			;temporary storage
			                  tmp2
                        tmp3
                        temp_w
                        temp_s
                        tempchar
                        readstart               ;buffer readback
                        bufhead                 ;buffer write
                        bufcount                ;buffer position counter
                        recchar                 ;holds read char
                        NumL			;Binary inputs for decimal convert routine
	        	            NumH
        		            TenK			;Decimal outputs from convert routine
	        	            Thou
        		            Hund
	        	            Tens
        		            Ones
                        endc

                        constant buf_start=0x40     ; Buffer starts at adress 0x40
                        constant buf_end=0x64       ; And ends at adress 0x40 -> 36 characters!

;############################ Org #############################################
		                    ORG	0x00           ; start a program memory location zero
		                    Goto	Start	; jump over subs

                ORG     0x04     ;Interrupt vector
                GOTO    isr

; ----------------------------- Subroutines ------------------------------------
;--------------------------------------------------------------------------------
HEX_Table  	ADDWF   PCL       , f
            	RETLW   0x30
            	RETLW   0x31
            	RETLW   0x32
            	RETLW   0x33
            	RETLW   0x34
            	RETLW   0x35
            	RETLW   0x36
            	RETLW   0x37
            	RETLW   0x38
            	RETLW   0x39
            	RETLW   0x41
            	RETLW   0x42
            	RETLW   0x43
            	RETLW   0x44
            	RETLW   0x45
            	RETLW   0x46

Text		addwf	PCL, f
		retlw	'I'
		retlw	'n'
		retlw	's'
		retlw	'e'
		retlw	'r'
		retlw	't'
		retlw	' '
		retlw	'C'
		retlw	'o'
		retlw	'i'
		retlw	'n'
                retlw	's'
		retlw	0x00

Text2		addwf	PCL, f
		retlw	'I'
		retlw	'n'
		retlw	'i'
		retlw	't'
		retlw	'i'
		retlw	'a'
		retlw	'l'
		retlw	'i'
		retlw	'z'
		retlw	'i'
		retlw	'n'
                retlw	'g'
                retlw	'.'
                retlw	'.'
                retlw	'.'
		retlw	0x00


;  Coin changer routines
; ------------------------------------------------
Rset		movlw	b'00001000'	;address of changer 08h
		call    address
                movlw   0x08            ;checksum (same as command)
                call    send
                return

ack             movlw   0x00        ; 00h for ack
                call    send
                return
          
Poll            movlw   0x0B           ; add 0bh to w for poll
                call    address
                movlw   0x0B            ;check (same as command)
                call    send
                return

Setup           movlw   0x09           ; add 09h to w for setup
                call    address
                movlw   0x09            ;check (same as command)
                call    send
                return

Tube            movlw   0x0A           ; add 0Ah to w for tube status
                call    address
                movlw   0x0A            ;check
                call    send
                return

Cointype        movlw   0x0C           ; add 0Ch to w for coin type
                call    address
                movlw   0xFF
                call    send
                movlw   0xFF
                call    send
                movlw   0xFF
                call    send
                movlw   0xFF
                call    send
                movlw   0x08
                call    send
                return

Dispense        movlw   0x0D          ; add 0dh to w for dispense
                call    address
                movlw   0x14           ;select coin type 3 (10c) and quantity 1
                call    send
                movlw   0x21       ;checksum
                call    send
                return

Expan           movlw   0x0F           ; add 0Fh to w for Expansion Command
                call    send
                return

address         BSF	STATUS,RP0
		BSF	TXSTA,TX9D	;set 10th high for address mode
		BCF	STATUS,RP0
                call    send
                BSF	STATUS,RP0
		BCF	TXSTA,TX9D	;set 10th low for data mode
		BCF	STATUS,RP0
                return

;  LCD Routines
; --------------------------------------------
LCD_Init	call 	LCD_Busy		;wait for LCD to settle
		movlw	0x20			;Set 4 bit mode
		call	LCD_Cmd
		movlw	0x28			;Set display shift
		call	LCD_Cmd
		movlw	0x06			;Set display character mode
		call	LCD_Cmd
		movlw	0x0c			;Set display on/off and cursor command
		call	LCD_Cmd			;Set cursor off
		call	LCD_Clr			;clear display
		retlw	0x00

LCD_Cmd		movwf	templcd
		swapf	templcd,	w	;send upper nibble
		andlw	0x0f			;clear upper 4 bits of W
		movwf	PORTA
		bcf	PORTA, 0x04             ;RS line to 1
		call	Pulse_e			;Pulse the E line high
		movf	templcd,	w	;send lower nibble
		andlw	0x0f			;clear upper 4 bits of W
		movwf	PORTA
		bcf	PORTA, 0x04             ;RS line to 1
		call	Pulse_e			;Pulse the E line high
		call 	LCD_Busy
		retlw	0x00

LCD_CharD	addlw	0x30			;add 0x30 to convert to ASCII
LCD_Char	movwf	templcd
		swapf	templcd,	w	;send upper nibble
		andlw	0x0f			;clear upper 4 bits of W
		movwf	PORTA
		bsf	PORTA, 0x04             ;RS line to 1
		call	Pulse_e			;Pulse the E line high
		movf	templcd,	w	;send lower nibble
		andlw	0x0f			;clear upper 4 bits of W
		movwf	PORTA
		bsf	PORTA, 0x04             ;RS line to 1
		call	Pulse_e			;Pulse the E line high
		call 	LCD_Busy
		retlw	0x00

LCD_Line1	movlw	0x80			;move to 1st row, first column
		call	LCD_Cmd
		retlw	0x00

LCD_Line2	movlw	0xc0			;move to 2nd row, first column
		call	LCD_Cmd
		retlw	0x00

LCD_Line1W	addlw	0x80			;move to 1st row, column W
		call	LCD_Cmd
		retlw	0x00

LCD_Line2W	addlw	0xc0			;move to 2nd row, column W
		call	LCD_Cmd
		retlw	0x00

LCD_CurOn	movlw	0x0d			;Set display on/off and cursor command
		call	LCD_Cmd
		retlw	0x00

LCD_CurOff	movlw	0x0c			;Set display on/off and cursor command
		call	LCD_Cmd
		retlw	0x00

LCD_Clr		movlw	0x01			;Clear display
		call	LCD_Cmd
		retlw	0x00

Pulse_e		bsf     TRISB, 0x07               ;**LCD_PORT, LCD_E
		nop
                nop
                bcf     TRISB, 0x07            ;**LCD_PORT, LCD_E
		retlw	0x00

LCD_Busy        bsf	STATUS,	RP0		;set bank 1
		movlw	0x0f			;set Port for input
		movwf	TRISA
		bcf	STATUS,	RP0		;set bank 0
		bcf	PORTA, 0x04                 ;set LCD for command mode
		bsf     TRISB,   0x06                   ;**LCD_PORT, LCD_RW	;setup to read busy flag
		nop                             ;**added wait for tAS (140ns min)
                bsf     TRISB, 0x07              ;**  LCD_PORT, LCD_E
		nop                             ;**added wait for tDA
                swapf	PORTA, w		;read upper nibble (busy flag)
		bcf	TRISB, 0x07            ;**LCD_PORT, LCD_E
		nop                            ;**wait for tEL (500ns min)
                nop
                movwf	templcd2
		bsf	TRISB, 0x07		;**LCD_PORT, LCD_E ;dummy read of lower nibble
		nop                             ;wait for tDA
                nop
                bcf	TRISB, 0x07             ;**LCD_PORT, LCD_E
		btfsc	templcd2, 7		;check busy flag, high = busy
		goto	LCD_Busy		;if busy check again
		bcf     TRISB,  0x06            ;**LCD_PORT, LCD_RW
		bsf	STATUS,	RP0		;set bank 1
		movlw	0x00			;set Port for output
		movwf	TRISA
		bcf	STATUS,	RP0		;set bank 0
		return

; BIT BANG RS232 Routines
; ------------------------------------------------------------------
SER_INIT        BSF     PORTB, 0x04             ;set SER_OUT high
                RETURN

XMIT_RS232      MOVWF   Xmit_Byte             ;move W to Xmit_Byte
                MOVLW   0x08                  ;set 8 bits out
                MOVWF   Bit_Cntr
                BCF     PORTB, 0x04
                CALL    Bit_Delay
Ser_Loop        RRF     Xmit_Byte , f         ;send one bit
                BTFSS   STATUS    , C
                BCF     PORTB, 0x04
                BTFSC   STATUS    , C
                BSF     PORTB, 0x04
                CALL    Bit_Delay
                DECFSZ  Bit_Cntr  , f         ;test if all done
                GOTO    Ser_Loop
                BSF     PORTB, 0x04
                CALL    Bit_Delay
                RETURN

Rcv_RS232       BTFSC   PORTB, 0x05         ;wait for start bit
                GOTO    Rcv_RS232
                CALL    Start_Delay	          ;do half bit time delay
                BTFSC   PORTB, 0x05         ;check still in start bit
                GOTO    Rcv_RS232
                MOVLW   0x08                  ;set up to read 8 bits
                MOVWF   Bit_Cntr
                CLRF    Rcv_Byte
Next_RcvBit     CALL    Bit_Delay
                BTFSS   PORTB, 0x05
                BCF     STATUS    , C
                BTFSC   PORTB, 0x05
                BSF     STATUS    , C
                RRF     Rcv_Byte  , f
                DECFSZ  Bit_Cntr  , f         ;test if all done
                GOTO    Next_RcvBit
                CALL    Bit_Delay
                MOVF    Rcv_Byte, W
                RETURN

Start_Delay     MOVLW   0x3D
                MOVWF   Delay_Count
Start_Wait      NOP
                DECFSZ  Delay_Count , f
                GOTO    Start_Wait
                RETURN

Bit_Delay       MOVLW   0x7C
                MOVWF   Delay_Count
Bit_Wait        NOP
                DECFSZ  Delay_Count , f
                GOTO    Bit_Wait
                RETURN

; TOGGLE THE LED
; --------------------------------------------------------------
Flash		BTFSS	PORTB,3		; Is LED1 on? if not do it!
		goto	$+3
		BCF	PORTB,3		;Turn off LED1
		goto	$+2
		BSF	PORTB,3		;Turn on LED1
		return

; SEND CHARACTER IN W VIA usart AND WAIT UNTIL FINISHED SENDING
; -------------------------------------------------------------
send            movwf   TXREG             ; send data in W
                bsf     STATUS,RP0		; RAM PAGE 1
WtHere          btfss   TXSTA,TRMT        ; (1) transmission is complete if hi
                goto    WtHere
                bcf     STATUS,RP0          ; RAM PAGE 0
                return

; READ BUFFER
;-------------------------returns with recchar-----------------------------
receive         movf    readstart, W
                xorwf   bufhead, W
                btfsc   STATUS, Z		; readstart == bufhead?
                retlw   0xEE
;               goto    receive		; Yep, wait! (if readstart == bufhead, every character has already been processed.)
                movf    readstart, W       ; Read character in recchar
                movwf   FSR		; Put readstart in FSR
                movf    INDF, W		; Read position pointed to by readstart
                movwf   recchar		; and store in recchar
                incf    readstart, f	; Set readstart to the next position in the buffer
                movlw   buf_end		; Put buf_end in W
                xorwf   readstart, W
                btfss   STATUS, Z		; readstart == buf_end?
                return			; readstart < buf_end, end subroutine
                movlw   buf_start		; Reached end of buffer, return to start!
                movwf   readstart		; readstart = buf_start
                return

; OUTPUT to rs232 hex
; -------------------------------------------------------------
recloop         call    receive
                xorlw   0xEE
                btfsc	STATUS, Z
                return
                movf    recchar, w
                call    outhex         ;LCD_HEX
                decfsz  bufcount, f
                goto    recloop
                return

outhex          movwf	tmp1
		swapf	tmp1,	w
		andlw	0x0f
		call	HEX_Table
		call    XMIT_RS232
		movf	tmp1, w
		andlw	0x0f
		call	HEX_Table
		call    XMIT_RS232
                movlw   'h'
                call    XMIT_RS232                 ;LCD_Char
                movlw   ' '
                call    XMIT_RS232
                return

cr              movlw   0x0D                    ; CR
                call    XMIT_RS232
                movlw   0x0A                    ; lf
                call    XMIT_RS232
                return

; test which coin is dropped and add to total
;-------------------------------------------------------------------------------
cointest        movf    recchar, w
                xorlw   0x42
                btfsc   STATUS, Z
                goto    add5
                movf    recchar, w
                xorlw   0x50
                btfsc   STATUS, Z
                goto    add5
                movf    recchar, w
                xorlw   0x54
                btfsc   STATUS, Z
                goto    add10
                movf    recchar, w
                xorlw   0x55
                btfsc   STATUS, Z
                goto    add10
                movf    recchar, w
                xorlw   0x56
                btfsc   STATUS, Z
                goto    add25
                movf    recchar, w
                xorlw   0x49
                btfsc   STATUS, Z
                goto    add100
                movf    recchar, w
                xorlw   0x4A
                btfsc   STATUS, Z
                goto    add200
                movf    recchar, w
                xorlw   0x01            ; escrow lever activation detected
                btfsc   STATUS, Z
                goto    rstotal         ; reset the total
                return

rstotal         clrf	NumL
		clrf	NumH
                goto    total

add5            clrf    tmp1
                MOVLW   5
                MOVWF   tmp1
                goto    addtotal

add10           clrf    tmp1
                movlw   10
                movwf   tmp1
                goto    addtotal

add25           clrf    tmp1
                movlw   25
                movwf   tmp1
                goto    addtotal

add100          clrf    tmp1
                movlw   100
                movwf   tmp1
                goto    addtotal

add200          clrf    tmp1
                movlw   200
                movwf   tmp1
                goto    addtotal

; add tmp1 to the total
;-----------------------------------------------------------------------------------
addtotal        incfsz	NumL, 	f
		goto	$+2
		incf	NumH,	f
                DECFSZ  tmp1 , f
                GOTO    $-4        	

; output $total to lcd
;----------------------------------------------------------------------------------
total           movlw	d'7'                    ;move over 7 spaces on line 2
		call	LCD_Line2W
                movlw   '$'
                call    LCD_Char
                call	Convert			;convert to decimal
		movf	TenK,	w		;display decimal characters
		call	LCD_CharD		;using LCD_CharD to convert to ASCII
		movf	Thou,	w
		call	LCD_CharD
		movf	Hund,	w
		call	LCD_CharD
                movlw   '.'
                call    LCD_Char
		movf	Tens,	w
		call	LCD_CharD
		movf	Ones,	w
		call	LCD_CharD
                retlw   0x00

; downloaded from http://www.piclist.com Takes number in NumH:NumL Returns decimal in
; ----------------------------------- TenK:Thou:Hund:Tens:Ones ----------------------------
Convert         swapf   NumH, w
                iorlw	B'11110000'
                movwf   Thou
                addwf   Thou,f
                addlw   0XE2
                movwf   Hund
                addlw   0X32
                movwf   Ones

                movf    NumH,w
                andlw   0X0F
                addwf   Hund,f
                addwf   Hund,f
                addwf   Ones,f
                addlw   0XE9
                movwf   Tens
                addwf   Tens,f
                addwf   Tens,f

                swapf   NumL,w
                andlw   0X0F
                addwf   Tens,f
                addwf   Ones,f

                rlf     Tens,f
                rlf     Ones,f
                comf    Ones,f
                rlf     Ones,f

                movf    NumL,w
                andlw   0X0F
                addwf   Ones,f
                rlf     Thou,f

                movlw   0X07
                movwf   TenK
                    ; At this point, the original number is equal to
                    ; TenK*10000+Thou*1000+Hund*100+Tens*10+Ones
                    ; if those entities are regarded as two's complement binary.  all of
                    ; them are negative except TenK.  number
                    ; needs  normalized done with simple byte arithmetic.
                movlw   0X0A                             ; Ten
Lb1             addwf   Ones,f
                decf    Tens,f
                btfss   3,0
                goto    Lb1
Lb2             addwf   Tens,f
                decf    Hund,f
                btfss   3,0
                goto    Lb2
Lb3             addwf   Hund,f
                decf    Thou,f
                btfss   3,0
                goto   Lb3
Lb4             addwf   Thou,f
                decf    TenK,f
                btfss   3,0
                goto   Lb4

                retlw	0x00

; REFUND
; ----------------------------------------------------------------------------
refund          call    Dispense
                call    Delay255
                call    Flash
                call    Delay255
                call    Flash
                call    Delay255
                call    Flash
                call    Delay255
                call    Flash
                return


; DELAY ROUTINE
; ----------------------------------------------------
Delay255	movlw	0xff			
		goto	d0
Delay100	movlw	d'100'			
		goto	d0
Delay50		movlw	d'50'			
		goto	d0
Delay20		movlw	d'20'			;delay 20mS
		goto	d0
Delay5		movlw	0x05			;delay 5.000 ms 
d0		movwf	count1
d1              movlw	0xE7                    ; 1 ms delay 20mhz (maybe)
                movwf	counta
                movlw	0x04
                movwf	countb
Delay_0         decfsz	counta, f
                goto	$+2
                decfsz	countb, f
                goto	Delay_0
                goto	$+1
		decfsz	count1	,f
		goto	d1
		retlw	0x00

;****************************************************************************************
;  Interrupt handle ------  RECEIVE CHARACTER usart AND STORE IN buffer
; ****************************************************************************************-
isr             MOVWF	temp_w          ; save w register
                MOVF	STATUS, W       ; W = STATUS
                MOVWF	temp_s          ; save STATUS register       
        
                btfss   PIR1,RCIF         ; Check if we're called because of receiving a character
                goto    end_isr		; Interrupt came from something else

                incf    bufcount, f     ; increase the buffer counter
                movf    RCREG,W            ; Retrieve the received character and store it in tempchar
                
                movwf   tempchar
                movf    bufhead, W		; Put start of buffer in FSR
                movwf   FSR
                movf    tempchar, W
                movwf   INDF		; Copy tempchar to INDF -- which is the adress pointed to in the FSR
				;   which is bufhead.

                incf    bufhead,f		; Advance buffer head
                movlw   buf_end		; Put buf_end in W
                xorwf   bufhead, W
                btfss   STATUS, Z		; bufhead == buf_end?
                goto    end_isr		; bufhead < buf_end, end interrupt
                
                movlw   buf_start		; Reached end of buffer, return to start!
                movwf   bufhead		; bufhead = buf_start	
				
end_isr         BCF	PIR1, RCIF	; clear the RCIF flag bit
                MOVF	temp_s, W	; Put STATUS and W back where they belong
             	MOVWF	STATUS
             	SWAPF	temp_w, F
             	SWAPF	temp_w, W
          	RETFIE			; Return from interrupt


;#####################################################################################################
; -------------------------------------- Start ---------------------------------
; ********************************
; *       	init             *
; ********************************
Start           movlw   7
                movwf   CMCON		; CMCON=7 set comperators off
                movlw   b'00000000'       ; set up portA and B
                movwf   PORTA
                movwf   PORTB
		BSF	RCSTA,SPEN      ;uart on
		BSF	RCSTA,CREN      ;uart on
                BSF	RCSTA,RX9       ; enable 9 bit uart
		BCF	STATUS,C	;Set carry 0
		BSF	STATUS,RP0	;init - set ram page 1
                MOVLW   B'00100000'     ; Disable all peripheral interrupts except receiver
                MOVWF   PIE1             ;Peripheral interrupt enable/disable
                MOVLW   B'01000000'     ;B'01000000' Disable all interrupts except peripheral
                MOVWF   INTCON           ;Interrupt control register
                MOVLW	b'00100010'      ; RB1(RX)& rb5=input, others output
		MOVWF	TRISB
		BSF	TXSTA,BRGH
		BSF	TXSTA,TXEN
		BSF	TXSTA,TX9	;select 9 bit
		movlw	d'129'         ; 9600 baud
	        movwf	SPBRG
		BCF	STATUS,RP0	;set ram page 0
                BSF     INTCON,7     ;Enable all unmasked interrupts

                movlw   buf_start		; Put adress of start buffer in the various registers
                movwf   bufhead
                movwf   readstart
                clrf    bufcount            ; clear the variables
                clrf	NumL
		clrf	NumH

                movf    RCREG,W
                movf    RCREG,W
                movf    RCREG,W            ; flush receive buffer
                call    Delay255
                call    Delay255
                call    SER_INIT
                call	LCD_Init

; ----------- lcd wait message -------------
		clrf	tmp1			;set counter register to zero
Message		movf	tmp1, w		;put counter value in W
		call	Text2			;get a character from the text table
		xorlw	0x00			;is it a zero?
		btfsc	STATUS, Z
		goto	$+4                    ; sends top line message
		call    LCD_Char                        ;LCD_Char
		incf	tmp1, f
		goto	Message              
               



;--------------changer sequence-------------------------------------------------------------
;---------reset the changer----------------         
                clrf    bufcount        ; clear the buffer counter
                call    Rset            ; send reset command to changer
                                        ; should return an ACK(00h) byte from vend
                call    Flash           ;toggle the led
                call    Delay255        ; wait 255
                call    recloop         ;print buffer to rs232
                call    Flash           ;toggle the led
                call    Delay255        ; wait 255
                call    cr
; poll to receive "just reset" response
                clrf    bufcount
                call    Poll
                call    Delay5
                call    recloop
                call    cr
; send ACK
                call    ack
                call    Delay50
; send STATUS request
                clrf    bufcount
                call    Setup
                call    Delay5
                call    recloop
                call    cr
; send ACK
                call    ack
                call    Delay50
;------enable the changer ---------------------------
                clrf    bufcount
                call    Tube        ;get tube status
                call    Delay5
                call    recloop
                call    cr
; send ACK
                call    ack
                call    Delay50
; coin type enable
                clrf    bufcount
                call    Cointype    ;
                call    Delay5
                call    recloop
                call    cr


;-----------big delay-------------------
                MOVLW   20
                MOVWF   tmp1
                call    Delay255
                DECFSZ  tmp1 , f
                GOTO    $-2

; output ready message to lcd
; ----------------------------------------------------------------------
                call    LCD_Clr             ;clear lcd
                movlw	d'5'                    ;move over 5 spaces on lcd line 1
		call	LCD_Line1W
                clrf	tmp1			;set counter register to zero
Message2	movf	tmp1, w		;put counter value in W
		call	Text			;get a character from the text table
		xorlw	0x00			;is it a zero?
		btfsc	STATUS, Z
		goto	$+4                    ; sends top line message
		call    LCD_Char                        ;LCD_Char
		incf	tmp1, f
		goto	Message2


; output total to second line of lcd
;------------------------------------------------------------------------------
                call    total       ;display total cash

; ///////////////////////////////////////////////////////////////////////////
; /////////////////////////////////////////////////////////////////////////////
;   MAIN LOOP             
Loop            clrf    bufcount        ;start the buffer counter at zero
                call    Poll            ; returns with data in buffer & bufcount=total
                call    Delay5
                call    ack
                call    Delay50
 
                call    receive         ;call for first chr in buffer only
                xorlw   0xEE            ;if return from call with EE in w than buffer empty
                btfsc	STATUS, Z
                goto    $+6           ; skip
                decf    bufcount, f    ;buffer has one less chr now
              
                call    cointest        ;test the recchar for cointypes

                movf    recchar, w
                call    outhex
                call    recloop     ; empty the buffer

                call    cr
                btfss   PORTA, 5      ;check to see if button pressed
                call    refund          ;dispense a dime if pressed
   ;             call    Flash
                call    Delay100
  
                goto    Loop

;   END MAIN LOOP
; ///////////////////////////////////////////////////////////////////////
; //////////////////////////////////////////////////////////////////////

		END

                
