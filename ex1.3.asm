.def CTR = r20
.def DIR = r21

.org 0x0000
jmp main

; Delay subroutine
wait_1000_ms:
    push r22
    push r23
    push r24
    push r25

    ldi r24, 0xE8	        ; Load low byte
    ldi r25, 0x03	        ; Load high byte
x_loop:			        ; x = 1000
    ldi r23, 0x10	        ; Load outer_loop counter
outer_loop:

    ldi r22, 0xFA	        ; Load inner_loop counter
inner_loop:
    nop
    dec r22		        ; Decrement inner loop counter
    brne inner_loop	        ; delay = 4*250 = 1000 cycles

    dec r23		        ; Decrement outer loop counter
    brne outer_loop	        ; delay = 16 * 1000 cycles

    sbiw r24, 1		        ; Decrement wait_1000_ms loop counter
    brne x_loop		        ; delay = 1000 * 16 * 1000 cycles

    pop r25
    pop r24
    pop r23
    pop r22
    ret

main:
; initialize stack pointer
ldi r24, LOW(RAMEND)		; 1 cycle
out SPL, r24		        ; 2 cycles
ldi r24, HIGH(RAMEND)		; 1 cycle
out SPH, r24		        ; 2 cycles

; set PORTD as output
ser r19
out ddrd, r19

; we start moving from MSB -> LSB, where T = 0
ldi CTR, 0x80
clr DIR
bst DIR, 0

; move right 
moveRight:
    out PORTD, CTR		; output the current position
    rcall wait_1000_ms		; wait for 1 sec
    lsr CTR		        ; shift right for 1 bit
    cpi CTR, 0x01	        ; compare with LSB
    brne moveRight	        ; if not LSB then continue

; change direction
changeDirection:
    out PORTD, CTR	        ; output the edge position
    rcall wait_1000_ms		; wait for 1 sec
    com DIR		        ; DIR <- ~DIR
    bst DIR, 0		        ; T <- DIR0
    cpi DIR, 0x01	        ; compare with 1
    brne moveRight	        ; if T != 1 then moveRight

; move left
moveLeft:
    out PORTD, CTR	        ; output the current position
    rcall wait_1000_ms		; wait for 1 sec
    lsl CTR		        ; shift left for 1 bit
    cpi CTR, 0x80	        ; compare with MSB
    brne moveLeft	        ; if not MSB then continue
    jmp changeDirection		; else changeDirection
