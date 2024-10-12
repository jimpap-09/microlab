.org 0x00
jmp main

; Delay subroutine
wait_x_msec:
    push r22
    push r23
    push r24
    push r25

    ldi r24, 0x01	        ; Load low byte
    ldi r25, 0x00	        ; Load high byte
x_loop:			        ; x = 0001
    ldi r23, 0x10	        ; Load outer_loop counter
outer_loop:

    ldi r22, 0xFA	        ; Load inner_loop counter
inner_loop:
    nop
    dec r22		        ; Decrement inner loop counter
    brne inner_loop	        ; delay = 4*250 = 1000 cycles

    dec r23		        ; Decrement outer loop counter
    brne outer_loop	        ; delay = 16 * 1000 cycles

    sbiw r24, 1		        ; Decrement wait_x_ms loop counter
    brne x_loop		        ; delay = x * 16 * 1000 cycles

    pop r25
    pop r24
    pop r23
    pop r22
    ret

main:
; initialize stack pointer
ldi r24, LOW(RAMEND)        	; 1 cycle
out SPL, r24		        ; 2 cycles
ldi r24, HIGH(RAMEND)       	; 1 cycle
out SPH, r24		        ; 2 cycles

rcall wait_x_msec

rcall wait_x_msec
