; define reg A, B, C, D, I
.def A = r18
.def B = r19
.def C = r20
.def D = r21
.def F0 = r22
.def F1 = r23
.def I = r24
.def T = r25

.org 0x0000
rjmp main

; F0 = (AB' + B'D)' = (B'(A+D))' = B + (A'D')
logicF0:
    push A
    push B
    push C
    push D

    com A		            ; A <- A'
    com D		            ; D <- D'
    and A, D		        ; A <- A'D'
    or B, A		            ; B <- B + A'D'

    mov F0, B		        ; F0 <- B + A'D'

    pop D
    pop C
    pop B
    pop A

    ret

; F1 = (A+C')(B+D')
logicF1:
    push A
    push B
    push C
    push D

    com C		            ; C <- C'
    or A, C		            ; A <- A + C'
    com D		            ; D <- D'
    or B, D		            ; B <- B + D'
    and A, B		        ; A <- (A + C')(B + D')

    mov F1, A		        ; F1 <- (A + C')(B + D')

    pop D
    pop C
    pop B
    pop A

    ret

main:
; initialize stack pointer
ldi r24, LOW(RAMEND)        ; 1 cycle
out SPL, r24		        ; 2 cycles
ldi r24, HIGH(RAMEND)       ; 1 cycle
out SPH, r24		        ; 2 cycles

; initialize reg A, B, C, D
ldi A, 0x51
ldi B, 0x41
ldi C, 0x21
ldi D, 0x01

; initialize the counter
ldi I, 0x06		            ; Load loop counter
loop:
    rcall logicF0	        ; call F0
    rcall logicF1	        ; call F1

    inc A		            ; A += 1

    ldi T, 0x02		        ; T = temporary register
    add B, T		        ; B += 2

    ldi T, 0x03
    add C, T		        ; C += 3

    ldi T, 0x04
    add D, T		        ; D += 4

    dec I		            ; I -= 1
    brne loop
