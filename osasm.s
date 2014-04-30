;/*****************************************************************************/
;/* OSasm.s: low-level OS commands, written in assembly                       */
;// Real Time Operating System 

; This example accompanies the book
;  "Embedded Systems: Real Time Interfacing to the Arm Cortex M3",
;  ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2011
;
;  Programs 6.4 through 6.12, section 6.2
;
;Copyright 2011 by Jonathan W. Valvano, valvano@mail.utexas.edu
;    You may use, edit, run or distribute this file
;    as long as the above copyright notice remains
; THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
; OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
; MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
; VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
; OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
; For more information about my classes, my research, and my books, see
; http://users.ece.utexas.edu/~valvano/
; */

;Modified by Zirui adapted from GPIO.s
; PD3 is an output to LED3, negative logic
; PD2 is an output to LED2, negative logic
; PD1 is an output to LED1, negative logic
; PD0 is an output to LED0, negative logic
PIND0              EQU 0x40007004   ; access PD3-PD0
GPIO_PORTD_DATA_R  EQU 0x400073FC
GPIO_PORTD_DIR_R   EQU 0x40007400
GPIO_PORTD_AFSEL_R EQU 0x40007420
GPIO_PORTD_DR8R_R  EQU 0x40007508
GPIO_PORTD_DEN_R   EQU 0x4000751C
GPIO_PORTD_AMSEL_R EQU 0x40007528
GPIO_PORTD_PCTL_R  EQU 0x4000752C
SYSCTL_RCGC2_R     EQU 0x400FE108
SYSCTL_RCGC2_GPIOD EQU 0x00000008   ; port D Clock Gating Control

        AREA |.text|, CODE, READONLY, ALIGN=2
        THUMB
        REQUIRE8
        PRESERVE8

        EXTERN  RunPt            ; currently running thread
		EXTERN  NextPt           ; next non-sleep thread
        EXPORT  OS_DisableInterrupts
        EXPORT  OS_EnableInterrupts
        ;EXPORT  StartCritical
       ;EXPORT  EndCritical
        EXPORT  StartOS

        EXPORT  PendSV_Handler

GPIO_Init
    ; 1) activate clock for Port D
    LDR R1, =SYSCTL_RCGC2_R         ; R1 = &SYSCTL_RCGC2_R
    LDR R0, [R1]                    ; R0 = [R1]
    ORR R0, R0, #SYSCTL_RCGC2_GPIOD ; R0 = R0|SYSCTL_RCGC2_GPIOD
    STR R0, [R1]                    ; [R1] = R0
    NOP
    NOP                             ; allow time to finish activating
    ; 2) no need to unlock PD3-0
    ; 3) disable analog functionality
    LDR R1, =GPIO_PORTD_AMSEL_R     ; R1 = &GPIO_PORTD_AMSEL_R
    LDR R0, [R1]                    ; R0 = [R1]
    BIC R0, R0, #0x0F               ; R0 = R0&~0x0F (disable analog functionality on PD3-0)
    STR R0, [R1]                    ; [R1] = R0    
    ; 4) configure as GPIO
    LDR R1, =GPIO_PORTD_PCTL_R      ; R1 = &GPIO_PORTD_PCTL_R
    LDR R0, [R1]                    ; R0 = [R1]
    MOV R2, #0x0000FFFF             ; R2 = 0x0000FFFF
    BIC R0, R0, R2                  ; R0 = R0&~0x0000FFFF (clear port control field for PD3-0)
    STR R0, [R1]                    ; [R1] = R0

    ; 5) set direction register
    LDR R1, =GPIO_PORTD_DIR_R       ; R1 = &GPIO_PORTD_DIR_R
    LDR R0, [R1]                    ; R0 = [R1]
    ORR R0, R0, #0x0F               ; R0 = R0|0x0F (make PD3-0 output)
    STR R0, [R1]                    ; [R1] = R0
    ; 6) regular port function
    LDR R1, =GPIO_PORTD_AFSEL_R     ; R1 = &GPIO_PORTD_AFSEL_R
    LDR R0, [R1]                    ; R0 = [R1]
    BIC R0, R0, #0x0F               ; R0 = R0&~0x0F (disable alt funct on PD3-0)
    STR R0, [R1]                    ; [R1] = R0
    ; enable 8mA drive (only necessary for bright LEDs)
    LDR R1, =GPIO_PORTD_DR8R_R      ; R1 = &GPIO_PORTD_DR8R_R
    LDR R0, [R1]                    ; R0 = [R1]
    ORR R0, R0, #0x0F               ; R0 = R0|0x0F (enable 8mA drive on PD3-0)
    STR R0, [R1]                    ; [R1] = R0
    ; 7) enable digital port
    LDR R1, =GPIO_PORTD_DEN_R       ; R1 = &GPIO_PORTD_DEN_R
    LDR R0, [R1]                    ; R0 = [R1]
    ORR R0, R0, #0x0F               ; R0 = R0|0x0F (enable digital I/O on PD3-0)
    STR R0, [R1]                    ; [R1] = R0
    BX  LR
    

OS_DisableInterrupts
        CPSID   I
        BX      LR


OS_EnableInterrupts
        CPSIE   I
        BX      LR

;*********** StartCritical************************
; make a copy of previous I bit, disable interrupts
; inputs:  none
; outputs: previous I bit
;StartCritical
;        MRS     R0, PRIMASK        ; Set prio int mask to mask all (except faults)
 ;       CPSID   I
  ;      BX      LR


;*********** EndCritical************************
; using the copy of previous I bit, restore I bit to previous value
; inputs:  previous I bit
; outputs: none
;EndCritical
 ;       MSR     PRIMASK, R0
  ;      BX      LR


PendSV_Handler                 ; 1) Saves R0-R3,R12,LR,PC,PSR
    CPSID   I                  ; 2) Prevent interrupt during switch
    PUSH    {R4-R11}           ; 3) Save remaining regs r4-11
	
	;modified by Zirui
	;LDR R10, =PIND0
	;LDR R11, [R10]
	;EOR R11, R11, #1 
	;STR R11, [R10]
	;EOR R11, R11, #1
	;STR R11, [R10]
	;end of modification
	
    LDR     R0, =RunPt         ; 4) R0=pointer to RunPt, old thread
    LDR     R1, [R0]           ;    R1 = RunPt
    STR     SP, [R1]           ; 5) Save SP into TCB
    LDR     R1, =NextPt        ; 6) R1 = pointer to NextPt, first non-sleep thread, modified by annyan and zirui
	LDR     R1, [R1]           ;    R1 = NextPt, end of modification
    STR     R1, [R0]           ;    RunPt = R1
    LDR     SP, [R1]           ; 7) new thread SP; SP = RunPt->sp;
	
	;modified by Zirui
	;LDR R10, =PIND0
	;LDR R11, [R10]
	;EOR R11, R11, #1
	;STR R11, [R10]
	;end of modification
	
    POP     {R4-R11}           ; 8) restore regs r4-11
    CPSIE   I                  ; 9) tasks run with interrupts enabled
    BX      LR                 ; 10) restore R0-R3,R12,LR,PC,PSR

StartOS
	
	BL  GPIO_Init              ;Modified by Zirui
	
    LDR     R0, =RunPt         ; currently running thread
    LDR     R2, [R0]           ; R2 = value of RunPt
    LDR     SP, [R2]           ; new thread SP; SP = RunPt->stackPointer;
    POP     {R4-R11}           ; restore regs r4-11
    POP     {R0-R3}            ; restore regs r0-3
    POP     {R12}
    POP     {LR}               ; discard LR from initial stack
    POP     {R2}               ; start location
    POP     {R1}               ; discard PSR
    CPSIE   I                  ; Enable interrupts at processor level
    BX      R2                 ; start first thread

    ALIGN
    END
