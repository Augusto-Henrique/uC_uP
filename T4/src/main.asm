;*******************************************************************************
;                                                                              *
;    Filename: main                                                            *
;    Date: 23/09/2025                                                          *
;    File Version: 0.1                                                         *
;    Author: Samuel e Augusto                                                  *
;                                                                              *
;*******************************************************************************
;                                                                              *
;    Known Issues: Não sei se funciona
;                                                                              *
;*******************************************************************************
;                                                                              *
;    Revision History:                                                         *
;    23/09/2025 - Criação                                                      *
;    24/09/2025 - Estados adicionados e configurados                           *
;                                                                              *
;*******************************************************************************
;*******************************************************************************
; PIC18F4550 Configuration Bit Settings
;*******************************************************************************
#include "p18f4550.inc"

; CONFIG1L
  CONFIG  PLLDIV = 1            ; PLL Prescaler Selection bits (No prescale (4 MHz oscillator input drives PLL directly))
  CONFIG  CPUDIV = OSC1_PLL2    ; System Clock Postscaler Selection bits ([Primary Oscillator Src: /1][96 MHz PLL Src: /2])
  CONFIG  USBDIV = 1            ; USB Clock Selection bit (used in Full-Speed USB mode only; UCFG:FSEN = 1) (USB clock source comes directly from the primary oscillator block with no postscale)

; CONFIG1H
  CONFIG  FOSC = XT_XT          ; Oscillator Selection bits (XT oscillator (XT))
  CONFIG  FCMEN = OFF           ; Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
  CONFIG  IESO = OFF            ; Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

; CONFIG2L
  CONFIG  PWRT = ON             ; Power-up Timer Enable bit (PWRT enabled)
  CONFIG  BOR = OFF             ; Brown-out Reset Enable bits (Brown-out Reset disabled in hardware and software)
  CONFIG  BORV = 0              ; Brown-out Reset Voltage bits (Maximum setting 4.59V)
  CONFIG  VREGEN = OFF          ; USB Voltage Regulator Enable bit (USB voltage regulator disabled)

; CONFIG2H
  CONFIG  WDT = OFF             ; Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
  CONFIG  WDTPS = 1             ; Watchdog Timer Postscale Select bits (1:1)

; CONFIG3H
  CONFIG  CCP2MX = OFF          ; CCP2 MUX bit (CCP2 input/output is multiplexed with RB3)
  CONFIG  PBADEN = OFF          ; PORTB A/D Enable bit (PORTB<4:0> pins are configured as digital I/O on Reset)
  CONFIG  LPT1OSC = OFF         ; Low-Power Timer 1 Oscillator Enable bit (Timer1 configured for higher power operation)
  CONFIG  MCLRE = OFF           ; MCLR Pin Enable bit (RE3 input pin enabled; MCLR pin disabled)

; CONFIG4L
  CONFIG  STVREN = OFF          ; Stack Full/Underflow Reset Enable bit (Stack full/underflow will not cause Reset)
  CONFIG  LVP = OFF             ; Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
  CONFIG  ICPRT = OFF           ; Dedicated In-Circuit Debug/Programming Port (ICPORT) Enable bit (ICPORT disabled)
  CONFIG  XINST = OFF           ; Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

; CONFIG5L
  CONFIG  CP0 = ON              ; Code Protection bit (Block 0 (000800-001FFFh) is code-protected)
  CONFIG  CP1 = ON              ; Code Protection bit (Block 1 (002000-003FFFh) is code-protected)
  CONFIG  CP2 = ON              ; Code Protection bit (Block 2 (004000-005FFFh) is code-protected)
  CONFIG  CP3 = ON              ; Code Protection bit (Block 3 (006000-007FFFh) is code-protected)

; CONFIG5H
  CONFIG  CPB = ON              ; Boot Block Code Protection bit (Boot block (000000-0007FFh) is code-protected)
  CONFIG  CPD = ON              ; Data EEPROM Code Protection bit (Data EEPROM is code-protected)

; CONFIG6L
  CONFIG  WRT0 = ON             ; Write Protection bit (Block 0 (000800-001FFFh) is write-protected)
  CONFIG  WRT1 = ON             ; Write Protection bit (Block 1 (002000-003FFFh) is write-protected)
  CONFIG  WRT2 = ON             ; Write Protection bit (Block 2 (004000-005FFFh) is write-protected)
  CONFIG  WRT3 = ON             ; Write Protection bit (Block 3 (006000-007FFFh) is write-protected)

; CONFIG6H
  CONFIG  WRTC = ON             ; Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) are write-protected)
  CONFIG  WRTB = ON             ; Boot Block Write Protection bit (Boot block (000000-0007FFh) is write-protected)
  CONFIG  WRTD = ON             ; Data EEPROM Write Protection bit (Data EEPROM is write-protected)

; CONFIG7L
  CONFIG  EBTR0 = ON            ; Table Read Protection bit (Block 0 (000800-001FFFh) is protected from table reads executed in other blocks)
  CONFIG  EBTR1 = ON            ; Table Read Protection bit (Block 1 (002000-003FFFh) is protected from table reads executed in other blocks)
  CONFIG  EBTR2 = ON            ; Table Read Protection bit (Block 2 (004000-005FFFh) is protected from table reads executed in other blocks)
  CONFIG  EBTR3 = ON            ; Table Read Protection bit (Block 3 (006000-007FFFh) is protected from table reads executed in other blocks)

; CONFIG7H
  CONFIG  EBTRB = ON            ; Boot Block Table Read Protection bit (Boot block (000000-0007FFh) is protected from table reads executed in other blocks)

;*******************************************************************************
; Reset Vector
;*******************************************************************************

RES_VECT  CODE    0x0000            ; processor reset vector
    GOTO    MAIN                    ; go to beginning of program

;*******************************************************************************
; Interruption routines
;*******************************************************************************

ISRHV     CODE    0x0008
    GOTO    HIGH_ISR
ISRLV     CODE    0x0018
    GOTO    LOW_ISR

ISRH      CODE                     

HIGH_ISR
    BTFSS   PIR1, TMR1IF       ; Timer1 estourou?
    GOTO    NOT_TMR1

    INCF    TICKS, F           ; conta n estouros
    BCF     PIR1, TMR1IF       ; limpa flag

NOT_TMR1
    RETFIE FAST

ISRL      CODE                     ; let linker place low ISR routine
      
LOW_ISR
;       <Search the device datasheet for 'context' and copy interrupt
;       context saving code here>
    RETFIE

;*******************************************************************************
; MAIN PROGRAM
;*******************************************************************************
CBLOCK 0x20
    STATE ;Variavel de estado
    TICKS ;Variavel de contagem
ENDC

;Define dos pinos
#define M_BUTTON PORTB, 0
#define P_SENSOR PORTC, 0
#define A_SENSOR PORTC, 1
#define B_SENSOR PORTC, 2
#define GATE PORTD, 0
 
;Define dos estados
IDLE EQU 0
START EQU 1
RIGHT EQU 2
OPEN EQU 3
TIME EQU 4
FINISH EQU 5

MAIN_PROG CODE                      ; let linker place main program

MAIN
    MOVLW 0x0F
    MOVWF ADCON1

    ; Configura entradas e saidas
    BSF TRISB, 0
    BSF TRISC, 0
    BSF TRISC, 1
    BSF TRISC, 2
    BSF TRISD, 0
    
    ; Inicializacao da máquina de estados
    MOVLW IDLE
    MOVWF STATE
    
    ;Inicialização do timer1
    BSF RCON, IPEN      ; habilita prioridades
    CLRF T1CON
    BSF T1CON, T1CKPS0  ; prescaler 1:8
    BCF T1CON, T1CKPS1
    BSF T1CON, TMR1ON   ; liga Timer1

    CLRF PIR1
    BSF PIE1, TMR1IE    ; habilita interrupcao Timer1
    BSF IPR1, TMR1IP    ; coloca Timer1 em alta prioridade
    
    ;Interrupções globais
    BSF INTCON, PEIE
    BSF INTCON, GIEH    ; habilita interrupcoes high
    BSF INTCON, GIEL    ; habilita interrupcoes low (nenhuma implementada)

    
;-------------------------------------------------------
; Máquina de estados 
;-------------------------------------------------------
    
IDLE_ROUTINE ; Estado de espera
    BTFSC A_SENSOR ; Só muda de estado quando sensor A é 1
    GOTO IDLE_ROUTINE
    
    MOVLW START ; Atualiza estado
    MOVWF STATE

START_ROUTINE
    BTFSC M_BUTTON
    GOTO START_ROUTINE
    
    MOVLW RIGHT
    MOVWF STATE
    
RIGHT_ROUTINE
    BTFSS B_SENSOR
    GOTO RIGHT_ROUTINE
    
    MOVLW OPEN
    MOVWF STATE
    
OPEN_ROUTINE
    BTFSS P_SENSOR
    GOTO OPEN_ROUTINE
    
    MOVF TIME
    MOVWF STATE

TIME_ROUTINE
    CALL Delay_5s
    MOVF FINISH
    MOVWF STATE

FINISH_ROUTINE
    BTFSS A_SENSOR
    GOTO FINISH_ROUTINE
    
    MOVF IDLE
    MOVWF STATE

ROTATE_RIGHT ; Tem que implementar 

ROTATE_LEFT ; Tem que implementar 

;=========================================================================
; Cálculo do número de estouros do Timer1 (Fosc = 8 MHz, prescaler 1:8)
;=========================================================================
; Tcy = Fosc / 4 = 8 MHz / 4 = 2 MHz ? 1 ciclo = 0,5 us
; Ttick = Tcy * prescaler = 0,5 us * 8 = 4 us
; Tempo 1 overflow = 65536 * Ttick ? 0,262 s
; Nº de estouros para 5 s = 5 / 0,262 ? 19 (Prefiro usar 20 para 5,24s)
;=========================================================================
;-------------------------------------------------------
; Rotina de delay de 5s
;-------------------------------------------------------
Delay_5s:
    CLRF    TICKS              ; zera contador
WAIT_LOOP:
    MOVLW   20                 ; 19/20 estouros (4,978s ou 5,24s)
    CPFSLT  TICKS              ; já chegou?
    GOTO    WAIT_LOOP          ; não -> continua esperando
    RETURN
END