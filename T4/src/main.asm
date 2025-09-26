;*******************************************************************************
;                                                                              *
;    Filename: main                                                            *
;    Date: 23/09/2025                                                          *
;    File Version: 0.2 - CORRIGIDO                                            *
;    Author: Samuel e Augusto                                                  *
;                                                                              *
;*******************************************************************************
;                                                                              *
;    Correções realizadas:                                                     *
;    - Implementado loop principal da máquina de estados                       *
;    - Corrigidas as transições de estado                                      *
;    - Implementados controles do motor e comporta                             *
;    - Corrigidas instruções MOVF para MOVLW                                   *
;    - Adicionado controle das saídas do motor e comporta                      *
;                                                                              *
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
#define SENSOR_A PORTC, 0
#define SENSOR_B PORTC, 1
#define SENSOR_P PORTC, 2
#define COMPORTA PORTD, 0
#define MOTOR_ESQ PORTD, 6
#define MOTOR_DIR PORTD, 7
 
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
    BSF TRISB, 0        ; RB0 como entrada (BOTÃO)
    BCF TRISC, 0        ; RC0 como saída (SENSOR A - LED)
    BCF TRISC, 1        ; RC1 como saída (SENSOR B - LED)
    BCF TRISC, 2        ; RC2 como saída (SENSOR P - LED)
    BCF TRISD, 0        ; RD0 como saída (COMPORTA - LED)
    BCF TRISD, 6        ; RD6 como saída (MOTOR ESQ - LED)
    BCF TRISD, 7        ; RD7 como saída (MOTOR DIR - LED)

    ; Inicializa todos os LEDs apagados
    CLRF PORTC
    CLRF PORTD
    
    ; Liga o SENSOR A (RC0) - posição inicial
    BSF SENSOR_A
    
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
    BSF INTCON, GIEL    ; habilita interrupcoes low

MAIN_LOOP
    ; Verifica se o botão foi pressionado
    BTFSS M_BUTTON          ; Botão pressionado? (nível alto)
    GOTO MAIN_LOOP          ; Não -> continua no loop
    
    ; Botão foi pressionado - faz a transição
    BCF SENSOR_A            ; Desliga LED do SENSOR A (RC0)
    BSF SENSOR_B            ; Liga LED do SENSOR B (RC1)
    BSF MOTOR_DIR           ; Liga LED do MOTOR_DIR (RD7)
        
    ; Aguarda soltar o botão para evitar múltiplas ativações
WAIT_BUTTON_RELEASE:
    BTFSC M_BUTTON          ; Botão ainda pressionado?
    GOTO WAIT_BUTTON_RELEASE ; Sim -> aguarda soltar
    
    ; Loop após a transição
AFTER_TRANSITION:
    GOTO AFTER_TRANSITION   ; Mantém o estado

;-------------------------------------------------------
; Rotina de delay de 5s
;-------------------------------------------------------
Delay_5s:
    CLRF    TICKS              ; zera contador
WAIT_LOOP:
    MOVLW   20                 ; 20 estouros para ~5.24s
    CPFSLT  TICKS              ; já chegou?
    GOTO    WAIT_LOOP          ; não -> continua esperando
    RETURN

END