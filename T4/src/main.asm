;*******************************************************************************
;                                                                              *
;    Filename: main                                                            *
;    Date: 23/09/2025                                                          *
;    File Version: 0.1                                                         *
;    Author: Samuel e Augusto                                                  *
;                                                                              *
;*******************************************************************************
;                                                                              *
;    Known Issues: Sem debounce do botao M
;                                                                              *
;*******************************************************************************
;                                                                              *
;    Revision History:                                                         *
;    23/09/2025 - Cria��o                                                      *
;    24/09/2025 - Estados adicionados e configurados                           *
;    25/09/2025 - Estados alterados e simplifica��o                            *
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
  CONFIG  FOSC = INTOSC_HS      ; Oscillator Selection bits (XT oscillator (XT))
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
    ; salvar contexto (WREG, STATUS, BSR)
    MOVWF   SAVED_W
    MOVFF   STATUS, SAVED_STATUS
    MOVFF   BSR, SAVED_BSR

    ; trata Timer1
    BTFSS   PIR1, TMR1IF
    GOTO    _HIGH_ISR_DONE

    INCF    TICKS, F           ; conta estouros
    BCF     PIR1, TMR1IF       ; limpa flag

_HIGH_ISR_DONE
    ; restaurar contexto
    MOVFF   SAVED_BSR, BSR
    MOVFF   SAVED_STATUS, STATUS
    MOVF    SAVED_W, W
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
    STATE           ; Variavel de estado
    TICKS           ; Variavel de contagem
    SAVED_W         ; tempor�rio para salvar WREG na ISR
    SAVED_STATUS    ; tempor�rio para salvar STATUS na ISR
    SAVED_BSR       ; tempor�rio para salvar BSR na ISR
ENDC

;Define dos pinos
#define M_BUTTON PORTB, 0
#define P_SENSOR PORTC, 2
#define A_SENSOR PORTC, 0
#define B_SENSOR PORTC, 1
#define GREEN_LED LATA, 0
#define YELLOW_LED LATA, 1
#define RED_LED LATA, 2
#define BLUE_LED LATA, 3
#define GATE LATE, 0
 
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
    BCF TRISE, 0
    BCF TRISA, 0
    BCF TRISA, 1
    BCF TRISA, 2
    BCF TRISA, 3
    
    BCF TRISD, 0
    BCF TRISD, 1
    BCF TRISD, 2
    BCF TRISD, 3
    
    ; inicializa sa�das
    BCF LATA,0
    BCF LATA,1
    BCF LATA,2
    BCF LATA,3
    BCF LATE,0
    
    ; Inicializacao da m�quina de estados
    MOVLW IDLE
    MOVWF STATE
    
    ;Inicializa��o do timer1
    BSF RCON, IPEN      ; habilita prioridades
    CLRF T1CON
    ; prescaler 1:8 -> T1CKPS1:T1CKPS0 = 11
    BSF T1CON, T1CKPS0
    BSF T1CON, T1CKPS1
    ; zera contador de TMR1 para transi��es previs�veis
    CLRF TMR1L
    CLRF TMR1H
    BSF T1CON, TMR1ON   ; liga Timer1

    CLRF PIR1
    BSF PIE1, TMR1IE    ; habilita interrupcao Timer1
    BSF IPR1, TMR1IP    ; coloca Timer1 em alta prioridade
    
    ;Interrup��es globais
    BSF INTCON, PEIE
    BSF INTCON, GIEH    ; habilita interrupcoes high
    BSF INTCON, GIEL    ; habilita interrupcoes low (nenhuma implementada)

  MAIN_LOOP
    ; Troca simples de estados
    MOVF STATE, W
    XORLW IDLE
    BTFSC STATUS, Z
    GOTO IDLE_ROUTINE

    MOVF STATE, W
    XORLW START
    BTFSC STATUS, Z
    GOTO START_ROUTINE

    MOVF STATE, W
    XORLW RIGHT
    BTFSC STATUS, Z
    GOTO RIGHT_ROUTINE

    MOVF STATE, W
    XORLW OPEN
    BTFSC STATUS, Z
    GOTO OPEN_ROUTINE

    MOVF STATE, W
    XORLW TIME
    BTFSC STATUS, Z
    GOTO TIME_ROUTINE

    MOVF STATE, W
    XORLW FINISH
    BTFSC STATUS, Z
    GOTO FINISH_ROUTINE

    GOTO MAIN_LOOP

;-------------------------------------------------------
; M�quina de estados 
;-------------------------------------------------------
    
IDLE_ROUTINE ; Carro andando at� sensor A = 1
    ; LEDs
    BSF GREEN_LED
    BCF YELLOW_LED
    BCF BLUE_LED
    BCF RED_LED
    
    BCF GATE ; Comporta fechada
    
    BTFSC   A_SENSOR
    GOTO    IDLE_REACHED
    GOTO    MAIN_LOOP

IDLE_REACHED
    MOVLW START
    MOVWF STATE
    GOTO MAIN_LOOP

START_ROUTINE ; Espera pressionar o bot�o M
    ; LEDs
    BSF GREEN_LED
    BCF YELLOW_LED
    BCF BLUE_LED
    BCF RED_LED

    BCF GATE  ; Comporta fechada

    ; bot�o M = 0 para come�ar
    BTFSC M_BUTTON
    GOTO MAIN_LOOP        ; se M = 1 continua esperando

    ; Bot�o pressionado -> entra em RIGHT
    MOVLW RIGHT
    MOVWF STATE
    GOTO MAIN_LOOP

RIGHT_ROUTINE ; Espera sensor B = 1
    ; LEDs
    BCF GREEN_LED
    BSF YELLOW_LED
    BCF BLUE_LED
    BCF RED_LED

    BCF GATE ; Comporta fechada
    CALL ROTATE_RIGHT
    
    ; B = 1 -> transita para OPEN
    BTFSS B_SENSOR
    GOTO MAIN_LOOP        ; B = 0 -> permanece em RIGHT
    MOVLW OPEN
    MOVWF STATE
    GOTO MAIN_LOOP

OPEN_ROUTINE ; Abre a comporta e espera sensor P = 1
    ; LEDs
    BCF GREEN_LED
    BSF YELLOW_LED
    BCF BLUE_LED
    BCF RED_LED

    ; Abre a comporta (uma vez)
    CALL Delay_5ms
    BSF GATE
    ;Leve delay antes de checar sensor P
    NOP
    NOP
    NOP

    ; Espera P=1
    BTFSS P_SENSOR
    GOTO MAIN_LOOP

    ; P=1 detectado -> fecha comporta e vai para TIME
    BCF GATE
    MOVLW TIME
    MOVWF STATE
    GOTO MAIN_LOOP


TIME_ROUTINE ; Espera 5s
    ; LEDs
    BCF GREEN_LED
    BCF YELLOW_LED
    BSF BLUE_LED
    BCF RED_LED

    BCF GATE ; Comporta fechada
    CALL Delay_5s ; delay de 5s
    MOVLW FINISH
    MOVWF STATE
    GOTO MAIN_LOOP

FINISH_ROUTINE ; Espera B = 0
    ; LEDs
    BCF GREEN_LED
    BCF YELLOW_LED
    BCF BLUE_LED
    BSF RED_LED

    BCF GATE ; Comporta fechada
    ; Aguarda sensor B = 0 para completar ciclo
    CALL ROTATE_LEFT
    
    BTFSS   B_SENSOR
    GOTO    FINISH_REACHED
    GOTO    MAIN_LOOP        ; B = 1 -> continua esperando

FINISH_REACHED
    MOVLW IDLE
    MOVWF STATE
    GOTO MAIN_LOOP
    
;-------------------------------------------------------
; Rotinas de rota��o do motor
;-------------------------------------------------------
ROTATE_RIGHT
    MOVLW   0xF1           ; 0x01 (0001)
    MOVWF   LATD
    CALL    Delay_5ms
    
    MOVLW   0xF4           ; 0x04 (0100)  
    MOVWF   LATD
    CALL    Delay_5ms
    
    MOVLW   0xF2           ; 0x02 (0010)
    MOVWF   LATD
    CALL    Delay_5ms
    
    MOVLW   0xF8           ; 0x08 (1000)
    MOVWF   LATD
    CALL    Delay_5ms
    RETURN

ROTATE_LEFT
   MOVLW   0x08
    MOVWF   LATD
    CALL    Delay_5ms
    
    MOVLW   0x02  
    MOVWF   LATD
    CALL    Delay_5ms
    
    MOVLW   0x04
    MOVWF   LATD
    CALL    Delay_5ms
    
    MOVLW   0x01
    MOVWF   LATD
    CALL    Delay_5ms
    
    RETURN
;=========================================================================
; C�lculo do n�mero de estouros do Timer1 (Fosc = 8 MHz, prescaler 1:8)
;=========================================================================
; Tcy = Fosc / 4 = 8 MHz / 4 = 2 MHz ? 1 ciclo = 0,5 us
; Ttick = Tcy * prescaler = 0,5 us * 8 = 4 us
; Tempo 1 overflow = 65536 * Ttick ? 0,262 s
; N� de estouros para 5 s = 5 / 0,262 ? 19 (Prefiro usar 20 para 5,24s)
;=========================================================================
;-------------------------------------------------------
; Rotina de delay de 5s
;-------------------------------------------------------
Delay_5s:
    CLRF    TICKS              ; zera contador
WAIT_LOOP:
    MOVF    TICKS, W
    XORLW   20                 ; Z=1 quando TICKS == 20
    BTFSS   STATUS, Z          ; se Z=0 -> executa GOTO WAIT_LOOP
    GOTO    WAIT_LOOP
    RETURN

;-------------------------------------------------------
; Rotina de delay para o motor
;-------------------------------------------------------
Delay_5ms:
    MOVLW   0x1F              ; Loop externo - valor reduzido
    MOVWF   0x40              ; Contador externo (usando endere�o 0x40)
Delay_Outer_5ms:
    MOVLW   0x4F              ; Loop interno - valor reduzido  
    MOVWF   0x41              ; Contador interno (usando endere�o 0x41)
Delay_Inner_5ms:
    NOP                       ; 1 ciclo
    NOP                       ; 1 ciclo
    DECFSZ  0x41, F           ; Decrementa contador interno
    GOTO    Delay_Inner_5ms   ; Continua loop interno
    DECFSZ  0x40, F           ; Decrementa contador externo
    GOTO    Delay_Outer_5ms   ; Continua loop externo
    RETURN

END