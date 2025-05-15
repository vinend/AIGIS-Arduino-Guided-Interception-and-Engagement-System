#define __SFR_OFFSET 0x00
#include <avr/io.h>

.global radar_main
.global currentAngle
.global radarStopped

.section .bss
radarStopped:
  .byte 1
currentAngle:
  .byte 1

.section .text
radar_main:
    ; === Init UART ===
    clr r16
    sts UCSR0A, r16
    sts UBRR0H, r16
    ldi r16, 103              ; Baud 9600 @ 16MHz
    sts UBRR0L, r16
    ldi r16, (1<<TXEN0)
    sts UCSR0B, r16
    ldi r16, (1<<UCSZ01)|(1<<UCSZ00)
    sts UCSR0C, r16

    ; === Init Servo PB2 (Digital Pin 10) ===
    ldi r16, (1<<DDB2)        ; Explicitly set pin 10 as output
    out DDRB, r16

    ; === Init IR Sensor PD2 (Digital Pin 2) ===
    cbi DDRD, 2               ; Set PD2 as input
    sbi PORTD, 2              ; Enable pull-up resistor
    
    ; === Timer0 CTC Setup ===
    ldi r16, (1<<WGM01)       ; CTC mode
    out TCCR0A, r16
    clr r16
    out TCCR0B, r16

    ; === Initialize radarStopped to 0 ===
    clr r16
    sts radarStopped, r16
    
    ; === Center servo to confirm it's working ===
    ldi r24, 90              ; Center position
    sts currentAngle, r24
    ldi r20, 20              ; Send 20 pulses for initial positioning
center_loop:
    sbi PORTB, 2             ; HIGH on PB2
    rcall pwm_delay
    cbi PORTB, 2             ; LOW on PB2
    rcall delay_20ms
    dec r20
    brne center_loop
    
    ; === Print startup message ===
    ldi ZL, lo8(msg_start)
    ldi ZH, hi8(msg_start)
    rcall print_uart

    ; --- Init I2C Master ---
    rcall I2C_init

main_loop:
    rjmp sweep_up

sweep_up:
    ldi r24, 0             ; Start at 0°
up_loop:
    in r21, PIND           ; Read IR sensor
    sbrs r21, 2            ; If PD2==0 (object), branch to detect
        rjmp detect_up
    ; Normal sweep motion when no object
    sts currentAngle, r24  ; store for ISR
    rcall rotate_servo     ; move servo
    inc r24                ; next angle
    cpi r24, 180
    brlo up_loop           ; continue until <180
    rjmp sweep_down        ; reverse direction after reaching 180

detect_up:
    rcall object_detect    ; handle object detection
    rjmp up_loop           ; continue sweep after handling

sweep_down:
    ldi r24, 180           ; Start at 180°
down_loop:
    in r21, PIND
    sbrs r21, 2            ; If PD2==0 (object), branch to detect
        rjmp detect_down
    ; Normal sweep motion when no object
    sts currentAngle, r24
    rcall rotate_servo
    dec r24                ; previous angle
    brne down_loop         ; continue until >0
    rjmp sweep_up          ; reverse direction after reaching 0

detect_down:
    rcall object_detect
    rjmp down_loop

; ---------------------------------------
; PWM Pulse Width: r24 (OCR0A units)
pwm_delay:
    push r21              ; Save r21
    clr r21
    out TCNT0, r21
    out OCR0A, r21        ; clear match
    out OCR0A, r24
    ldi r21, 0b00001100   ; prescaler 64
    out TCCR0B, r21
wait_pwm:
    in r21, TIFR0
    sbrs r21, OCF0A
    rjmp wait_pwm
    clr r21
    out TCCR0B, r21
    ldi r21, (1<<OCF0A)
    out TIFR0, r21
    pop r21               ; Restore r21
    ret

; ---------------------------------------
; Rotate servo at position in r24
rotate_servo:
    ldi r20, 10         ; number of pulses for smooth motion
rotate_loop:
    sbi PORTB, 2       ; drive PB2 high
    rcall pwm_delay    ; generate pulse width based on r24
    cbi PORTB, 2       ; drive PB2 low
    rcall delay_20ms   ; pause between pulses
    dec r20
    brne rotate_loop
    ret

; ---------------------------------------
; Delay for (20ms - r24 pulse duration)
delay_rest_20ms:
    push r26
    push r27
    ldi r26, lo8(500)
    ldi r27, hi8(500)
    clr r1                ; Ensure carry subtraction base is zero
    sub r26, r24          ; Subtract low byte
    sbc r27, r1           ; Subtract high byte with carry

    ; Multiply (500 - r24) × 4
    lsl r26
    rol r27
    lsl r26
    rol r27

delay_loop:
    sbiw r26, 1
    brne delay_loop
    pop r27
    pop r26
    ret

; ---------------------------------------
; Delay for ~20ms
delay_20ms:
    push r26
    push r27
    ldi r26, lo8(1000)
    ldi r27, hi8(1000)
delay_20ms_loop:
    sbiw r26, 1
    brne delay_20ms_loop
    pop r27
    pop r26
    ret

; ---------------------------------------
; Delay for ~50ms
delay_50ms:
    push r26
    push r27
    ldi r26, lo8(30000)
    ldi r27, hi8(30000)
delay_50ms_loop:
    sbiw r26, 1
    brne delay_50ms_loop
    pop r27
    pop r26
    ret

; ---------------------------------------
; UART Print from progmem Z
print_uart:
    lpm r18, Z+
    cpi r18, 0
    breq done_uart
wait_uart:
    lds r17, UCSR0A
    sbrs r17, UDRE0
    rjmp wait_uart
    sts UDR0, r18
    rjmp print_uart
done_uart:
    ret

; ---------------------------------------
; Handle IR detection: send I2C flag, wait until sensor releases
object_detect:
    rcall I2C_start
    ldi r27, 0b10010000    ; SLA+W (0x48<<1)
    rcall I2C_write
    ldi r27, 1             ; Object detected flag
    rcall I2C_write
    lds r27, currentAngle  ; Load last servo angle
    rcall I2C_write        ; Send servo angle
    rcall I2C_stop
wait_no_obj:
    in r21, PIND
    sbrs r21, 2            ; Skip if PD2==1 (no object)
    rjmp wait_no_obj       ; Repeat while object present
    ret

; ---------------------------------------
; I2C Master Functions
I2C_init:
    ; Set SCL frequency ~100kHz (16MHz/(16+2*TWBR))
    ldi r16, 72
    sts TWBR, r16
    ; Prescaler = 1
    ldi r16, 0
    sts TWSR, r16
    ; Enable TWI
    ldi r16, (1<<TWEN)
    sts TWCR, r16
    ret

I2C_start:
    ; Send START
    ldi r16, (1<<TWINT)|(1<<TWSTA)|(1<<TWEN)
    sts TWCR, r16
I2C_start_wait:
    lds r16, TWCR
    sbrs r16, TWINT
    rjmp I2C_start_wait
    ; Check for START transmitted (0x08)
    lds r16, TWSR
    andi r16, 0xF8
    cpi r16, 0x08
    brne I2C_start
    ret

I2C_write:
    ; Write byte in r27
    sts TWDR, r27
    ldi r16, (1<<TWINT)|(1<<TWEN)
    sts TWCR, r16
I2C_write_wait:
    lds r16, TWCR
    sbrs r16, TWINT
    rjmp I2C_write_wait
    ret

I2C_stop:
    ; Send STOP
    ldi r16, (1<<TWINT)|(1<<TWSTO)|(1<<TWEN)
    sts TWCR, r16
    ret

; ---------------------------------------
.section .progmem.data
; OCR0A values corresponding to these angles:
; 250 = ~0°, 480 = ~180°
scan_pos:
    ; Sweep from left to right
    .word 250, 265, 280, 295, 310, 325  ; 0° to ~60°
    .word 340, 355, 370, 385, 400, 415  ; ~60° to ~120°
    .word 430, 445, 460, 480            ; ~120° to 180°
    ; Return from right to left
    .word 460, 445, 430, 415, 400, 385  ; 180° to ~120°
    .word 370, 355, 340, 325, 310, 295  ; ~120° to ~60°
    .word 280, 265, 250                 ; ~60° to 0°

msg_scan:
    .ascii "Scanning...\n"
    .byte 0

msg_start:
    .ascii "Radar system starting...\n"
    .byte 0

msg_patrol:
    .ascii "Starting new patrol cycle...\n"
    .byte 0