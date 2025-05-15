;===========================
; TURRET SLAVE - AVR ASM
;===========================
#define __SFR_OFFSET 0x00
#include "avr/io.h"

.global slave_main

slave_main:
    ; --- Enable pull-ups on SDA (PC4) and SCL (PC5) ---
    cbi DDRC, 4
    sbi PORTC, 4
    cbi DDRC, 5
    sbi PORTC, 5
    ; --- Set PB1 (Digital Pin 9) as output for servo ---
    sbi DDRB, 1
    
    ; --- Setup Timer0 for precise servo pulse timing ---
    ldi r16, (1<<WGM01)    ; CTC mode
    out TCCR0A, r16
    clr r16
    out TCCR0B, r16        ; Timer stopped (will be configured during use)

    ; --- LED (PB5) on pin 13, Buzzer (PD0/PB0) on pin 8 ---
    sbi DDRB, 5            ; Configure LED pin as output (PB5, pin 13)
    sbi DDRB, 0            ; Configure buzzer pin as output (PB0, pin 8)
    cbi PORTB, 5           ; Turn off LED initially
    cbi PORTB, 0           ; Turn off buzzer initially

    ; --- Init I2C Slave 0x48 ---
    rcall I2C_slave_init

listen_loop:
    rcall I2C_listen
    rcall I2C_read
    mov r21, r27        ; save detection flag
    rcall I2C_read
    mov r20, r27        ; save angle
    cpi r21, 0          ; if flag==0, clear alert
    breq clear_alert

    ; Ensure angle is within valid limits (15-165)
    cpi r20, 15
    brlo clip_low
    cpi r20, 165
    brsh clip_high
    rjmp angle_ok

clip_low:
    ldi r20, 15
    rjmp angle_ok

clip_high:
    ldi r20, 165

angle_ok:
    mov r22, r20
    ; Use improved servo control method
    rcall send_servo_pulse
    
    ; Also update PWM registers for continuous operation
    rcall angle_to_pwm
    sts OCR1AL, r24
    sts OCR1AH, r25

    ; Alert sequence when target is detected
    ; Turn on LED
    sbi PORTB, 5     ; Turn on LED (PB5, pin 13)
    
    ; Buzzer alarm pattern (3 short beeps)
    ldi r23, 3       ; 3 beeps
buzzer_loop:
    sbi PORTB, 0     ; Turn on buzzer (PB0, pin 8)
    rcall delay_50ms ; Beep for 50ms
    cbi PORTB, 0     ; Turn off buzzer
    rcall delay_50ms ; Pause for 50ms
    dec r23
    brne buzzer_loop
    
    ; Auto-clear LED after alert
    cbi PORTB, 5     ; Turn off LED
    
    rjmp listen_loop

clear_alert:
    cbi PORTB, 5     ; Turn off LED (PB5, pin 13)
    cbi PORTB, 0     ; Ensure buzzer is off (PB0, pin 8)
    rjmp listen_loop

;===========================
; Improved Servo Control Functions
;===========================
; Rotate servo based on angle in r24
rotate_turret_servo:
    ldi r20, 10          ; Send 10 pulses for stability
loop_servo_pulse:
    sbi PORTB, 1         ; HIGH on PB1 (Digital Pin 9)
    rcall turret_delay_timer0   ; Delay based on angle (r24)
    cbi PORTB, 1         ; LOW on PB1
    rcall delay_20ms     ; Delay between pulses
    dec r20
    brne loop_servo_pulse
    ret

; Delay based on pulse width in r24
turret_delay_timer0:
    clr r21
    out TCNT0, r21
    mov r21, r24
    out OCR0A, r21
    ldi r21, 0b00001100     ; Timer0 CTC mode, prescaler 256
    out TCCR0B, r21

turret_wait_ctc:
    in r21, TIFR0
    sbrs r21, OCF0A
    rjmp turret_wait_ctc
    clr r21
    out TCCR0B, r21
    ldi r21, (1 << OCF0A)
    out TIFR0, r21
    ret

;===========================
; I2C Slave Functions
;===========================

; Initialize I2C slave with address 0x48
I2C_slave_init:
    ; Set slave address to 0x48 with general call recognition enabled
    ldi r16, 0x48 << 1 | 1
    sts TWAR, r16
    ; Set prescaler to 1
    ldi r16, 0
    sts TWSR, r16
    ; Enable TWI and acknowledge
    ldi r16, (1<<TWEN)|(1<<TWEA)|(1<<TWINT)
    sts TWCR, r16
    ret

; Listen for I2C transactions addressed to this device
I2C_listen:
    ; Enable acknowledge and clear interrupt flag to start listening
    ldi r16, (1<<TWEN)|(1<<TWEA)|(1<<TWINT)
    sts TWCR, r16
I2C_listen_wait:
    ; Wait for TWI interrupt flag
    lds r16, TWCR
    sbrs r16, TWINT
    rjmp I2C_listen_wait
    
    ; Check if it's an address match
    lds r16, TWSR
    andi r16, 0xF8
    cpi r16, 0x60    ; SLA+W received, ACK sent
    breq I2C_listen_addr_match
    cpi r16, 0x70    ; General call address received, ACK sent
    breq I2C_listen_addr_match
    
    ; If not an address match, continue listening
    rjmp I2C_listen
    
I2C_listen_addr_match:
    ret

; Read a byte from I2C master (stores in r27)
I2C_read:
    ; Clear interrupt flag to receive data
    ldi r16, (1<<TWEN)|(1<<TWINT)|(1<<TWEA)
    sts TWCR, r16
    sts TWCR, r16
I2C_read_wait:
    ; Wait for data
    lds r16, TWCR
    sbrs r16, TWINT
    rjmp I2C_read_wait
    
    ; Check status and get received byte
    lds r16, TWSR
    andi r16, 0xF8
    cpi r16, 0x80    ; Data received, ACK sent
    breq I2C_read_ok
    cpi r16, 0x90    ; Data received after general call, ACK sent
    breq I2C_read_ok
    cpi r16, 0xA0    ; STOP or REPEATED START received
    breq I2C_read_stop
    
    ; Handle unexpected status - restart listening
    ldi r27, 0       ; Return 0 on error
    rjmp I2C_read_done
    
I2C_read_ok:
    lds r27, TWDR    ; Load received byte into r27
    rjmp I2C_read_done
    
I2C_read_stop:
    ldi r27, 0       ; Return 0 on STOP condition
    
I2C_read_done:
    ; If STOP received, reset to listen mode
    cpi r16, 0xA0
    brne I2C_read_exit
    
    ; Re-enable listening after receiving STOP
    ldi r16, (1<<TWEN)|(1<<TWEA)|(1<<TWINT)
    sts TWCR, r16
    
I2C_read_exit:
    ret

;===========================
; Improved Servo Control Functions
;===========================

; Send multiple servo pulses for better reliability
send_servo_pulse:
    ; Save the angle in r24
    mov r24, r22
    
    ; Send multiple pulses for better response
    ldi r20, 5       ; Number of pulses
servo_pulse_loop:
    rcall send_single_pulse
    rcall delay_20ms
    dec r20
    brne servo_pulse_loop
    ret

; Send a single pulse with timing based on r24
send_single_pulse:
    ; Configure Timer0 for precise pulse timing
    clr r21
    out TCNT0, r21
    mov r21, r24
    out OCR0A, r21
    
    ; Start the pulse
    sbi PORTB, 1     ; HIGH on PB1
    
    ; Configure timer - CTC mode, prescaler 256
    ldi r21, 0b00001100
    out TCCR0B, r21
    
wait_for_pulse_end:
    in r21, TIFR0
    sbrs r21, OCF0A
    rjmp wait_for_pulse_end
    
    ; End the pulse and clean up
    cbi PORTB, 1     ; LOW on PB1
    clr r21
    out TCCR0B, r21
    ldi r21, (1 << OCF0A)
    out TIFR0, r21
    ret

; Delay of 20ms for servo timing
delay_20ms:
    ldi r21, 200
d20_1: ldi r22, 200
d20_2: ldi r23, 2
d20_3: dec r23
    brne d20_3
    dec r22
    brne d20_2
    dec r21
    brne d20_1
    ret

;===========================
; Angle to PWM Conversion
;===========================
; Converts angle in r22 to PWM value in r24:r25
angle_to_pwm:
    ; Assuming a linear mapping:
    ; Angle 15 -> PWM 1000 (1ms pulse)
    ; Angle 165 -> PWM 2000 (2ms pulse)
    ; PWM = 1000 + ((angle - 15) * (2000 - 1000) / (165 - 15))
    
    ldi r23, 1000 / 256    ; Base PWM high byte
    ldi r24, 1000 % 256    ; Base PWM low byte
    mov r25, r22           ; Copy angle to r25
    subi r25, 15           ; Subtract minimum angle (15)
    ldi r26, 6             ; Scale factor ((2000 - 1000) / (165 - 15) = 6)
    mul r25, r26           ; Multiply (angle - 15) by scale factor
    add r24, r0            ; Add low byte of result to PWM low byte
    adc r23, r1            ; Add high byte of result to PWM high byte
    clr r1                 ; Clear r1 after multiplication
    ret

;===========================
; Delay Functions
;===========================

; Approximately 50ms delay at 16MHz
delay_50ms:
    ldi r18, 250
delay_outer:
    ldi r17, 250
delay_inner:
    nop
    nop
    dec r17
    brne delay_inner
    dec r18
    brne delay_outer
    ret