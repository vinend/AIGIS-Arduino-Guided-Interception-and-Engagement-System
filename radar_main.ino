#include <Wire.h>
#include <Servo.h>

extern "C" void radar_main();             // Assembly logic
extern "C" volatile uint8_t radarStopped; // Shared with Assembly
extern "C" volatile uint8_t currentAngle; // Shared with Assembly

#define IR_PIN 2      // IR sensor on digital pin 2 (INT0)
#define SERVO_PIN 10  // Servo on digital pin 10 (PB2)
#define SLAVE_ADDR 0x48

Servo testServo;      // For direct control if needed
volatile unsigned long lastTriggerTime = 0;
volatile unsigned long resumeTime = 0;  // New: schedule resume time after detection

void setup() {
  Serial.begin(9600);
  delay(1000);  // Wait for serial to connect
  
  Serial.println(F("Radar system initializing..."));
  
  // Initialize I2C as master
  Wire.begin();
  
  // Direct port manipulation for IR sensor with pull-up
  DDRD &= ~(1 << DDD2);   // Clear the bit (input)
  PORTD |= (1 << PORTD2); // Set the bit (pullup)
  
  // Also set via Arduino API
  pinMode(IR_PIN, INPUT_PULLUP);
  
  // Test the IR sensor
  Serial.print(F("Initial IR state: "));
  Serial.println(digitalRead(IR_PIN) ? "HIGH (no object)" : "LOW (object detected)");
  
  // Also set up the servo pin just to be sure
  pinMode(SERVO_PIN, OUTPUT);
  
  // Test servo directly
  testServo.attach(SERVO_PIN);
  testServo.write(0);     // Full left
  delay(500);
  testServo.write(90);    // Center position
  delay(500);
  testServo.write(180);   // Full right
  delay(500);
  testServo.write(90);    // Back to center
  delay(500);
  testServo.detach();     // Let assembly take over
  
  // Make sure radarStopped is clear
  radarStopped = 0;
  
  // Enable external interrupt INT0 (Digital pin 2)
  // Using direct register manipulation for more reliable operation
  cli(); // Disable interrupts temporarily
  
  // Configure INT0 for falling edge (CHANGE mode didn't work reliably)
  EICRA |= (1 << ISC01);   // Falling edge of INT0 generates interrupt
  EICRA &= ~(1 << ISC00);
  EIMSK |= (1 << INT0);    // Enable INT0
  
  // Also use Arduino's API as backup
  attachInterrupt(digitalPinToInterrupt(IR_PIN), IR_ISR, FALLING);
  
  // Initialize control variables
  radarStopped = 0;
  currentAngle = 90;  // Start at middle position
  
  sei(); // Re-enable interrupts
  
  Serial.println(F("Starting radar main..."));
  radar_main();  // Starts the main servo logic from Assembly
}

void loop() {
  // Periodic checks to ensure everything is working
  static unsigned long lastBlink = 0;
  static unsigned long lastIRCheck = 0;
  
  // Heartbeat every 5 seconds
  if (millis() - lastBlink > 5000) {
    lastBlink = millis();
    
    // Check system status
    Serial.print(F("System alive, radarStopped="));
    Serial.print(radarStopped);
    Serial.print(F(", IR="));
    Serial.println(digitalRead(IR_PIN) ? "HIGH (no object)" : "LOW (object detected)");
    
    // If radarStopped is stuck, reset it
    if (radarStopped == 1 && digitalRead(IR_PIN) == HIGH) {
      Serial.println(F("⚠ radarStopped stuck with no object! Resetting..."));
      radarStopped = 0;
    }
  }
  
  // Check IR sensor directly every second
  if (millis() - lastIRCheck > 1000) {
    lastIRCheck = millis();
    
    // Direct port reading for maximum reliability
    bool irDetected = ((PIND & (1 << PIND2)) == 0);
    
    if (irDetected) {
      Serial.println(F("IR LOW detected directly in loop"));
      
      // If radar isn't stopped but there's an object, trigger detection manually
      if (radarStopped == 0) {
        Serial.println(F("Forcing IR detection..."));
        IR_ISR();
      }
    }
  }

  // Auto-resume radar after scheduled delay
  if (radarStopped == 1 && resumeTime != 0 && millis() > resumeTime) {
    // Ensure sensor clear before resuming
    if ((PIND & (1 << PIND2)) != 0) {
      Serial.println(F("Resuming radar scan..."));
      radarStopped = 0;
      resumeTime = 0;
    }
  }
}

// Interrupt handler for INT0 (IR sensor)
// This gets called when an object is detected by the IR sensor
void IR_ISR() {
  // Disable further interrupts while handling this one
  cli();
  
  Serial.println(F("🚨 IR INTERRUPT TRIGGERED!"));
  
  // Get current time for debounce
  unsigned long now = millis();
  
  // Debounce: Ignore if triggered too soon after previous one
  if (now - lastTriggerTime < 300) {
    sei();
    return;
  }
  
  lastTriggerTime = now;
  
  // Double-check the pin state using direct port reading for reliability
  // Low means object detected
  if ((PIND & (1 << PIND2)) != 0) {
    Serial.println(F("False trigger - no object present"));
    sei();
    return;
  }
  
  Serial.println(F("IR TRIGGERED - Target Detected!"));
  Serial.print(F("Current angle: "));
  Serial.println(currentAngle);

  // Stop the radar
  radarStopped = 1;
  resumeTime = now + 3000;  // schedule resume after 3 seconds
  
  // Send current angle to turret via I2C
  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write(currentAngle);
  byte result = Wire.endTransmission();
  
  if (result == 0) {
    Serial.print(F("Angle sent to turret: "));
    Serial.println(currentAngle);
  } else {
    Serial.print(F("I2C error: "));
    Serial.println(result);
  }

  sei();  // re-enable interrupts quickly
}