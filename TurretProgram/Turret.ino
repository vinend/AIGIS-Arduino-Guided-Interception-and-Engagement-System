// Declare the external assembly function
extern "C" void slave_main();

void setup() {
  // Initialize serial for debugging
  Serial.begin(9600);
  Serial.println("Turret Slave starting...");
  
  // Set up pins for I2C (A4=SDA, A5=SCL on most Arduinos)
  // This is just for explicit hardware initialization
  pinMode(A4, INPUT_PULLUP);  // SDA
  pinMode(A5, INPUT_PULLUP);  // SCL
  
  // Set up pins used in assembly code
  pinMode(9, OUTPUT);  // OC1A (PB1) for servo
  pinMode(13, OUTPUT); // LED (PB5)
  pinMode(8, OUTPUT);  // Buzzer (PB0, digital pin 8)
  
  delay(1000); // Give time for devices to stabilize
  Serial.println("Calling assembly slave_main...");
  
  // Call the assembly main function
  slave_main();
}

void loop() {
  // Nothing to do here, assembly code has its own loop
}
