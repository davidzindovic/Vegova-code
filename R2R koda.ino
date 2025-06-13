// Arduino Uno 6-bit DAC controller with analog measurement
// Pins 2-7 used for 6-bit DAC output (D2 = LSB, D7 = MSB)
// Analog inputs on pins A0 and A1

// DAC output pins (must be PORTD pins 2-7 on Uno)
#define DAC_START_PIN 2
#define DAC_PIN_COUNT 6

// Analog input pins
#define ANALOG_PIN1 A0
#define ANALOG_PIN2 A1

// Measurement parameters
#define SETTLING_TIME_MS 10  // Time to wait for signal to stabilize
#define MEASUREMENT_DELAY_MS 50  // Time between measurements

void setup() {
  Serial.begin(115200);
  
  // Set DAC pins as outputs (pins 2-7 on PORTD)
  DDRD |= 0xFC;  // Binary 11111100 - pins 2-7 as outputs
  
  // Print CSV header
  Serial.println("DAC Voltage,Analog1,Analog2");
}

void loop() {
  static uint8_t dacValue = 0;
  
  // Update DAC output using direct port manipulation
  setDacValue(dacValue);
  
  // Wait for signal to stabilize
  delay(SETTLING_TIME_MS);
  
  // Read analog inputs
  int analog1 = analogRead(ANALOG_PIN1);
  int analog2 = analogRead(ANALOG_PIN2);
  
  // Calculate approximate DAC voltage (assuming 5V reference)
  float dacVoltage = (dacValue & 0x3F) * (5.0 / 63.0);  // 6-bit resolution
  
  // Output data in CSV format
  Serial.print(dacVoltage, 4);
  Serial.print(",");
  Serial.print(analog1);
  Serial.print(",");
  Serial.println(analog2);
  
  // Increment DAC value (6-bit, but we'll let it roll over to simulate 8-bit steps)
  dacValue += 4;  // Step by 4 to simulate 8-bit behavior (256/64=4)
  
  // Delay between measurements
  delay(MEASUREMENT_DELAY_MS);
}

// Set DAC value using direct port manipulation (PORTD on Uno)
void setDacValue(uint8_t value) {
  // Mask to only use 6 bits and shift to align with PORTD (pins 2-7)
  value &= 0x3F;
  
  // PORTD contains pins 0-7. We need to:
  // 1. Preserve the state of pins 0-1 (RX/TX - don't touch these!)
  // 2. Set pins 2-7 according to our value
  
  // Read current PORTD state
  uint8_t portState = PORTD;
  
  // Clear bits 2-7 (mask 0x03 keeps bits 0-1)
  portState &= 0x03;
  
  // Set the new bits (shift left by 2 to align with PORTD)
  portState |= (value << 2);
  
  // Write to the register
  PORTD = portState;
}
