// ESP32 6-bit DAC controller with analog measurement
// Pins 2-7 used for 6-bit DAC output (D2 = LSB, D7 = MSB)
// Analog inputs on pins 34 and 35

// DAC output pins (must be consecutive starting from 2 for port manipulation)
#define DAC_START_PIN 2
#define DAC_PIN_COUNT 6

// Analog input pins
#define ANALOG_PIN1 34
#define ANALOG_PIN2 35

// Measurement parameters
#define SETTLING_TIME_MS 10  // Time to wait for signal to stabilize
#define MEASUREMENT_DELAY_MS 50  // Time between measurements

// For ESP32, we need to use GPIO_OUT_REG for direct port manipulation
#define DAC_PORT_MASK 0xFC  // Binary 11111100 - pins 2-7

void setup() {
  Serial.begin(115200);
  
  // Set DAC pins as outputs
  for (int i = 0; i < DAC_PIN_COUNT; i++) {
    pinMode(DAC_START_PIN + i, OUTPUT);
  }
  
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
  
  // Calculate approximate DAC voltage (assuming 3.3V reference)
  float dacVoltage = (dacValue & 0x3F) * (3.3 / 63.0);  // 6-bit resolution
  
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

// Set DAC value using direct port manipulation
void setDacValue(uint8_t value) {
  // Mask to only use 6 bits
  value &= 0x3F;
  
  // For ESP32, we need to manipulate the GPIO_OUT_REG register
  // First read current state of all pins
  uint32_t portState = GPIO.out;
  
  // Clear the bits we want to change (pins 2-7)
  portState &= ~DAC_PORT_MASK;
  
  // Set the new bits (shift left by 2 because we start at pin 2)
  portState |= ((uint32_t)value << DAC_START_PIN);
  
  // Write to the register
  GPIO.out = portState;
}
