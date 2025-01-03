#include <Arduino.h>
#include <Wire.h>
#include <BMP180TwoWire.h>
#define I2C_ADDRESS 0x77
// Create an BMP180 object using the I2C interface
BMP180TwoWire bmp180(&Wire, I2C_ADDRESS);
int LED = 7; 
int LED2 = 8;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  pinMode(LED, OUTPUT); 
  pinMode(LED2, OUTPUT);
  // Wait for serial connection to open (only necessary on some boards)
  while (!Serial);
  Wire.begin();
  // Initialize the BMP180 sensor
  if (!bmp180.begin()) {
    Serial.println("begin() failed. Check your BMP180 Interface and I2C Address.");
    while (1);  // Halt if initialization fails
  }
  // Reset sensor to default parameters
  bmp180.resetToDefaults();
  // Enable ultra-high resolution mode for pressure measurements
  bmp180.setSamplingMode(BMP180MI::MODE_UHR);
}
void loop() {
  delay(1000);  // Delay between measurements
  // Start a temperature measurement
  if (!bmp180.measureTemperature()) {
    Serial.println("Could not start temperature measurement. Is a measurement already running?");
    return;
  }
  // Wait for the measurement to finish
  do {
    delay(100);
  } while (!bmp180.hasValue());
  // Display temperature
  Serial.print("Temperature: ");
  Serial.print(bmp180.getTemperature());
  Serial.println(" degC");
  // Start a pressure measurement (after temperature)
  if (!bmp180.measurePressure()) {
    Serial.println("Could not start pressure measurement. Is a measurement already running?");
    return;
  }

  // Wait for the measurement to finish
  do {
    delay(100);
  } while (!bmp180.hasValue());

  // Display pressure
  Serial.print("Pressure: ");
  Serial.print(bmp180.getPressure());
  Serial.println(" Pa");

  // Control LEDs based on pressure and temperature
  if (bmp180.getPressure() >= 100500.00) {
    digitalWrite(LED, HIGH);  // Turn on LED1 if pressure >= 100500 Pa
  } else {
    digitalWrite(LED, LOW);   // Turn off LED1 if pressure < 100500 Pa
  }

  if (bmp180.getTemperature() >= 27.00) {
    digitalWrite(LED2, HIGH);  // Turn on LED2 if temperature >= 27°C
  } else {
    digitalWrite(LED2, LOW);   // Turn off LED2 if temperature < 27°C
  }
}


