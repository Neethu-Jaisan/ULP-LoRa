#include <BME280.h>  // Include the BME280 Sensor library
/*******************************************************************************************************************
** Declare all program constants                                                                                  **
*******************************************************************************************************************/
const uint32_t SERIAL_SPEED = 115200; ///< Default   baud rate for Serial I/O

/*! @brief The pin used for slave-select can be freely chosen from the digital pins available. This is the default
*          pin used on an Arduino MEGA2560 
*/
const uint8_t  SPI_CS_PIN = 10;
/*******************************************************************************************************************
** Declare global variables and instantiate classes                                                               **
*******************************************************************************************************************/
/*! Instantiate the class */
BME280_Class   BME280; ///< Create an instance of the BME280 class

/*!
* @brief     This converts a pressure measurement into a height in meters
* @details   The corrected sea-level pressure can be passed into the function if it is know, otherwise the standard
*            atmospheric pressure of 1013.25hPa is used (see https://en.wikipedia.org/wiki/Atmospheric_pressure
* @param[in] seaLevel Sea-Level pressure in millibars
* @return    floating point altitude in meters.
*/
float altitude(const float seaLevel = 1013.25)
{
  static float Altitude;
  int32_t temp, hum, press;
  BME280.getSensorData(temp, hum, press); // Get the most recent values from the device
  Altitude = 44330.0*(1.0 - pow(((float)press / 100.0) / seaLevel, 0.1903)); // Convert into altitude in meters
  return(Altitude);
} // of method altitude()

/*!
    @brief    Arduino method called once at startup to initialize the system
    @details  This is an Arduino IDE method which is called first upon boot or restart. It is only called one time
              and then control goes to the main "loop()" method, from which control never returns
    @return   void
*/
void setup() 
{
  Serial.begin(SERIAL_SPEED);
  #ifdef  __AVR_ATmega32U4__ // If this is a 32U4 processor, then wait 3 seconds to initialize USB
    delay(3000);
  #endif
  Serial.println(F("Starting Hardware SPIDemo example program for BME280"));
  Serial.print(F("- Initializing BME280 sensor\n"));

  while (!BME280.begin(SPI_CS_PIN))
  {
    Serial.println(F("-  Unable to find BME280. Waiting 3 seconds."));
    delay(3000);
  } // of loop until device is located
  BME280.mode(SleepMode);
  Serial.print(F("- Sensor detected in operating mode \""));
  Serial.print(BME280.mode());
  Serial.println(F("\"."));
  if(BME280.mode()==0) 
  {
    Serial.print(F("- Turning sensor to normal mode, mode is now \""));
    Serial.print(BME280.mode(NormalMode));
    Serial.println("\"");
  } // of if-then we have sleep mode
  Serial.println(F("- Setting 16x oversampling for all sensors"));
  BME280.setOversampling(TemperatureSensor,Oversample16);
  BME280.setOversampling(HumiditySensor,   Oversample16);
  BME280.setOversampling(PressureSensor,   Oversample16);
  Serial.println(F("- Setting IIR filter to maximum value of 16 samples"));
  BME280.iirFilter(IIR16); // Use enumerated type values
  Serial.println(F("- Setting time between measurements to 1 second"));
  BME280.inactiveTime(inactive1000ms);
  Serial.print(F("- Each measurement cycle will take "));
  Serial.print(BME280.measurementTime(MaximumMeasure)/1000); // returns microseconds
  Serial.println(F("ms.\n\n"));
} // of method setup()

/*!
    @brief    Arduino method for the main program loop
    @details  This is the main program for the Arduino IDE, it is an infinite loop and keeps on repeating.
    @return   void */
void loop()
{
  static uint8_t loopCounter = 0;                      // iteration counter
  static int32_t temperature, humidity, pressure;      // Store readings
  BME280.getSensorData(temperature,humidity,pressure); // Get most recent readings
  Serial.print(F("Temperature: "));
  Serial.print(temperature/100.0); // Temperature in deci-degrees
  Serial.print(F("C "));
  if (BME280.getOversampling(HumiditySensor)!=0)
  {
    Serial.print(F("Humidity: "));
    Serial.print(humidity/100.0); // Humidity in deci-percent
    Serial.print(F("% "));
  } // of if-then humidity sensing turned off
  Serial.print(F("Pressure: "));
  Serial.print(pressure/100.0); // Pressure in Pascals
  Serial.print(F("hPa Altitude: "));
  Serial.print(altitude());
  Serial.println(F("m"));
  delay(1000);
  if (++loopCounter%10==0) // Every 10th reading
  {
    Serial.print(F("\n- Turning "));
    if (BME280.getOversampling(HumiditySensor)==0)
    {
      BME280.setOversampling(HumiditySensor,Oversample16); // Turn humidity sensing on
      Serial.print(F("ON"));
    } else  
    {
      BME280.setOversampling(HumiditySensor,SensorOff); // No longer interested in humidity
      Serial.print(F("OFF"));
    } // of if-then-else humidity sensing turned off
    Serial.println(F(" humidity sensing"));
    Serial.print(F("- Each measurement cycle will now take "));
    Serial.print(BME280.measurementTime(MaximumMeasure)/1000.0); // returns microseconds
    Serial.println(F("ms.\n"));
  } // of if-then first loop iteration
} // of method loop()
