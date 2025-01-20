

#include "DHT.h"

#include <GP2YDustSensor.h>

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#include "LowPower.h"

#define DHTPIN A2
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

#define SHARP_LED_PIN A1
#define SHARP_VO_PIN A0

#define SLEEP
#define battery_pin          A3
#define Sample_rate          10

#ifdef SLEEP
bool next = false;
#endif

GP2YDustSensor dustSensor(GP2YDustSensorType::GP2Y1010AU0F, SHARP_LED_PIN, SHARP_VO_PIN);


#ifdef COMPILE_REGRESSION_TEST
# define CFG_in866 1
#else
# warning "You must replace the values marked FILLMEIN with real values from the TTN control panel!"
# define FILLMEIN (#dont edit this, edit the lines that use FILLMEIN)
#endif

// LoRaWAN NwkSKey, network session key
// This should be in big-endian (aka msb).

static const PROGMEM u1_t NWKSKEY[16] = {0x22, 0x4F, 0x0D, 0xDE, 0xE3, 0xE2, 0x37, 0x44, 0x10, 0xE4, 0xA0, 0xB5, 0x51, 0xBE, 0x43, 0x9A};

// LoRaWAN AppSKey, application session key
// This should also be in big-endian (aka msb).

static const u1_t PROGMEM APPSKEY[16] = {0xEE, 0xB5, 0xC6, 0x32, 0x13, 0xBA, 0x43, 0x1C, 0x47, 0x21, 0x2D, 0x98, 0xA1, 0xB5, 0x71, 0xEF};

// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
// The library converts the address to network byte order as needed, so this should be in big-endian (aka msb) too.

static const u4_t DEVADDR = 0x01c6aaf5 ; // <-- Change this address for every node!

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in arduino-lmic/project_config/lmic_project_config.h,
// otherwise the linker will complain).

void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

///
  uint16_t temperature=0;
  uint16_t humidity=0;
  uint16_t dust_density=0;
  uint16_t battery_voltage=0;
//}__attribute__((packed))appData_t;

static uint8_t mydata[8];
//appData_t appData = {0};
static osjob_t sendjob;
const unsigned long TX_INTERVAL = 60;

const lmic_pinmap lmic_pins = {
  .nss = 6,                       // chip select
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 5,                       // reset pin
  .dio = {2, 3, LMIC_UNUSED_PIN},
};


void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      break;
    /*
      || This event is defined but not used in the code. No
      || point in wasting codespace on it.
      ||
      || case EV_RFU1:
      ||     //////////////Serial.println(F("EV_RFU1"));
      ||     break;
    */
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.println(F("Received "));
        Serial.println(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
      // Schedule next transmission
#ifndef SLEEP
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
#else
      next = true;
#endif
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    /*
      || This event is defined but not used in the code. No
      || point in wasting codespace ohttps://www.thethingsnetwork.org/forum/t/arduino-sx1276-with-lmic-library-on-arduino-ide-sleep-mode-problem/54692n it.
      ||
      || case EV_SCAN_FOUND:
      ||    //////////////Serial.println(F("EV_SCAN_FOUND"));
      ||    break;
    */
    case EV_TXSTART:
      Serial.println(F("EV_TXSTART"));
      break;
    case EV_TXCANCELED:
      Serial.println(F("EV_TXCANCELED"));
      break;
    case EV_RXSTART:
      /* do not print anything -- it wrecks timing */
      break;
    case EV_JOIN_TXCOMPLETE:
      Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
      break;
    default:
      Serial.print(F("Unknown event: "));
      Serial.println((unsigned) ev);
      break;
  }
}

void do_send(osjob_t* j) {
 // appData = {0};

 
  batteryVoltage();
  delay(100);
  
  readDHT();
  delay(100);
  
  readDustSensor();
  delay(100);


  
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(2, (uint8_t*)&mydata, sizeof(mydata), 0);
    Serial.println(F("Packet queued"));
  }
}
void setup() {
  Serial.begin(9600);
  delay(100);     // per sample code on RF_95 test
  Serial.println(F("Starting"));

  dustSensor.begin();
  delay(500);

  dht.begin();
  delay(500);

#ifdef VCC_ENABLE
  // For Pinoccio Scout boards
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, HIGH);
  delay(1000);
#endif

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
#ifdef PROGMEM
  // On AVR, these values are stored in flash and only copied to RAM
  // once. Copy them to a temporary buffer here, LMIC_setSession will
  // copy them into a buffer of its own again.
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x13, DEVADDR, nwkskey, appskey);
#else
  // If not running an AVR with PROGMEM, just use the arrays directly
  LMIC_setSession (0x13, DEVADDR, NWKSKEY, APPSKEY);
#endif

#if defined(CFG_eu868)
  // Set up the channels used by the Things Network, which corresponds
  // to the defaults of most gateways. Without this, only three base
  // channels from the LoRaWAN specification are used, which certainly
  // works, so it is good for debugging, but can overload those
  // frequencies, so be sure to configure the full frequency range of
  // your network here (unless your network autoconfigures them).
  // Setting up channels should happen after LMIC_setSession, as that
  // configures the minimal channel set. The LMIC doesn't let you change
  // the three basic settings, but we show them here.
  //LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  //LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  //LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  //LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  //LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  //LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  //LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  //LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  //LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
  // TTN defines an additional channel at 869.525Mhz using SF9 for class B
  // devices' ping slots. LMIC does not have an easy way to define set this
  // frequency and support for class B is spotty and untested, so this
  // frequency is not configured here.
#elif defined(CFG_us915) || defined(CFG_au915)
  // NA-US and AU channels 0-71 are configured automatically
  // but only one group of 8 should (a subband) should be active
  // TTN recommends the second sub band, 1 in a zero based count.
  // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
  LMIC_selectSubBand(1);
#elif defined(CFG_as923)
  // Set up the channels used in your country. Only two are defined by default,
  // and they cannot be changed.  Use BAND_CENTI to indicate 1% duty cycle.
  // LMIC_setupChannel(0, 923200000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
  // LMIC_setupChannel(1, 923400000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);

  // ... extra definitions for channels 2..n here
#elif defined(CFG_kr920)
  // Set up the channels used in your country. Three are defined by default,
  // and they cannot be changed. Duty cycle doesn't matter, but is conventionally
  // BAND_MILLI.
  // LMIC_setupChannel(0, 922100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
  // LMIC_setupChannel(1, 922300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
  // LMIC_setupChannel(2, 922500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);

  // ... extra definitions for channels 3..n here.
#elif defined(CFG_in866)
  // Set up the channels used in your country. Three are defined by default,
  // and they cannot be changed. Duty cycle doesn't matter, but is conventionally
  // BAND_MILLI.
  LMIC_setupChannel(0, 865062500, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
  LMIC_setupChannel(1, 865402500, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
  LMIC_setupChannel(2, 865985000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
  LMIC_setupChannel(3, 865232500, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 866185000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 866385000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 866585000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 866785000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band

  // ... extra definitions for channels 3..n here.
#else
# error Region not supported
#endif

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF12;

  // Set data rate and transmit power for uplink
  LMIC_setDrTxpow(DR_SF12, 14);

  // Start job
  do_send(&sendjob);
}

void loop() {
#ifndef SLEEP

  os_runloop_once();

#else

  if (next == false) {

    os_runloop_once();

  } else {
  
// calculate the number of sleepcycles (8s) given the TX_INTERVAL
    int sleepcycles = (TX_INTERVAL / 8);
    Serial.println((String)"Enter sleeping for " + sleepcycles + "cycles of 8 seconds");
    delay(500);

    for (int i = 0; i < sleepcycles; i++) {
      // Enter power down state for 8 s with ADC and BOD module disabled
      LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);

    }
    Serial.println(F("Sleep complete"));
    next = false;

    do_send(&sendjob);
  }
#endif
}

void readDHT(void)
{
  Serial.println(F("DHTxx test!"));

  int humidity = (dht.readHumidity());
  int temperature = (dht.readTemperature())*100;
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }
//  appData.temperature = temperature;
//  appData.humidity = humidity;
  Serial.print(F("Humidity: "));
  Serial.print(humidity);
  Serial.print(F("%  Temperature: "));
  Serial.print(temperature);
  Serial.println(F("°C "));
  mydata[0]=highByte(temperature);
  mydata[1]=lowByte(temperature);
  mydata[2]=(humidity);
  
  
  
}

void readDustSensor(void)
{
  for (uint8_t i = 0; i < 100; i++)
  {
    dustSensor.getDustDensity();
    delay(5);
  }
  // Read dust density
  int dustDensity = (dustSensor.getDustDensity());
  int runningAverage = dustSensor.getRunningAverage();

  // Validate readings
  if (isnan(dustDensity) || isnan(runningAverage)) {
    Serial.println(F("Failed to read from dust sensor!"));
    return;
  }
  //appData.dust_density = dustDensity;
  // Print readings
  Serial.print(F("Dust density: "));
  Serial.print(dustDensity);
  Serial.print(F(" ug/m3; Running average: "));
  Serial.print(runningAverage);
  Serial.println(F(" ug/m3"));
  delay(1000);
  mydata[3]=highByte(runningAverage);
  mydata[4]=lowByte(runningAverage);
}


/**
 *
 * @brief Battery Voltage
 * @param none
 * @retval none
 *
 **/
void batteryVoltage(void)
{
//  int batteryVoltage = 0;
//  uint16_t Batv_analog = analogRead(battery_pin);
//  for (uint8_t i = 0; i < Sample_rate; i++)
//  {
//    batteryVoltage += (((analogRead(battery_pin))/1023) * 3.3);
//    delay(10);
//  }
//  batteryVoltage /= Sample_rate;
//  batteryVoltage = (map(batteryVoltage,0,33,0,60)/10);
//  //appData.battery_voltage = batteryVoltage;
//  Serial.print(F("Battery voltage : "));
//  Serial.println(batteryVoltage);
//  Serial.print(F("Battery voltage Analog : "));
//  Serial.println(Batv_analog);
float analogvalue = 0, battVolt = 0;
 
  for (byte  i = 0; i < 10; i++) {
    analogvalue += analogRead(A3);
    delay(5);
  }
  analogvalue = analogvalue / 10;
#ifdef DEBUG
  Serial.print("analogvalue= ");
  Serial.println(analogRead(A3));
#endif
  battVolt = ((analogvalue * 3.3) / 1024) * 2; //ADC voltage*Ref. Voltage/1024
  Serial.print("Voltage= ");
  Serial.print(battVolt);
  Serial.println("V");
 mydata[5]=(int(battVolt));
 //mydata[7]=lowByte(battVolt);
}
