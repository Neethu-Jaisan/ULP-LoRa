#include <DHT.h>
#include <GP2YDustSensor.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "LowPower.h"

#define DHTPIN A2
#define DHTTYPE DHT11  //Select according to the DHT Type
DHT dht(DHTPIN, DHTTYPE);

#define SHARP_LED_PIN A1
#define SHARP_VO_PIN A0

#define SLEEP
#define battery_pin          A3
#define Sample_rate          10
#define DHTSens              8  //Sleep pin connected to the vcc of DHTxx
#define DustSens             9  //Sleep pin connected to the vcc of DHTxx

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

static const PROGMEM u1_t NWKSKEY[16] = {0x22, 0x4f, 0x0d, 0xde, 0xe3, 0xe2, 0x37, 0x44, 0x10, 0xe4, 0xa0, 0xb5, 0x51, 0xbe, 0x43, 0x9a};

// LoRaWAN AppSKey, application session key
// This should also be in big-endian (aka msb).

static const u1_t PROGMEM APPSKEY[16] = { 0xee, 0xb5, 0xc6, 0x32, 0x13, 0xba, 0x43, 0x1c, 0x47, 0x21, 0x2d, 0x98, 0xa1, 0xb5, 0x71, 0xef };

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
  uint16_t t=0;
  uint16_t h=0;
  uint16_t battery_voltage=0;
//}attribute((packed))appData_t;

static uint8_t mydata[8];
//appData_t appData = {0};
static osjob_t sendjob;
const unsigned long TX_INTERVAL = 1500;

const lmic_pinmap lmic_pins = {
  .nss = 6,                       // chip select
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 5,                       // reset pin
  .dio = {2, 3, LMIC_UNUSED_PIN},
};


void onEvent (ev_t ev) {
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      break;
    case EV_BEACON_FOUND:
      break;
    case EV_BEACON_MISSED:
      break;
    case EV_BEACON_TRACKED:
      break;
    case EV_JOINING:
      break;
    case EV_JOINED:
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
      break;
    case EV_REJOIN_FAILED:
      break;
    case EV_TXCOMPLETE:
      if (LMIC.txrxFlags & TXRX_ACK)
      if (LMIC.dataLen) {
        
// data received in rx slot after tx
Serial.print(F("Data Received: "));
Serial.write(LMIC.frame + LMIC.dataBeg, LMIC.dataLen);
Serial.println();
      }
      // Schedule next transmission
#ifndef SLEEP
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
#else
      next = true;
#endif
      break;
    case EV_LOST_TSYNC:
      break;
    case EV_RESET:
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      break;
    case EV_LINK_DEAD:
    
      break;
    case EV_LINK_ALIVE:
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
      break;
    case EV_TXCANCELED:
      break;
    case EV_RXSTART:
      /* do not print anything -- it wrecks timing */
      break;
    case EV_JOIN_TXCOMPLETE:
      break;
    default:
      Serial.println((unsigned) ev);
      break;
  }
}

void do_send(osjob_t* j) {
 // appData = {0};

  digitalWrite(DHTSens, HIGH);
  delay(100);
  batteryVoltage();
  delay(100);
  
  readDHT();
  delay(100);
  
  digitalWrite(DustSens, HIGH);
  delay(100);
  readDustSensor();
  delay(100);
  
  if (LMIC.opmode & OP_TXRXPEND) {
  } else {
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(2, (uint8_t*)&mydata, sizeof(mydata), 0);
    Serial.println(F("Packet queued"));
  }
  digitalWrite(DHTSens, LOW);
  delay(100);
  digitalWrite(DustSens, LOW);
  delay(100);
}
void setup() {
  Serial.begin(9600);
  pinMode(DHTSens, OUTPUT);
  dustSensor.begin();
  delay(100);
  dht.begin();

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


#if defined(CFG_in866)
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
extern volatile unsigned long timer0_overflow_count;
  if (next==false) {

  os_runloop_once();

  } else {
  
// calculate the number of sleepcycles (8s) given the TX_INTERVAL
    int sleepcycles = (TX_INTERVAL/ 8);
    for (int i = 0; i < sleepcycles; i++) {
      // Enter power down state for 8 s with ADC and BOD module disabled
      LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
      cli();
      timer0_overflow_count+= 8*64*clockCyclesPerMicrosecond();
      sei();
    }
    Serial.println(F("Sleep complete"));
    next = false;

  do_send(&sendjob);
  }
#endif
  }



void readDHT(void)
{
  int h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  int t = dht.readTemperature()*100;
  // Read temperature as Fahrenheit (isFahrenheit = true)
  int f = dht.readTemperature(true);
  if (isnan(h) || isnan(t)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }



  mydata[0]=highByte(t);
  mydata[1]=lowByte(t);
  mydata[2]=(h);
  
  
  
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
    return;
  }
  //appData.dust_density = dustDensity;
  // Print readings
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
float analogvalue = 0, battVolt = 0;
 
  for (byte  i = 0; i < 10; i++) {
    analogvalue += analogRead(A3);
    delay(5);
  }
  analogvalue = analogvalue / 10;
#ifdef DEBUG
#endif
  battVolt = ((analogvalue * 3.3) / 1024) * 2; //ADC voltage*Ref. Voltage/1024
  mydata[5]=(int(battVolt));
 //mydata[7]=lowByte(battVolt);
}
