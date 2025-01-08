#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "LowPower.h"
#include <Arduino.h>
#include <Wire.h>
#include <ArtronShop_SHT45.h>
ArtronShop_SHT45 sht45(&Wire, 0x44); // SHT45-AD1B => 0x44

#ifdef COMPILE_REGRESSION_TEST
# define CFG_in866 1
#endif
bool next = false;

static const PROGMEM u1_t NWKSKEY[16] = {0xC9, 0x77, 0x5B, 0xE0, 0x10, 0x36, 0x96, 0xD2, 0x76, 0x81, 0xA1, 0x41, 0xFD, 0xD0, 0xE6, 0x9A};

static const u1_t PROGMEM APPSKEY[16] = {0x08, 0xDB, 0xE3, 0x86, 0x60, 0x04, 0x96, 0x75, 0xC5, 0x5B, 0x90, 0xBB, 0x04, 0xD2, 0x11, 0x92};

static const u4_t DEVADDR = 0xfc0097c2; // <-- Change this address for every node!

void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static uint8_t mydata[4];
static osjob_t sendjob;

const unsigned TX_INTERVAL = 60;

const lmic_pinmap lmic_pins = {
    .nss = 6,                       // chip select on feather (rf95module) CS
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 5,                       // reset pin
    .dio = {2, 3, LMIC_UNUSED_PIN}, // assumes external jumpers [feather_lora_jumper]
                                    // DIO1 is on JP1-1: is io1 - we connect to GPO6
                                    // DIO1 is on JP5-3: is D2 - we connect to GPO5
};

void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      break;
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
      
      /****schedule next transmission****/
      
#ifndef SLEEP
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
#else
      next = true;
#endif
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
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


void do_send(osjob_t* j){
 
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
       Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
    SHT45();
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {

  Wire.begin();
    Serial.begin(115200);
    delay(100);     // per sample code on RF_95 test
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
    // ... extra definitions for channels 3..n here.
    #else
    # error Region not supported
    #endif
    // Disable link check validation
    LMIC_setLinkCheckMode(0);
    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;
    // Set data rate and transmit power for uplink
    LMIC_setDrTxpow(DR_SF7,14);
    // Start job
    do_send(&sendjob);
}


void loop(){
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

void SHT45()
{ 
    sht45.measure();
    Serial.print("Temperature: ");
    Serial.print(sht45.temperature(),1);
    Serial.println(" *C");
    mydata[0] = highByte(int(sht45.temperature()));
    mydata[1] = lowByte(int(sht45.temperature()));
    
    Serial.print("Humidity:");
    Serial.print(sht45.humidity(), 1);
    Serial.println("Rh");
    mydata[2] = highByte(int(sht45.humidity()));
    mydata[3] = lowByte(int(sht45.humidity()));
    Serial.println();
    delay(500);
}
