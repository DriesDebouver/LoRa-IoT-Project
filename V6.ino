#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

// LoRaWAN NwkSKey, network session key
static const PROGMEM u1_t NWKSKEY[16] = { ... };
// LoRaWAN AppSKey, application session key
static const u1_t PROGMEM APPSKEY[16] = { ... };
// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x.....;

// VARIABLES:
float tempDht; //DHT11 Sensor
byte humiDht; //DHT11 Sensor
float tempBmp; //BMP280 Sensor
int presBmp; //BMP280 Sensor
int lux; //BH1750 Sensor

//DHT11 Sensor - Digital pin 3:
#include "DHT.h"
#define DHTPIN 3
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

//BMP280 Sensor - I2C:
#include <SPI.h>
#include <Adafruit_BMP280.h>
Adafruit_BMP280 bmp;

//BH1750 Sensor - I2C:
#include <Wire.h>
#include <BH1750.h>
BH1750 lightMeter;

// DATA:
byte payload[10];
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty cycle limitations).
const unsigned TX_INTERVAL = 10;

// Pin mapping (epecifically for Arduino Uno + Dragino LoRa Shield US900):
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 9,
  .dio = {2, 6, 7},
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
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    default:
      Serial.print(F("Unknown event: "));
      Serial.println((unsigned) ev);
      break;
  }
}

void do_send(osjob_t* j) {
  Serial.println("**********************************");
  // DHT11 SENSOR:
  dhtSensor();
  // BMP280 SENSOR:
  bmpSensor();
  // BH1750 SENSOR:
  luxSensor();

  // float to int:
  uint32_t tempDhtInt = tempDht * 100;
  uint32_t tempBmpInt = tempBmp * 100;

  // Average temperature calculation:
  uint32_t gemTemp = ( (tempDht + tempBmp) / 2 ) * 100;

  // Value's in payload:
  // DHT Temp:
  payload[0] = highByte(tempDhtInt);
  payload[1] = lowByte(tempDhtInt);
  // BMP Temp:
  payload[2] = highByte(tempBmpInt);
  payload[3] = lowByte(tempBmpInt);
  // Average Temp:
  payload[4] = highByte(gemTemp);
  payload[5] = lowByte(gemTemp);
  // Humidity:
  payload[6] = humiDht;
  // Pressure:
  payload[7] = highByte(presBmp);
  payload[8] = lowByte(presBmp);
  // Light:
  payload[9] = highByte(lux * 100);
  payload[10] = lowByte(lux * 100);

  // Check if there is not a current TX/RX job running:
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Send data:
    LMIC_setTxData2(1, payload, sizeof(payload), 0);
    Serial.println(F("Packet queued"));
    Serial.print(F("Sending packet on frequency: "));
    Serial.println(LMIC.freq);
  }
}

void setup() {
  while (!Serial);
  Serial.begin(115200);
  delay(100);
  Serial.println(F("Starting"));
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

  // Disable link check validation
  LMIC_setLinkCheckMode(0);
  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF7, 14);
  // Start job
  do_send(&sendjob);
}

void loop() {
  unsigned long now;
  now = millis();
  if ((now & 512) != 0) {
    digitalWrite(13, HIGH);
  }
  else {
    digitalWrite(13, LOW);
  }
  os_runloop_once();
}

void dhtSensor() {
  dht.begin();
  tempDht = dht.readTemperature();
  humiDht = dht.readHumidity();
  Serial.println(F("DHT11:"));
  Serial.print(F("Temperature: "));
  Serial.print(tempDht);
  Serial.println(F(" °C"));
  Serial.print(F("Humidity: "));
  Serial.print(humiDht);
  Serial.println(F(" %"));
}

void bmpSensor() {
  bmp.begin();
  tempBmp = bmp.readTemperature();
  presBmp = bmp.readPressure() / 100;
  Serial.println(F("BMP280:"));
  Serial.print(F("Temperature: "));
  Serial.print(tempBmp);
  Serial.println(F(" °C"));
  Serial.print(F("Pressure: "));
  Serial.print(presBmp);
  Serial.println(F(" mb / hPa"));
}

void luxSensor() {
  Wire.begin();
  lightMeter.begin();
  lux = lightMeter.readLightLevel();
  Serial.println(F("BH1750:"));
  Serial.print(F("Light: "));
  Serial.print(lux);
  Serial.println(F(" lux"));
}
