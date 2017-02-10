/*******************************************************************************
 * Copyright (c) 2017 Frank Leon Rose
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * Temperature sensor built with
 * Feather LoRa M0
 * Adafruit BME280 temp, humidity, and pressure sensor
 * Adafruit FRAM module for storage
 *
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#include <Adafruit_FRAM_SPI.h>

#include "Logging.h"

#include "Timer.h"

#define LOG_LEVEL LOG_LEVEL_DEBUG //  _INFOS, _DEBUG, _VERBOSE, _NOOUTPUT

#define SEALEVELPRESSURE_HPA (1013.25)

#define BME_CS 19
Adafruit_BME280 bme(BME_CS); // hardware SPI

#define FRAM_CS 18

#define MIN(a,b) ((a)<(b) ? (a) : (b))

Timer timer;

// void     write8 (uint16_t addr, uint8_t value);
// void     write (uint16_t addr, const uint8_t *values, size_t count);
// uint8_t  read8  (uint16_t addr);
// void     read (uint16_t addr, uint8_t *values, size_t count);
class NonVolatileStore {
  const uint size;
  const uint dataOffset;
public:
  bool begin() {
    if (!isMagicSet()) {
      Log.Info(F("Did not find magic number! Clearing storage." CR));
      resetStore();
    }
    return true;
  }
protected:
  NonVolatileStore(uint size)
    : size(size),
      dataOffset(sizeof(uint32_t)) {
  }
  bool isMagicSet() {
    #define MAGIC_NUMBER 0xFADE0042
    uint32_t magic_value = 0;
    // Magic number is in first 4 bytes of store.
    readImpl(0, (uint8_t *)&magic_value, sizeof(magic_value));
    Log.Debug(F("Read magic number %x" CR), magic_value);
    return (magic_value==MAGIC_NUMBER);
  }
  virtual uint8_t readbyteImpl(uint16_t offset) = 0;
  virtual void readImpl(uint16_t offset, const void *addr, uint16_t size) =  0;
  virtual void writebyteImpl(uint16_t offset, uint8_t value) = 0;
  virtual void writeImpl(uint16_t offset, void *bytes, uint16_t size) = 0;
public:
  uint8_t readbyte(uint16_t offset) {
    ASSERT((dataOffset + offset)<this->size);
    return readbyteImpl(dataOffset + offset);
  }
  uint32_t readu32(uint16_t offset) {
    uint32_t value = 0;
    ASSERT((dataOffset + offset + sizeof(value))<=this->size);
    readImpl(dataOffset + offset, (uint8_t *)&value, sizeof(value));
    return value;
  }
  void read(uint16_t offset, void *addr, uint16_t size) {
    ASSERT((dataOffset + offset + size)<=this->size);
    readImpl(dataOffset + offset, addr, size);
  }
  void writeu32(uint16_t offset, uint32_t value) {
    ASSERT((dataOffset + offset + sizeof(value))<=this->size);
    writeImpl(dataOffset + offset, (uint8_t *)&value, sizeof(value));
  }
  void resetStore() {
    uint8_t zeroes[100];
    memset(zeroes, 0, sizeof(zeroes));
    for (uint16_t off = 0; off < this->size; off += sizeof(zeroes)) {
      writeImpl(off, zeroes, MIN(sizeof(zeroes), (this->size - off)));
    }
    uint32_t magic_value = MAGIC_NUMBER;
    writeImpl(0, (uint8_t *)&magic_value, sizeof(magic_value));
  }
};

class FramStore : public NonVolatileStore {
  Adafruit_FRAM_SPI &fram;
  const uint offset;

public:
  FramStore(Adafruit_FRAM_SPI &fram, uint size, uint offset = 0)
    : NonVolatileStore(size), fram(fram), offset(offset) {
  }

  bool begin() {
    if (!fram.begin()) {
      return false;
    }
    return NonVolatileStore::begin();
  }
protected:
  virtual uint8_t readbyteImpl(uint16_t offset) {
    return fram.read8(offset);
  }
  virtual void readImpl(uint16_t offset, const void *addr, uint16_t size) {
    fram.read(offset, (uint8_t *)addr, size);
  }
  virtual void writebyteImpl(uint16_t offset, uint8_t value) {
    fram.writeEnable(true);
    fram.write8(offset, value);
    fram.writeEnable(false);
  }
  virtual void writeImpl(uint16_t offset, void *bytes, uint16_t size) {
    fram.writeEnable(true);
    fram.write(offset, (uint8_t *)bytes, size);
    fram.writeEnable(false);
  }
};

Adafruit_FRAM_SPI fram = Adafruit_FRAM_SPI(FRAM_CS);
FramStore store = FramStore(fram, 100);

#define SEQ_NO_OFFSET 0
uint32_t loraSeqNo = 0;

// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const PROGMEM u1_t NWKSKEY[16] = /* MSB */ { 0x9C, 0xEB, 0x81, 0xB5, 0xE5, 0x68, 0x5E, 0x5D, 0x6A, 0x4E, 0x9F, 0x5F, 0xC1, 0x26, 0x24, 0x48 };

// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const u1_t PROGMEM APPSKEY[16] = /* MSB */ { 0xC2, 0xD3, 0xCB, 0x7E, 0x15, 0xD7, 0x7A, 0xFD, 0xA5, 0xB7, 0x2E, 0xF9, 0x1C, 0x02, 0xCB, 0x95 };

// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x26011EF8 ; // <-- Change this address for every node!

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

uint8_t readings[10];
uint16_t head = 0;
uint16_t battery = 0xFF;

#define PACKET_FORMAT_ID 0xFA

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL_SEC = 10; // seconds

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {3, 11, LMIC_UNUSED_PIN}, // 3, 11, 10
};

void onEvent (ev_t ev) {
    Log.Debug(F("onEvent [%x]: "), os_getTime());
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Log.Debug(F("EV_SCAN_TIMEOUT" CR));
            break;
        case EV_BEACON_FOUND:
            Log.Debug(F("EV_BEACON_FOUND" CR));
            break;
        case EV_BEACON_MISSED:
            Log.Debug(F("EV_BEACON_MISSED" CR));
            break;
        case EV_BEACON_TRACKED:
            Log.Debug(F("EV_BEACON_TRACKED" CR));
            break;
        case EV_JOINING:
            Log.Debug(F("EV_JOINING" CR));
            break;
        case EV_JOINED:
            Log.Debug(F("EV_JOINED" CR));
            break;
        case EV_RFU1:
            Log.Debug(F("EV_RFU1" CR));
            break;
        case EV_JOIN_FAILED:
            Log.Debug(F("EV_JOIN_FAILED" CR));
            break;
        case EV_REJOIN_FAILED:
            Log.Debug(F("EV_REJOIN_FAILED" CR));
            break;
        case EV_TXCOMPLETE:
            Log.Debug(F("EV_TXCOMPLETE (includes waiting for RX windows)" CR));
            if (LMIC.txrxFlags & TXRX_ACK)
              Log.Debug(F("Received ack" CR));
            if (LMIC.dataLen) {
              Log.Debug(F("Received %d bytes of payload" CR), LMIC.dataLen);
            }
            loraSeqNo = LMIC_getSeqnoUp(); // LMIC_getSeqnoUp returns the NEXT seq to use. Store that.
            Log.Debug(F("Sent packet number %d" CR), loraSeqNo);
            store.writeu32(SEQ_NO_OFFSET, loraSeqNo);
            break;
        case EV_LOST_TSYNC:
            Log.Debug(F("EV_LOST_TSYNC" CR));
            break;
        case EV_RESET:
            Log.Debug(F("EV_RESET" CR));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Log.Debug(F("EV_RXCOMPLETE" CR));
            break;
        case EV_LINK_DEAD:
            Log.Debug(F("EV_LINK_DEAD" CR));
            break;
        case EV_LINK_ALIVE:
            Log.Debug(F("EV_LINK_ALIVE" CR));
            break;
        case EV_SCAN_FOUND:
            Log.Debug(F("EV_SCAN_FOUND" CR));
            break;
        case EV_TXSTART:
            Log.Debug(F("EV_TXSTART" CR));
            break;
         default:
            Log.Debug(F("Unknown event" CR));
            break;
    }
}

void dumpBytes(const char *msg, uint8_t *bytes, uint16_t size) {
  Log.Debug(msg);
  for (uint16_t i = 0; i < size; ++i) {
    Log.Debug_("%x ", bytes[i]);
  }
  Log.Debug_(F(CR));
}

void do_send() {
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Log.Debug(F("OP_TXRXPEND, not sending" CR));
    }
    else {
        // Prepare upstream data transmission at the next possible time.
        uint8_t packet[2 + sizeof(readings)];

        ASSERT(head<sizeof(readings));
        uint16_t remain = sizeof(readings) - head;
        packet[0] = PACKET_FORMAT_ID;
        memcpy(packet + 1, readings + head, remain);
        memcpy(packet + 1 + remain, readings, sizeof(readings) - remain);
        packet[sizeof(packet)-1] = battery;

        dumpBytes("Writing packet: ", packet, sizeof(packet));

        LMIC_setTxData2(1 /*0x13*/, packet, sizeof(packet), 0);
        Log.Debug(F("Packet queued" CR));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void printValues() {
    Log.Debug(F("Temperature = %f  *C" CR), bme.readTemperature());

    Log.Debug(F("Pressure = %f hPa" CR), (bme.readPressure() / 100.0F));

    Log.Debug(F("Approx. Altitude = %f m" CR), bme.readAltitude(SEALEVELPRESSURE_HPA));

    Log.Debug(F("Humidity = %f \%" CR), bme.readHumidity());
}

void sendTemp() {
  printValues();

  uint8_t temp = bme.readTemperature();
  readings[head--] = temp;
  if (head >= sizeof(readings)) {
    head = sizeof(readings) - 1;
  }

  do_send();
}

void setup() {
    Log.Init(LOG_LEVEL, 115200);
    // Wait for 15 seconds. If no Serial by then, keep going. We are not connected.
    for (int timeout=0; timeout<15 && !Serial; ++timeout) {
      delay(1000);
    }
    Log.Debug(F("Starting" CR));

    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

    // LMIC init

    Log.Debug(F("os_init" CR));
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    Log.Debug(F("LMIC_reset" CR));
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    Log.Debug(F("LMIC_setSession" CR));
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    #if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
    Log.Debug(F("EUROPE!" CR));
    #elif defined(CFG_us915)
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    Log.Debug(F("LMIC_selectSubBand" CR));
    LMIC_selectSubBand(1);
    #endif

    // Disable link check validation
    Log.Debug(F("LMIC_setLinkCheckMode" CR));
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    Log.Debug(F("LMIC_setDrTxpow" CR));
    LMIC_setDrTxpow(DR_SF7,14);

    // default settings
    Log.Debug(F("Begin measurement!" CR));
    bool status = bme.begin();
    if (!status) {
        Log.Debug(F("Could not find a valid BME280 sensor, check wiring!" CR));
        while (1);
    }

    Log.Debug(F("Begin storage!" CR));
    status = store.begin();
    if (!status) {
        Log.Debug(F("Could not find a valid FRAM module, check wiring!" CR));
        while (1);
    }
    loraSeqNo = store.readu32(SEQ_NO_OFFSET);
    LMIC_setSeqnoUp(loraSeqNo);

    // fram.writeEnable(true);
    // fram.write8(0, 0x6A);
    // fram.writeEnable(false);

    // uint8_t byte = fram.read8(0);
    // Log.Debug(F("Read byte"));
    // Log.Debugln(byte);

    memset(readings, 0xFF, sizeof(readings));

    timer.every(TX_INTERVAL_SEC * 1000, sendTemp);

    Log.Debug(F("setup complete" CR));
}

void loop() {
  // Log.Debug(F("os_runloop_once" CR));
    os_runloop_once();
    timer.update();
}
