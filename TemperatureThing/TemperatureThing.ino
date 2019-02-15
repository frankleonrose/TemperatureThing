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

#undef min
#undef max

#include <SPI.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#include <Arduino_LoRaWAN_ttn.h>
#include <Logging.h>
#include <ParameterStore.h>
#include <LoraStack.h>

#include <Adafruit_FRAM_SPI.h>
#include <AdafruitFramSPIStore.h>

#include <Timer.h>

#define LOG_LEVEL LOG_LEVEL_DEBUG //  _INFOS, _DEBUG, _VERBOSE, _NOOUTPUT

#define SEALEVELPRESSURE_HPA (1013.25)

#define BME_CS 19
Adafruit_BME280 bme(BME_CS); // hardware SPI

#define FRAM_CS 18

bool Serial_::dtr() { return true; } // Satisfy linkage of MCCI Arduino LoRaWAN Library/LogPrint.cpp

Timer timer;

#define SEQ_NO_LOST_TO_RESET (1482285 + 114856)
#define SEQ_NO_20171206 522747
#define SEQ_NO_20171209 523264
#define SEQ_NO_20180213 523353
#define SEQ_NO_20190124 523546
#define SEQ_NO_20190215 605838
#define STORE_SEQ_NO

// Lorawan Network Session Key, App Session Key, and Device Address
// temp-0001
static const u1_t PROGMEM NWKSKEY[16] = /* MSB */ { 0x05, 0x3B, 0x54, 0x8C, 0xB4, 0x27, 0x1C, 0x78, 0x1E, 0x47, 0x49, 0xE3, 0x92, 0xEB, 0x2C, 0xA8 };
static const u1_t PROGMEM APPSKEY[16] = /* MSB */ { 0xB7, 0xB3, 0xF6, 0x8A, 0x9E, 0xBE, 0x1B, 0x2B, 0x15, 0x04, 0x7A, 0x00, 0xCB, 0x99, 0xB4, 0xE3 };
static const u4_t DEVADDR = 0x260213AD ; // <-- Change this address for every node!

// temp-0001a
// static const u1_t PROGMEM NWKSKEY[16] = /* MSB */ { 0xA4, 0xC0, 0xC4, 0x24, 0x75, 0xA2, 0x89, 0x47, 0x6A, 0x4D, 0xC1, 0x2B, 0xB2, 0xDA, 0x60, 0x44 };
// static const u1_t PROGMEM APPSKEY[16] = /* MSB */ { 0x26, 0x32, 0x0D, 0x85, 0x54, 0xE2, 0x17, 0x57, 0x8C, 0x08, 0xFE, 0xD5, 0xB3, 0x11, 0xCA, 0xF5 };
// static const u4_t DEVADDR = 0x260214B3; // <-- Change this address for every node!

uint8_t readings[10];
uint16_t head = 0;

#define PACKET_FORMAT_ID 0xFA

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL_SEC = 3600; // Every hour

// Pin mapping
const Arduino_LoRaWAN::lmic_pinmap define_lmic_pins = {
#if 0 // Feather LoRa wiring (with IO1 <--> GPIO#6)
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {3, 6, LMIC_UNUSED_PIN},
#else // Which board?
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {3, 11, LMIC_UNUSED_PIN}, // 3, 11, 10
#endif
};
Adafruit_FRAM_SPI fram = Adafruit_FRAM_SPI(FRAM_CS);
AdafruitFramSPIStore byteStore(fram, 400, 200);
ParameterStore pstore(byteStore);
LoraStack_LoRaWAN lorawan(define_lmic_pins, pstore);
LoraStack node(lorawan, pstore, TTN_FP_US915);

void onEvent(void *ctx, uint32_t event) {
  Log.Debug(F("Event: %d" CR), event);
  if (event==EV_TXCOMPLETE) {
    Log.Debug(F("EV_TXCOMPLETE (includes waiting for RX windows)" CR));
    digitalWrite(LED_BUILTIN, LOW);
  }
//             if (LMIC.txrxFlags & TXRX_ACK)
//               Log.Debug(F("Received ack" CR));
//             if (LMIC.dataLen) {
//               Log.Debug(F("Received %d bytes of payload" CR), LMIC.dataLen);
//             }
//             loraSeqNo = LMIC_getSeqnoUp(); // LMIC_getSeqnoUp returns the NEXT seq to use. Store that.
//             Log.Debug(F("Sent packet number %d" CR), loraSeqNo);
// #if defined(STORE_SEQ_NO)
//             store.writeu32(SEQ_NO_OFFSET, loraSeqNo + SEQ_NO_ADJUST);
// #endif
}

void onReceive(const uint8_t *payload, size_t size, port_t port) {
  Log.Debug(F("Received message on port: %d" CR), port);
  Log.Debug(F("Message [%d]: %*m" CR), size, size, payload);
}

uint8_t readBatteryLevel() {
    #define VBATPIN A7

    float measuredvbat = analogRead(VBATPIN);
    measuredvbat *= 2;    // we divided by 2, so multiply back
    measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
    measuredvbat /= 1024; // convert to voltage
    Log.Debug("VBat: %f", measuredvbat);

    // Normalize to 0-100
    float minv = 3.2;
    float maxv = 4.2;
    uint8_t level = 100 * (measuredvbat - minv) / (maxv - minv);
    Log.Debug_(" [VBat int: %d]" CR, level);
    if (level<0) {
      level = 0;
    }
    else if (level>100) {
      level = 100;
    }
    return level;
}

void do_send() {
    // Check if there is not a current TX/RX job running
    // if (LMIC.opmode & OP_TXRXPEND) {
    //     Log.Debug(F("OP_TXRXPEND, not sending" CR));
    // }
    // else {
    // Prepare upstream data transmission at the next possible time.
    uint8_t packet[2 + sizeof(readings)];

    ASSERT(head<sizeof(readings));
    uint16_t remain = sizeof(readings) - head;
    packet[0] = PACKET_FORMAT_ID;
    memcpy(packet + 1, readings + head, remain);
    memcpy(packet + 1 + remain, readings, sizeof(readings) - remain);
    packet[sizeof(packet)-1] = readBatteryLevel();

    Log.Debug(F("head %d, remain %d" CR), head, remain);
    Log.Debug(F("Readings: %*m" CR), sizeof(readings), readings);
    Log.Debug(F("Writing packet: %*m" CR), sizeof(packet), packet);

    digitalWrite(LED_BUILTIN, HIGH);

    // LMIC_setTxData2(1, packet, sizeof(packet), 0);
    ttn_response_t ret = node.sendBytes(packet, sizeof(packet));
    if (ret!=TTN_SUCCESSFUL_TRANSMISSION) {
      Log.Error(F("Failed to transmit: %d" CR), ret);
    }
    else {
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
  // printValues();

  /* 0-240: -10 to 50 step 0.25 */
  float tempc = bme.readTemperature();
  uint8_t encoded;
  if (tempc < -10.0) {
    encoded = 241; // Underflow
  }
  else if (tempc>50.0) {
    encoded = 242; // Overflow
  }
  else {
    encoded = ((tempc+10) * 4.0);
  }
  if (--head >= sizeof(readings)) {
    head = sizeof(readings) - 1;
  }
  readings[head] = encoded;

  do_send();
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    Serial.begin(115200);
    Log.Init(LOG_LEVEL, Serial);
    // Wait for 15 seconds. If no Serial by then, keep going. We are not connected.
    for (int timeout=0; timeout<15 && !Serial; ++timeout) {
      delay(1000);
    }
    Log.Debug(F("Starting" CR));

#define LORA_CS 8
    Log.Debug("Writing default value to NSS: %d", LORA_CS);
    digitalWrite(FRAM_CS, HIGH); // Default, unselected
    pinMode(FRAM_CS, OUTPUT);
    digitalWrite(BME_CS, HIGH); // Default, unselected
    pinMode(BME_CS, OUTPUT);
    digitalWrite(LORA_CS, HIGH); // Default, unselected
    pinMode(LORA_CS, OUTPUT);

    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

    Log.Debug(F("Connecting to storage!" CR));
    bool status = byteStore.begin();
    if (!status) {
        Log.Error(F("Could not find a valid FRAM module, check wiring!" CR));
        while (1);
    }
    Log.Debug(F("Initializing parameter store!" CR));
    status = pstore.begin();
    if (!status) {
        Log.Error(F("Failed to initialize Parameter Store!" CR));
        while (1);
    }
    // Update count if we haven't moved beyond it...
    const int32_t updatedCount = SEQ_NO_20190215;
    uint32_t currentCount = 0;
    if (PS_SUCCESS!=pstore.get("FCNTUP", &currentCount)) {
      currentCount = 0;
    }
    if (currentCount<updatedCount) {
      pstore.set("FCNTUP", updatedCount);
    }

    Log.Debug(F("Registering lorawan event listener!" CR));
    status = lorawan.RegisterListener(onEvent, NULL);
    if (!status) {
        Log.Error(F("Failed to register listener!" CR));
        while (1);
    }
    // const char *appEui = "0000000000000000";
    // const char *devEui = "0000000000000000";
    // const char *appKey = "00000000000000000000000000000000";
    // status = node.provision(appEui, devEui, appKey);
    const char *devAddr = "260213AD";
    const char *nwkSKey = "053B548CB4271C781E4749E392EB2CA8";
    const char *appSKey = "B7B3F68A9EBE1B2B15047A00CB99B4E3";
    Log.Debug(F("Personalizing node!" CR));
    status = node.personalize(devAddr, nwkSKey, appSKey);
    if (!status) {
        Log.Error(F("Failed to personalize device!" CR));
        while (1);
    }

    Log.Debug(F("Registering ttn message listener!" CR));
    node.onMessage(onReceive);

    // LMIC init
    lorawan.SetDebugMask(Arduino_LoRaWAN::LOG_BASIC | Arduino_LoRaWAN::LOG_ERRORS | Arduino_LoRaWAN::LOG_VERBOSE);
    Log.Debug(F("Begin lorawan interaction!" CR));
    lorawan.begin();

    // // Disable link check validation
    // Log.Debug(F("LMIC_setLinkCheckMode" CR));
    // LMIC_setLinkCheckMode(0);

    // // TTN uses SF9 for its RX2 window.
    // LMIC.dn2Dr = DR_SF9;

    // // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    // Log.Debug(F("LMIC_setDrTxpow" CR));
    // LMIC_setDrTxpow(DR_SF7, 14);

    // default settings
    Log.Debug(F("Connecting to BME280!" CR));
    bool bstatus = bme.begin();
    if (!bstatus) {
        Log.Debug(F("Could not find a valid BME280 sensor, check wiring!" CR));
        // while (1);
    }

// #if defined(STORE_SEQ_NO)
//     loraSeqNo = store.readu32(SEQ_NO_OFFSET) - SEQ_NO_ADJUST;
// #endif
    // LMIC_setSeqnoUp(loraSeqNo);

    // fram.writeEnable(true);
    // fram.write8(0, 0x6A);
    // fram.writeEnable(false);

    // uint8_t byte = fram.read8(0);
    // Log.Debug(F("Read byte"));
    // Log.Debugln(byte);

    memset(readings, 0xFF, sizeof(readings));

    timer.every(TX_INTERVAL_SEC * 1000, sendTemp);

    Log.Debug(F("Setup complete" CR));

    do_send();
}

void loop() {
  // Log.Debug(F("os_runloop_once" CR));
    lorawan.loop();
    timer.update();
}

void LMIC_DEBUG_PRINTF(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    // char buffer[200];
    // vsnprintf(buffer, sizeof(buffer), fmt, args);
    Log.DebugArgs(fmt, args);
    va_end(args);
}
