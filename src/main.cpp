#if !defined(ESP8266)
#error This file is for ESP8266 only (Wifi Events)
#endif

// ------------------------------------ DEFINE ---------------------------------

#define COMPDATE __DATE__ " " __TIME__

// #define DEBUG_TO_SERIAL 1
// #define DEBUG_ASYNCMQTTCLIENT 1
// ------------------------------------ DEBUG ----------------------------------

#ifdef DEBUG_TO_SERIAL
const long SERIAL_BAUD                = 115200;
 #define DEBUG_BEGIN()          Serial.begin(SERIAL_BAUD)
 #define DEBUG_PRINT(x)         Serial.print (x)
 #define DEBUG_PRINTLN(x)       Serial.println (x)
 #define DEBUG_PRINTF(...)    { Serial.printf("%lu",millis()); Serial.print("mS L"); Serial.print (__LINE__); Serial.print(" "); Serial.printf(__VA_ARGS__); }
 #define DEBUG_FLUSH()          Serial.flush()
#else
 #define DEBUG_BEGIN()
 #define DEBUG_PRINT(x)
 #define DEBUG_PRINTLN(x)
 #define DEBUG_PRINTF(...)
 #define DEBUG_FLUSH()
#endif

// ------------------------------------ LIBRARIES ------------------------------

#include <Arduino.h>                 // Arduino library - https://github.com/esp8266/Arduino/tree/master/cores/esp8266
#include <Wire.h>                    // Arduino library - https://github.com/esp8266/Arduino/tree/master/libraries/Wire
// BME280 library demands SPI, but it's not used
#include <SPI.h>                     // Arduino library - https://github.com/esp8266/Arduino/tree/master/libraries/SPI

// WiFi
#include <ESP8266WiFi.h>             // Arduino library - https://github.com/esp8266/Arduino/blob/master/libraries/ESP8266WiFi/src/ESP8266WiFi.h
#include <WiFiClient.h>              // Arduino library - https://github.com/esp8266/Arduino/tree/master/libraries/ESP8266WiFi/src/WiFiClient.h
#include <ESP8266HTTPClient.h>       // Arduino library - https://github.com/esp8266/Arduino/tree/master/libraries/ESP8266HTTPClient/src
#include <ESP8266httpUpdate.h>       // Arduino library - https://github.com/esp8266/Arduino/tree/master/libraries/ESP8266httpUpdate/src

#include <AsyncMqttClient.h>         // https://github.com/marvinroger/async-mqtt-client
//#include <PubSubClient.h>            // https://github.com/knolleary/pubsubclient
#include "secrets.h"                 // Credentials
// rtcmem and sensor
#include <rtcmem.h>
#include <CRC32.h>                   // https://github.com/bakercp/CRC32
#include <BME280I2C.h>               // https://www.github.com/finitespace/BME280

// ------------------------------------ CONSTANTS ------------------------------

// WiFi
#ifndef SECRETS_H
 #define SECRETS_H
const char WIFI_SSID[]                = "*** WIFI SSID ***";
const char WIFI_PASSWORD[]            = "*** WIFI PASSWORD ***";
const char MQTT_HOST[]                = "192.168.1.x";
const int MQTT_PORT                   = 1883;
#endif

// Device sleeptimes
const uint32_t SECONDS_OFFSET         = 60;
const uint64_t SLEEPTIME              = 60e6;
const unsigned long FORCE_SLEEP_MILLIS = 10e6; // force sleep after 10 seconds
#if defined(DEBUG_TO_SERIAL)
const uint16_t BATTERY_TRESHOLD       = 3200; // in mV
const uint16_t MAX_RECORDCOUNT        = 10;
const uint64_t LONG_SLEEPTIME         = 60e6;
#else
const uint16_t BATTERY_TRESHOLD       = 3200; // in mV
const uint16_t MAX_RECORDCOUNT        = 10;
const uint64_t LONG_SLEEPTIME         = 3600e6;
#endif
// MQTT
const char* MQTT_TOPIC_TELE           = "sensor/%d/tele";
const char* MQTT_MSG_TELE             = "{\"deviceid\":%d,\"VCC\":\"%s\",\"counter\":%d,\"compiled\":\"%s\"}";

const char* MQTT_TOPIC_SENSOR         = "sensor/%d/bme280/%d";
const char* MQTT_MSG_SENSOR           = "{\"deviceid\":%d,\"sensorid\":%d,\"o\":%d,\"h\":%d,\"p\":%d,\"t\":%d}";

const char* MQTT_TOPIC_ACTION         = "sensor/%d/action";
const char* MQTT_MSG_ACTION           = "{\"deviceid\":%d,\"VCC\":\"%s\",\"action\":\"replace battery\"}";

// ------------------------------------ GLOBALS --------------------------------

// Device
uint32_t deviceID;                   // Unique ID of MCU=ESP8266
uint32_t sensorID;                   // Fingerprint ID of BME280
unsigned long previousMillis          = 0;
bool ActionWiFiGotIP                  = false;  // 10
bool ActionMQTT                       = false;  // 20
bool ActionGoToSleep                  = false;  // 99
bool ActionBatteryLow                 = false; 

// rtcmem
RTCMEM localmemory;

// WiFi
WiFiClient espClient;

// AsyncMqttClient = NEW MQTT
AsyncMqttClient mqttClient;
uint16_t lastSentPackageId = 0;
uint16_t lastAcknPackageId = 0;

// PubSubClient = OLD MQTT
//void callback(char* topic, byte* payload, unsigned int length); // pre-declare
//PubSubClient mqtt(MQTT_HOST, MQTT_PORT, callback, espClient);

// BME280
BME280I2C::Settings settings(
	BME280::OSR_X1,
	BME280::OSR_X1,
	BME280::OSR_X1,
	BME280::Mode_Forced,
	BME280::StandbyTime_1000ms,
	BME280::Filter_Off,
	BME280::SpiEnable_False,
	BME280I2C::I2CAddr_0x76 // I2C address. I2C specific.
);

BME280I2C bme(settings);

typedef struct {
  uint32_t hum100;
  uint32_t pres100;
  int32_t  temp100;
} sensorData;

sensorData sensordata;

// ------------------------------------ FUNCTIONS ------------------------------

void GoToSleep();       // pre declare
void SaveAllToMQTT();   // pre declare

// define int __get_adc_mode(void) { return (int) (mode /* ADC_VCC = 255 */); }
ADC_MODE(ADC_VCC);

// char* dtostrf (double __val, signed char __width, unsigned char __prec, char *__s)
char* dtostrfd(double number, unsigned char prec, char* buffer)
{
  if ((isnan(number)) || (isinf(number))) {  // Fix for JSON output (https://stackoverflow.com/questions/1423081/json-left-out-infinity-and-nan-json-status-in-ecmascript)
    strcpy(buffer, "null");
    return buffer;
  } else {
    return dtostrf(number, 1, prec, buffer);
  }
}

// set deviceID with 6 byte
void setDeviceId()
{
  #if defined(ESP8266)
    deviceID = ESP.getChipId();
  #elif defined(ESP32)
    deviceID = ESP.getEfuseMac() & 0xffffff;
  #endif
}

// set sensorID
void setSensorId()
{
  uint8_t test[32];
  sensorID = CRC32::calculate(bme.compensationParameters(test), 32);
#ifdef DEBUG
  for (int i=0; i < 32; i++) DEBUG_PRINTF("%.2X ", test[i]);
  DEBUG_PRINTLN(); DEBUG_PRINTF("%.8X\n", CRC32::calculate(bme.compensationParameters(test), 32));
#endif
}

void BMEbegin() {
  // BME280
  DEBUG_PRINTF("Starting BME280\n");
  Wire.begin();
  if (!bme.begin()) {
    DEBUG_PRINTLN(F("Could not find BME280 sensor"));
    DEBUG_PRINTLN(F("You have 60 seconds to connect the wires ..."));
    GoToSleep();
  }
  if (bme.chipModel() != BME280::ChipModel_BME280) {
    DEBUG_PRINTLN(F("Found BME280, but not full BME280 model"));
    DEBUG_PRINTLN(F("You have 60 seconds to connect a full BME280 ..."));
    GoToSleep();
  }
}

void ReadBME(sensorData *data)
{
  // read BME280
  float temp(NAN), hum(NAN), pres(NAN);
  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_hPa);
  bme.read(pres, temp, hum, tempUnit, presUnit);

  data->hum100  = (uint32_t)(hum * 100);
  data->pres100 = (uint32_t)(pres * 100);
  data->temp100 = (int32_t)(temp * 100);
}

// ******************** AsyncMqttClient ********************

void onMqttConnect(bool sessionPresent) {
#if defined(DEBUG_ASYNCMQTTCLIENT)
  DEBUG_PRINTF("Connected to MQTT. Session present: %s\n",((sessionPresent) ? "true" : "false"));
#endif
  ActionMQTT = true;
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
#if defined(DEBUG_ASYNCMQTTCLIENT)
  DEBUG_PRINTLN("Disconnected from MQTT.");
#endif
// ERROR
ActionGoToSleep = true;
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
#if defined(DEBUG_ASYNCMQTTCLIENT)
  DEBUG_PRINTLN("Subscribe acknowledged.");
  DEBUG_PRINT("  packetId: ");
  DEBUG_PRINTLN(packetId);
  DEBUG_PRINT("  qos: ");
  DEBUG_PRINTLN(qos);
#endif
}

void onMqttUnsubscribe(uint16_t packetId) {
#if defined(DEBUG_ASYNCMQTTCLIENT)
  DEBUG_PRINTLN("Unsubscribe acknowledged.");
  DEBUG_PRINT("  packetId: ");
  DEBUG_PRINTLN(packetId);
#endif
}


void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
#if defined(DEBUG_ASYNCMQTTCLIENT)
  DEBUG_PRINTLN("Publish received.");
  DEBUG_PRINT("  topic: ");
  DEBUG_PRINTLN(topic);
  DEBUG_PRINT("  qos: ");
  DEBUG_PRINTLN(properties.qos);
  DEBUG_PRINT("  dup: ");
  DEBUG_PRINTLN(properties.dup);
  DEBUG_PRINT("  retain: ");
  DEBUG_PRINTLN(properties.retain);
  DEBUG_PRINT("  len: ");
  DEBUG_PRINTLN(len);
  DEBUG_PRINT("  index: ");
  DEBUG_PRINTLN(index);
  DEBUG_PRINT("  total: ");
  DEBUG_PRINTLN(total);
#endif
}

void onMqttPublish(uint16_t packetId) {
  lastAcknPackageId = packetId;
#if defined(DEBUG_ASYNCMQTTCLIENT)
  DEBUG_PRINTF("Publish packetid %d acknowledged.\n", packetId);
#endif
}

void MqttConnect() {
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);

  mqttClient.setServer(MQTT_HOST, MQTT_PORT);

  mqttClient.connect();
}

// step 2
void mqtt_tele() {
  // VCC log
  char buffer[200];
  char topic[100];
  char sVCC[10];

  dtostrfd((double)ESP.getVcc()/1000, 3, sVCC);
  sprintf(topic, MQTT_TOPIC_TELE, deviceID);
  sprintf(buffer, MQTT_MSG_TELE, deviceID, sVCC, localmemory.counter(), COMPDATE);
  
  // uint16_t AsyncMqttClient::publish(const char* topic, uint8_t qos, bool retain, const char* payload, size_t length, bool dup, uint16_t message_id) {
  // do {
    delay(0);
    lastSentPackageId = mqttClient.publish(/* topic */ topic, /* qos */ 1, /* retain */ true, /* payload */ buffer);
    delay(0);
  // } while (lastSentPackageId == 0);
#if defined(DEBUG_ASYNCMQTTCLIENT)
  DEBUG_PRINTF("Published tele package. packId: %u\n  %s: %s\n", lastSentPackageId, topic, buffer);
#endif
}

// step 3
void mqtt_sensor() {
  char buffer[200];
  char topic[100];
  uint16_t records = localmemory.recordCount();
  uint16_t recordnr = 0;
  while (recordnr < records)
  {
    localmemory.getRecord(&sensordata, recordnr);

    sprintf(topic, MQTT_TOPIC_SENSOR, deviceID, sensorID);
    sprintf(buffer, MQTT_MSG_SENSOR, deviceID, sensorID, (records-recordnr-1) * SECONDS_OFFSET, sensordata.hum100, sensordata.pres100, sensordata.temp100);
  do {
    delay(1);
    lastSentPackageId = mqttClient.publish(/* topic */ topic, /* qos */ 0, /* retain */ true, /* payload */ buffer);
    delay(1);
#if defined(DEBUG_ASYNCMQTTCLIENT)
  DEBUG_PRINTF("Published sensor package %u.  packId: %u\n  %s: %s\n", recordnr, lastSentPackageId, topic, buffer);
#endif
  } while (lastSentPackageId == 0);
    recordnr++;
  }
}

// step 3
void mqtt_action() {
  if (ActionBatteryLow) {
    char buffer[200];
    char topic[100];
    char sVCC[10];
    dtostrfd((double)ESP.getVcc()/1000, 3, sVCC);
    sprintf(topic, MQTT_TOPIC_ACTION, deviceID);
    sprintf(buffer, MQTT_MSG_ACTION, deviceID, sVCC);

    do {
      delay(1);
      lastSentPackageId = mqttClient.publish(/* topic */ topic, /* qos */ 1, /* retain */ true, /* payload */ buffer);
      delay(1);
#if defined(DEBUG_ASYNCMQTTCLIENT)
  DEBUG_PRINTF("Published action package.  packId: %u\n  %s: %s\n", lastSentPackageId, topic, buffer);
#endif
    } while (lastSentPackageId == 0);
  
  }
}

void updateSketch() {
  DEBUG_PRINTF("update sketch. Counter: %d\n", localmemory.counter());
  if ((localmemory.counter() & 63) == 63) {
    ESPhttpUpdate.setLedPin(LED_BUILTIN, LOW);
    t_httpUpdate_return ret = ESPhttpUpdate.update(espClient, HTTP_UPDATE_SERVER, COMPDATE);

    switch (ret) {
      case HTTP_UPDATE_FAILED:
        DEBUG_PRINTF("HTTP_UPDATE_FAILD Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
        break;
      case HTTP_UPDATE_NO_UPDATES:
        DEBUG_PRINTLN("HTTP_UPDATE_NO_UPDATES");
        break;
      case HTTP_UPDATE_OK:
        DEBUG_PRINTLN("HTTP_UPDATE_OK");
        break;
    }
  }

}

// ********************  WIFI  ********************
// More events: https://github.com/esp8266/Arduino/blob/master/libraries/ESP8266WiFi/src/ESP8266WiFiGeneric.h

void onSTAConnected(const WiFiEventStationModeConnected& e /*String ssid, uint8 bssid[6], uint8 channel*/) {
  DEBUG_PRINTF("WiFi Connected: SSID %s @ BSSID %.2X:%.2X:%.2X:%.2X:%.2X:%.2X Channel %d\n",
    e.ssid.c_str(), e.bssid[0], e.bssid[1], e.bssid[2], e.bssid[3], e.bssid[4], e.bssid[5], e.channel);
 }

void onSTADisconnected(const WiFiEventStationModeDisconnected& e /*String ssid, uint8 bssid[6], WiFiDisconnectReason reason*/) {
  DEBUG_PRINTF("WiFi Disconnected: SSID %s BSSID %.2X:%.2X:%.2X:%.2X:%.2X:%.2X Reason %d\n",
    e.ssid.c_str(), e.bssid[0], e.bssid[1], e.bssid[2], e.bssid[3], e.bssid[4], e.bssid[5], e.reason);
  // Reason: https://github.com/esp8266/Arduino/blob/master/libraries/ESP8266WiFi/src/ESP8266WiFiType.h

  ActionWiFiGotIP = false;
  ActionGoToSleep = true;
}

void onSTAGotIP(const WiFiEventStationModeGotIP& e /*IPAddress ip, IPAddress mask, IPAddress gw*/) {
  DEBUG_PRINTF("WiFi GotIP: localIP %s SubnetMask %s GatewayIP %s\n",
    e.ip.toString().c_str(), e.mask.toString().c_str(), e.gw.toString().c_str());

  ActionWiFiGotIP = true;
}

// WiFi ON and connect
void connectToWiFi() {
  DEBUG_PRINTF("ConnectToWiFi\n");
  WiFi.forceSleepWake();
  delay(1);
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void disconnectFromWiFi() {
  WiFi.disconnect(true);
  delay(1);
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  delay(5);
  delay(50); // stabilize power for BME280
}

// save mem, wifi off, sleep
void GoToSleep() {
  DEBUG_PRINTF("GoToSleep\n");

  if (WiFi.isConnected()) disconnectFromWiFi();
  
  // Read BME280 - this is old data.
  delay(50); // experience we need some time to get stable reading (probably stable power?)
  ReadBME(&sensordata); // it returns last reading, and initiate new one
  // save in rtc memory
  localmemory.addRecord(&sensordata, sizeof(sensordata));
  localmemory.saveMem();

  DEBUG_PRINTF("deepSleep\n");
  DEBUG_FLUSH();
  // WAKE_RF_DISABLED to keep the WiFi radio disabled when we wake up
  if (ActionBatteryLow) {
    ESP.deepSleep(LONG_SLEEPTIME, RF_DISABLED);
  } else {
    ESP.deepSleep(SLEEPTIME, RF_DISABLED);
  }
}

void SaveAllToMQTT() {
  // DEBUG_PRINTF("SaveAllToMQTT -> mqtt_sensor\n");
  mqtt_sensor();
  // DEBUG_PRINTF("SaveAllToMQTT -> mqtt_tele\n");
  mqtt_tele();
  // DEBUG_PRINTF("SaveAllToMQTT -> mqtt_action\n");
  mqtt_action();
}

// ------------------------------------ SETUP ----------------------------------

void setup() {
  static WiFiEventHandler e1, e2, e4;
  // Set WiFi Off
  if (WiFi.getMode() != WIFI_OFF) {
    WiFi.persistent(true);
    WiFi.setAutoConnect(false);           // do not automatically connect on power on to the last used access point
    WiFi.setAutoReconnect(false);         // attempt to reconnect to an access point in case it is disconnected
    WiFi.mode(WIFI_OFF);
    WiFi.persistent(false);
  }
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  delay(1);

  // Serial
  DEBUG_BEGIN();
  DEBUG_PRINTLN();
  DEBUG_PRINTF("setup, WiFi is off\n");

  // RTCMEM
  if (localmemory.loadMem()) {
    DEBUG_PRINTF("RTCMEM: OK  Records: %d\n", localmemory.recordCount());
  } else {
    DEBUG_PRINTLN("RTCMEM: Error (normal at first boot)");
    DEBUG_PRINTF("Record size: %u\n", sizeof(sensordata))
    localmemory.reset(sizeof(sensordata));
  }

  // Check for transfering memory
  if (localmemory.recordCount() >= MAX_RECORDCOUNT || localmemory.isMemoryFull())
  {
    e1 = WiFi.onStationModeConnected(onSTAConnected);
    e2 = WiFi.onStationModeDisconnected(onSTADisconnected);
    e4 = WiFi.onStationModeGotIP(onSTAGotIP);
    connectToWiFi();
    ActionGoToSleep = false;
  } else {
    ActionGoToSleep = true;
  }

  // connect to BME
  BMEbegin();

  if (ESP.getVcc()<BATTERY_TRESHOLD) ActionBatteryLow = true;
   
  previousMillis = millis();
 }

// ------------------------------------ LOOP -----------------------------------

void loop() {

  if (ActionGoToSleep) {
    GoToSleep();                     // Disconnect WiFi, read BME and deep sleep
  }

  if (ActionMQTT) {
    SaveAllToMQTT();
#if defined(DEBUG_TO_SERIAL)
  DEBUG_PRINTLN("Going to reset memory");
#endif
    localmemory.reset(sizeof(sensordata));
    // updateSketch();
    ActionMQTT = false;
    while (lastAcknPackageId != lastSentPackageId)
    {
      delay(1);
    }
    ActionGoToSleep = true;
  }

  if (ActionWiFiGotIP) {
    setDeviceId();
    setSensorId();
#if defined(DEBUG_ASYNCMQTTCLIENT)
  DEBUG_PRINTF("Connect to Mqtt");
#endif
    MqttConnect();                   // AsyncMQTTClient, client = "esp8266xxxxxx",xxxxxx=ChipId
    ActionWiFiGotIP = false;
  }

  // gotosleep if it takes to long to get WiFi
  if ((unsigned long)(millis() - previousMillis) >= FORCE_SLEEP_MILLIS) {
    ActionGoToSleep = true;
  }

}
