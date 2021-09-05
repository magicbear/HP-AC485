#include "math.h"
#include <driver/adc.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <HTTPUpdate.h>
#include <rom/rtc.h>
#include <esp_task_wdt.h>
#include "config.h"
#define ESPhttpUpdate httpUpdate

#define MQTT_CLASS "HP-AC485"
#define VERSION "1.0"

// GPIO15: default pull up
#define PIN_RX_EN 15
// GPIO12: default pull down
#define PIN_TX_EN 12
#define PIN_LED_WIFI_OK 2

// NOTICE: NAME not longer then 15 bytes
typedef __attribute__((__packed__)) struct {
    uint8_t  reversion;   // Config
    uint8_t  align1;   // Config
    uint8_t  align2;   // Config
    uint8_t  align3;   // Config
    uint32_t baudrate;
} conf_t;
//  __attribute__ ((aligned (2)))

conf_t config;
#define AUTO_CONF(conf_name, default_value, init) { 32 + offsetof(conf_t, conf_name), sizeof(config.conf_name), default_value,  &config.conf_name, init, #conf_name }
#define AUTO_CONF_FLOAT(conf_name, default_value, init) { 32 + offsetof(conf_t, conf_name), sizeof(config.conf_name), {.f=default_value},  &config.conf_name, init, #conf_name }
#define AUTO_CONF_INT_COMMAND(topic, cmd, field, action)  else if (strcmp(topic, cmd) == 0) { \
  payload[length] = 0; \
  config.field = atoi((char *)payload); \
  cfg_save(def_cfg); \
  sendMeta(); \
  action; \
}
#define AUTO_CONF_FLOAT_COMMAND(topic, cmd, field, action)  else if (strcmp(topic, cmd) == 0) { \
  payload[length] = 0; \
  config.field = atof((char *)payload); \
  cfg_save(def_cfg); \
  sendMeta(); \
  action; \
}

const hp_cfg_t def_cfg[] = {
    AUTO_CONF(reversion, 1, false),
    AUTO_CONF(baudrate, 115200, false),
    {140, 0, {.uint8=0}, &dev_name, true, "name"},                    // STRING: name
    {178, 0, (uint8_t)0, &ssid, true, "ssid"},                    // STRING: SSID
    {210, 0, (uint8_t)0, &password, true, "password"},                   // STRING: WIFI PASSWORD 
    {230, 0, (uint8_t)0, &mqtt_server, true, "mqtt_srv"},        // STRING: MQTT SERVER
    {250, sizeof(uint16_t), (uint16_t)1234, &port, true, "mqtt_port"},             // UINT16: PORT
    {NULL, 0,0, NULL, false, NULL}
};

char mqtt_cls[sizeof(MQTT_CLASS) + 13];
char msg_buf[160];
long last_rssi = -1;
uint32_t last_send_meta;
uint32_t last_send_rssi;
bool realtime_data;
WiFiClient espClient;
PubSubClient client(espClient);

#define rebootSystem ESP.restart

unsigned long execMills = 0;

void setup() {
    pinMode(PIN_LED_WIFI_OK, OUTPUT);
    pinMode(PIN_TX_EN, OUTPUT);
    pinMode(PIN_RX_EN, OUTPUT);
    digitalWrite(PIN_TX_EN, LOW);
    digitalWrite(PIN_RX_EN, HIGH);
    digitalWrite(PIN_LED_WIFI_OK, LOW);
    
    // put your setup code here, to run once:
    WiFi.persistent( false );
    Serial.begin(115200);
    byte mac[6];
    WiFi.macAddress(mac);
    sprintf(mqtt_cls, MQTT_CLASS"-%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    
    int resetCode = rtc_get_reset_reason(0);  
    cfg_begin();
    bool cfgCheck = CFG_CHECK();
    if (!CFG_LOAD())
    {
        CFG_INIT(true);
    }
    if (!cfgCheck)
    {
        // First time startup
    }
    Serial.printf("\n");
    cfg_initalize_info(sizeof(conf_t));
    Serial.printf("Version: %s\n", VERSION);
    Serial.printf("Device ID: %s\n", mqtt_cls+sizeof(MQTT_CLASS));
  
    startupWifiConfigure(def_cfg, msg_buf, sizeof(msg_buf), mqtt_cls);
    Serial.printf("\n");
    Serial.printf("SSID = %s  PASSWORD = %s\n", ssid, password);
    Serial.printf("Startup Baud Rate: %d\n", config.baudrate);

    client.setServer((const char *)mqtt_server, port);//端口号
    client.setCallback(callback); //用于接收服务器接收的数据
    
    Serial.end();
    Serial.begin(config.baudrate,SERIAL_8N1);
    digitalWrite(PIN_RX_EN, LOW);
}


void sendMeta()
{
    last_send_meta = millis();
  // max length = 64 - 21
    char *p = msg_buf + sprintf(msg_buf, "{\"name\":\"%s\",\"board\":\"%s\"}", dev_name, ARDUINO_BOARD);
    client.publish("dev", msg_buf);
}


bool callback(char* topic, byte* payload, unsigned int length) {//用于接收数据
  if (strcmp(topic, "send") == 0)
  {
      digitalWrite(PIN_RX_EN, HIGH);
      delayMicroseconds(1);
      digitalWrite(PIN_TX_EN, HIGH);
      delayMicroseconds(1);
      Serial.write(payload, length);
      Serial.flush();
      delayMicroseconds(1);
      digitalWrite(PIN_TX_EN, LOW);
      delayMicroseconds(1);
      digitalWrite(PIN_RX_EN, LOW);
      return true;
  }
  return false;
}

void on_mqtt_connected()
{
    sendMeta();
    last_rssi = -1;
}

void loop() {
    uint32_t currentMillis = millis();

    msg_buf[0] = 0;
    char *p = msg_buf;
    while (Serial.available())
    {
        *p++ = Serial.read();
        if (p >= msg_buf + sizeof(msg_buf)) break;
    }
    if (p != msg_buf)
    {
        digitalWrite(PIN_RX_EN, HIGH);
        delayMicroseconds(1);
        digitalWrite(PIN_TX_EN, HIGH);
        delayMicroseconds(1);
        Serial.write((uint8_t *)msg_buf, (uint8_t)(p-msg_buf));
        Serial.flush();
        delayMicroseconds(1);
        digitalWrite(PIN_TX_EN, LOW);
        delayMicroseconds(1);
        digitalWrite(PIN_RX_EN, LOW);
        sprintf(msg_buf, "{\"payload\":%d}", p-msg_buf);
        client.publish("callback", msg_buf);
    }
    if (check_connect(mqtt_cls, &client, on_mqtt_connected)) {//确保连上服务器，否则一直等待。
      client.loop();//MUC接收数据的主循环函数。
      long rssi = WiFi.RSSI();
      static uint32_t last_send_state = 0;
       if (currentMillis - last_send_state >= 10000 || (abs(rssi - last_rssi) >= 3 && currentMillis - last_send_rssi >= 15000))
       {
          last_send_rssi = currentMillis;
          last_send_state = currentMillis;
          last_rssi = rssi;
          sprintf(msg_buf, "{\"rssi\":%ld,\"version\":\"%s\",\"boot\":%ld}", rssi, VERSION, currentMillis);
          client.publish("status", msg_buf);
       }
       if (currentMillis - last_send_meta >= 60000)
       {
//          Serial.printf("PING %ld  WiFI: %d\n", currentMillis, WiFi.status());
          sendMeta();
       }
       digitalWrite(PIN_LED_WIFI_OK, HIGH);
    } else 
    {
        digitalWrite(PIN_LED_WIFI_OK, LOW);
    }
}
