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

int adc_calibrate_ch;
int adc_calibrate_volt;

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
    Serial.printf("CONFIG SRC: %s\n", cfg_get_backend());
    Serial.printf("Reset Reason: %d / %d\n", rtc_get_reset_reason(0), rtc_get_reset_reason(1));
    Serial.printf("CPU Speed: %d MHz  XTAL: %d MHz  APB: %d Hz\n", getCpuFrequencyMhz(), getXtalFrequencyMhz(), getApbFrequency());
    Serial.printf("Version: %s\n", VERSION);
    Serial.printf("Board: %s\n", ARDUINO_BOARD);
    Serial.printf("Build Date: %s %s  CFG SIZE: %d\n", __DATE__, __TIME__, sizeof(conf_t));
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


void callback(char* topic, byte* payload, unsigned int length) {//用于接收数据
  int l=0;
  int p=1;
  if (strcmp(topic, "ota") == 0)
  {
    WiFiClient ota_client;

    char bufferByte = payload[length];
    payload[length] = 0;
    Serial.printf("Start OTA from URL: %s\n", (char *)payload);
    t_httpUpdate_return ret = ESPhttpUpdate.update(ota_client, (char *)payload);

    payload[length] = bufferByte;

    switch (ret) {
      case HTTP_UPDATE_FAILED:
        sprintf(msg_buf, "{\"ota\":\"%s\"}", ESPhttpUpdate.getLastErrorString().c_str());
        client.publish("status", msg_buf);
        Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
        rebootSystem();
        break;

      case HTTP_UPDATE_NO_UPDATES:
        sprintf(msg_buf, "{\"ota\":\"no updates\"}");
        client.publish("status", msg_buf);
        Serial.println("HTTP_UPDATE_NO_UPDATES");
        rebootSystem();
        break;

      case HTTP_UPDATE_OK:
        sprintf(msg_buf, "{\"ota\":\"success\"}");
        client.publish("status", msg_buf);
        Serial.println("HTTP_UPDATE_OK");
        rebootSystem();
        break;
    }
  } else if (strcmp(topic, "setName") == 0)
  {
      if (length < 32)
      {
          strncpy(dev_name, (const char *)payload, length);
          dev_name[length] = 0;
      }
      CFG_SAVE();
      sendMeta();
  } else if (strcmp(topic, "toggle_realtime") == 0)
  {
      realtime_data = !realtime_data;
      Serial.printf("Toggle Realtime Data: %s\n", realtime_data ? "ON" : "OFF");
  } else if (strcmp(topic, "send") == 0)
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
      return;
  }
}

bool reconnect() {//等待，直到连接上服务器
  static uint32_t disconnectTime = 0;
  static wl_status_t lastWiFiStatus = WL_NO_SHIELD;
  static int retry_failed_count = 0;

  if (WiFi.status() != lastWiFiStatus)
  {
      if (WiFi.status() != WL_CONNECTED) {
          disconnectTime = millis();
      } else 
      {
          disconnectTime = 0;
          IPAddress myAddress = WiFi.localIP();
          Serial.printf("Connected to wifi, IP: ");
          Serial.print(myAddress);
          Serial.printf("\n");
      }
      lastWiFiStatus = WiFi.status();
  }
  if (WiFi.status() != WL_CONNECTED && millis() - disconnectTime >= 30000) {
      Serial.printf("WiFi: Status = %d (Disconnected), RESET SYSTEM\n", WiFi.status());
      rebootSystem();
  }
  if (WiFi.status() == WL_CONNECTED && !client.connected()){
      Serial.printf("Connecting to MQTT %s:%d\n", mqtt_server, port);
      esp_task_wdt_init(30, true);
      esp_task_wdt_add(NULL);
      if (client.connect(mqtt_cls)) {//接入时的用户名，尽量取一个很不常用的用户名
        retry_failed_count = 0;
        Serial.printf("Connect to MQTT success, login by: %s\n",mqtt_cls);
        esp_task_wdt_delete(NULL);
        esp_task_wdt_deinit();
        sendMeta();
        last_rssi = -1;
      } else {
        esp_task_wdt_reset();
        retry_failed_count++;
        Serial.printf("Connect failed, rc=%d\n", client.state());//连接失败
        client.disconnect();
        delay(1000);
        if (retry_failed_count >= 10)
        {
            Serial.printf("MQTT: Reconnect Too many times, RESET SYSTEM\n");
            rebootSystem();
        }
      }
  }
  return WiFi.status() == WL_CONNECTED && client.connected();
}

int uint16_cmpfunc (const void * a, const void * b) {
   return ( *(uint16_t*)a - *(uint16_t*)b );
}

uint16_t adc_filter_value(adc1_channel_t ch)
{
    uint32_t sensor_adc = 0;
    uint16_t sensor_adcValues[64];
    for (int i = 0; i < 64; i++)
    {
        sensor_adcValues[i] = adc1_get_raw(ch);
    }
    qsort(sensor_adcValues, 32, sizeof(uint16_t), uint16_cmpfunc);
    for (int i = 24-8; i < 24+8; i++)
    {
        sensor_adc += sensor_adcValues[i];
    }
    sensor_adc >>= 4;
    return sensor_adc;
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
    if (realtime_data){
//        sprintf(msg_buf, "{\"Volt_A\":%f,\"Volt_B\":%f,\"Volt_C\":%f,\"Atten_A\":%d,\"Atten_B\":%d,\"Atten_C\":%d}", volt_a, volt_b, volt_c, atten_a, atten_b, atten_c);
//        client.publish("callback", msg_buf);
    }
    if (reconnect()) {//确保连上服务器，否则一直等待。
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
