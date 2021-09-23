#include "math.h"
#include <driver/adc.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <HTTPUpdate.h>
#include <rom/rtc.h>
#include <esp_task_wdt.h>
#include "config.h"
#include <ModbusMaster.h>

#if defined(CORE_DEBUG_LEVEL) && CORE_DEBUG_LEVEL > 0
#error "HP-485 Using system UART to connect RS-485, Please change settings to disable system log (Tools->Core Debug Level->None)"
#endif

#define MQTT_CLASS "HP-AC485"

// 使能IRACC 485 RTU程序
//#define RS485_IRACC

#define VERSION "1.0"

// GPIO15: default pull up
#define PIN_RX_EN 15
// GPIO12: default pull down
#define PIN_TX_EN 12
#define PIN_LED_WIFI_OK 2

// NOTICE: NAME not longer then 15 bytes
typedef struct {
    uint8_t  reversion;   // Config
    uint32_t query_interval;  // Config To Query AC
    uint32_t query_acids;     // Config
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
    AUTO_CONF(query_interval, 30000, false),  // 30 seconds
    AUTO_CONF(query_acids, 127, false),       // ACIDs 0-7
    {140, 0, {.uint8 = 0}, &dev_name, true, "name"},                  // STRING: name
    {178, 0, (uint8_t)0, &ssid, true, "ssid"},                    // STRING: SSID
    {210, 0, (uint8_t)0, &password, true, "password"},                   // STRING: WIFI PASSWORD
    {230, 0, (uint8_t)0, &mqtt_server, true, "mqtt_srv"},        // STRING: MQTT SERVER
    {250, sizeof(uint16_t), (uint16_t)1234, &port, true, "mqtt_port"},             // UINT16: PORT
    {NULL, 0, 0, NULL, false, NULL}
};

char mqtt_cls[sizeof(MQTT_CLASS) + 13];
char msg_buf[160];
uint32_t last_send_meta;
bool realtime_data;
WiFiClient espClient;
PubSubClient client(espClient);
ModbusMaster node;

unsigned long execMills = 0;

uint8_t acSpeed[64];

void preTransmission()
{
    digitalWrite(0, HIGH);
    digitalWrite(PIN_RX_EN, HIGH);
    delayMicroseconds(1);
    digitalWrite(PIN_TX_EN, HIGH);
}

void postTransmission()
{
#ifdef SOC_UART_FIFO_LEN
    // Fix ESP32 >= 2.0.0 Serial.flush invalid
    while (SOC_UART_FIFO_LEN != Serial.availableForWrite())
    {
        yield();
        delayMicroseconds(1);
    }
#else
    Serial.flush(true);
#endif
    delayMicroseconds((int)(10000000. / config.baudrate * 1));  // Wait for 1 bytes to send
    digitalWrite(PIN_TX_EN, LOW);
    digitalWrite(PIN_RX_EN, LOW);
}

void setup() {
    pinMode(PIN_LED_WIFI_OK, OUTPUT);
    pinMode(PIN_TX_EN, OUTPUT);
    pinMode(PIN_RX_EN, OUTPUT);
    digitalWrite(PIN_TX_EN, LOW);
    digitalWrite(PIN_RX_EN, HIGH);  // Disable RX
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
    Serial.printf("Device ID: %s\n", mqtt_cls + sizeof(MQTT_CLASS));

    startupWifiConfigure(def_cfg, msg_buf, sizeof(msg_buf), mqtt_cls);
    Serial.printf("\n");
    Serial.printf("SSID = %s  PASSWORD = %s\n", ssid, password);
    Serial.printf("Startup Baud Rate: %d\n", config.baudrate);

    init_mqtt_client(&client, callback, onCfgUpdated);

    Serial.end();
    Serial.begin(config.baudrate, SERIAL_8N1);
    node.begin(1, Serial);
    // Callbacks allow us to configure the RS485 transceiver correctly
    node.preTransmission(preTransmission);
    node.postTransmission(postTransmission);
    postTransmission(); // Enable Receiver, TXD RXD is using the same of TTL Serial
}


void onCfgUpdated()
{
    CFG_SAVE();
    sendMeta();
}


void sendMeta()
{
    last_send_meta = millis();
    // max length = 64 - 21
    char *p = msg_buf + sprintf(msg_buf, "{\"name\":\"%s\",\"board\":\"%s\",\"baudrate\":%ld,\"query_interval\":%ld,\"query_acids\":%ld}", dev_name, ARDUINO_BOARD,
      config.baudrate, config.query_interval, config.query_acids);
    client.publish("dev", msg_buf);
}


void processRTUResult(uint16_t result, uint16_t address, uint16_t qty, int line)
{
    switch (result)
    {
      case node.ku8MBSuccess:
      {
          //InputRegister 0x0001
          uint16_t *data = (uint16_t *)calloc(sizeof(uint16_t), qty);
          for (int i = 0; i < qty; i++)
              data[i] = node.getResponseBuffer(i);
          client.publish("data_callback", (uint8_t *)data, (unsigned int)(qty * sizeof(uint16_t)), false);
          free(data);
          break;
      }
      case node.ku8MBIllegalFunction:
          client.publish("error", "Illegal Function");
          printLog(LOG_ERROR, "Illegal function on address %d, qty: %d on main.ino:%d", address, qty, line);
          break;
      case node.ku8MBIllegalDataAddress:
          client.publish("error", "Illegal Data Address");
          printLog(LOG_ERROR, "Illegal Data Address on address %d, qty: %d on main.ino:%d", address, qty, line);
          break;
      case node.ku8MBIllegalDataValue:
          client.publish("error", "Illegal Data Value");
          printLog(LOG_ERROR, "Illegal Data Value on address %d, qty: %d on main.ino:%d", address, qty, line);
          break;
      case node.ku8MBSlaveDeviceFailure:
          client.publish("error", "Slave Device Failure");
          printLog(LOG_ERROR, "Slave Device Failure on address %d, qty: %d on main.ino:%d", address, qty, line);
          break;
       case node.ku8MBInvalidSlaveID:
          client.publish("error", "Invalid Slave ID");
          printLog(LOG_ERROR, "Invalid Slave ID on address %d, qty: %d on main.ino:%d", address, qty, line);
          break;
       case node.ku8MBInvalidFunction:
          client.publish("error", "Invalid Function");
          printLog(LOG_ERROR, "Invalid Function on address %d, qty: %d on main.ino:%d", address, qty, line);
          break;
       case node.ku8MBResponseTimedOut:
          client.publish("error", "Response Timed Out");
          printLog(LOG_ERROR, "Response timed out on address %d, qty: %d on main.ino:%d", address, qty, line);
          break;
       case node.ku8MBInvalidCRC:
          client.publish("error", "Invalid CRC");
          printLog(LOG_ERROR, "Invalid CRC on address %d, qty: %d on main.ino:%d", address, qty, line);
          break;
       default:
          sprintf(msg_buf, "Modbus RTU ERROR: %d", result);
          client.publish("error", msg_buf);
          printLog(LOG_ERROR, "Unknow Error %d on address %d, qty: %d on main.ino:%d", result, address, qty, line);
    }
}


typedef enum {
    None,
    readCoils,
    readDiscreteInputs,
    readHoldingRegisters,
    readInputRegisters,
    writeSingleCoil,
    writeSingleRegister,
    writeMultipleCoils,
    writeMultipleRegisters
} rs485_opt_t;

#ifdef RS485_IRACC
void HAVC_parseStatus(uint8_t acId, uint16_t *data)
{
    if ((config.query_acids >> acId) & 0x1 == 0) return;
    char topic[32];
    sprintf(topic, "dev.%d", acId);
    sprintf(msg_buf, "{\"cls\":\"HP-HVAC485\"}");
    client.publish(topic, msg_buf);

    acSpeed[acId] = data[0]>>12;
    sprintf(topic, "status.%d", acId);
    sprintf(msg_buf, "{\"Power\":%s,\"Speed\":%d,\"AirFlow\":%d,\"cleanFilter\":%d,\"Mode\":%d,\"temperature\":%.1f,\"ErrorCode\":%d,\"inside_temp\":%.1f,\"inside_sensor_ok\":%s}",
        (data[0] & 0x1) ? "true" : "false",
        data[0] >> 12,
        data[0] >> 4 & 0xf,
        data[1] >> 8,
        data[1] & 0xf,
        data[2] / 10.0,
        data[3],
        (data[4] & 0x7ff) / 10.0,
        (data[4] >> 11 == 0) && data[5] == 0x8000 ? "true" : "false"
        );
    client.publish(topic, msg_buf);
}

void HAVC_getStatus(uint8_t acId, uint8_t getCount = 1)
{
    uint16_t result = node.readInputRegisters(0x07D0 + acId * 6, 6 * getCount);
    if (result == node.ku8MBSuccess)
    {
        //InputRegister 0x0001
        uint16_t *data = (uint16_t *)calloc(6 * getCount, sizeof(uint16_t));
        for (int i = 0; i < 6 * getCount; i++)
            data[i] = node.getResponseBuffer(i);
        for (int i = 0; i < getCount; i++)
          HAVC_parseStatus(acId + i, data + i * 6);
        free(data);
    } else {
        processRTUResult(result, 0x07D0 + acId * 6, 6 * getCount, __LINE__);
    }
}
#endif

bool callback(const char* topic, byte* payload, unsigned int length) {//用于接收数据
    if (strcmp(topic, "ota") == 0)
    {
        httpUpdate.setLedPin(PIN_LED_WIFI_OK, LOW);
        return false;         // To process parent on_message
    } else if (strcmp(topic, "send") == 0)
    {
        preTransmission();
        delayMicroseconds(1);
        Serial.write(payload, length);
        Serial.flush();
        postTransmission();
        return true;
    }
#ifdef RS485_IRACC
    else if (strcmp(topic, "havc_status") == 0 || strcmp(topic, "getStatus") == 0)
    {
        payload[length] = 0;
        HAVC_getStatus(atoi((const char *)payload));
    } else if (strcmp(topic, "set_temperature") == 0)
    {
        payload[length] = 0;
        uint8_t acId;
        float   temperature;
        char dtopic[32];
        if (sscanf((const char *)payload, "%d|%f", &acId, &temperature) == 2)
        {
            uint16_t result = node.writeSingleRegister(0x07D2 + acId * 3, (uint16_t)(temperature * 10));
            if (result == node.ku8MBSuccess)
            {
                sprintf(dtopic, "status.%d", acId);
                sprintf(msg_buf, "{\"temperature\":%.1f}",temperature);
                client.publish(dtopic, msg_buf);
            } else {
                processRTUResult(result, 0x07D2 + acId * 3, 3, __LINE__);
            }
        } else {
            client.publish("error", "Parse Error, Non support format");
            printLog(LOG_ERROR, "Invalid argument format, address: %u, temperature: %f\n", acId, temperature);
            return true;
        }
    } else if (strcmp(topic, "set_mode") == 0)
    {
        payload[length] = 0;
        uint8_t acId;
        uint8_t ac_mode;
        char dtopic[32];
        if (sscanf((const char *)payload, "%d|%d", &acId, &ac_mode) == 2)
        {
            uint16_t result = node.writeSingleRegister(0x07D1 + acId * 3, (uint16_t)(ac_mode));
            if (result == node.ku8MBSuccess)
            {
                sprintf(dtopic, "status.%d", acId);
                sprintf(msg_buf, "{\"Mode\":%d}",ac_mode);
                client.publish(dtopic, msg_buf);
            } else {
                processRTUResult(result, 0x07D1 + acId * 3, 3, __LINE__);
            }
        } else {
            client.publish("error", "Parse Error, Non support format");
            printLog(LOG_ERROR, "Invalid argument format, address: %u, ac_mode: %d\n", acId, ac_mode);
            return true;
        }
    } else if (strcmp(topic, "set_power") == 0)
    {
        payload[length] = 0;
        uint8_t acId;
        uint8_t ac_power;
        uint8_t ac_speed;
        char dtopic[32];
        
        int parseCount = sscanf((const char *)payload, "%d|%d,%d", &acId, &ac_power, &ac_speed);
        printLog(LOG_INFO, "Set AC ID: %d Power = %d Speed = %d Arguments: %d\n", acId, ac_power, ac_speed, parseCount);
        if (parseCount == 2 || parseCount == 3)
        {
            uint16_t result = node.writeSingleRegister(0x07D0 + acId * 3, (uint16_t)((parseCount == 2 ? 0xff00 : ac_speed << 12) | (ac_power ? 0x61 : 0x60)));
            if (result == node.ku8MBSuccess)
            {
                sprintf(dtopic, "status.%d", acId);
                acSpeed[acId] = ac_speed;
                if (parseCount == 2)
                {
                    sprintf(msg_buf, "{\"Power\":%s}", ac_power ? "true" : "false");
                } else {
                    sprintf(msg_buf, "{\"Power\":%s, \"Speed\": %d}", ac_power ? "true" : "false", ac_speed);
                }
                client.publish(dtopic, msg_buf);
            } else {
                processRTUResult(result, 0x07D0 + acId * 3, 3, __LINE__);
            }
            return true;
        } else {
            client.publish("error", "Parse Error, Non support format");
            printLog(LOG_ERROR, "Invalid argument format, address: %u, power: %d, speed: %d\n", acId, ac_power, ac_speed);
            return true;
        }
    } else if (strcmp(topic, "set_fanspeed") == 0)
    {
        payload[length] = 0;
        uint8_t acId;
        uint8_t ac_power;
        uint8_t ac_speed;
        char dtopic[32];
        
        int parseCount = sscanf((const char *)payload, "%d|%d", &acId, &ac_speed);
        printLog(LOG_INFO, "Set AC ID: %d Speed = %d Arguments: %d\n", acId, ac_power, ac_speed, parseCount);
        if (parseCount == 2)
        {
            uint16_t result = node.writeSingleRegister(0x07D0 + acId * 3, (uint16_t)((ac_speed << 12) | 0xff));
            if (result == node.ku8MBSuccess)
            {
                sprintf(dtopic, "status.%d", acId);
                acSpeed[acId] = ac_speed;
                sprintf(msg_buf, "{\"Speed\": %d}", ac_speed);
                client.publish(dtopic, msg_buf);
            } else {
                processRTUResult(result, 0x07D0 + acId * 3, 3, __LINE__);
            }
            return true;
        } else {
            client.publish("error", "Parse Error, Non support format");
            printLog(LOG_ERROR, "Invalid argument format, address: %u, power: %d, speed: %d\n", acId, ac_power, ac_speed);
            return true;
        }
    }
#endif
    else if (strcmp(topic, "reset") == 0)
    {
        if (length > 0 && payload[0] == '1')
        {
            cfg_reset(def_cfg);
        } else
        {
            CFG_INIT(false);
        }
        printLog(LOG_INFO, "Reset System\n");
    }
    AUTO_CONF_INT_COMMAND(topic, "set_query_interval", query_interval, )
    AUTO_CONF_INT_COMMAND(topic, "set_query_acids", query_acids, )
    AUTO_CONF_INT_COMMAND(topic, "set_baudrate", baudrate, {
        Serial.end();
        Serial.begin(config.baudrate, SERIAL_8N1);
    }) else {
      rs485_opt_t opt = None;
      if (strcmp(topic, "readCoils") == 0) opt = readCoils;
      else if (strcmp(topic, "readDiscreteInputs") == 0) opt = readDiscreteInputs;
      else if (strcmp(topic, "readHoldingRegisters") == 0) opt = readHoldingRegisters;
      else if (strcmp(topic, "readInputRegisters") == 0) opt = readInputRegisters;
      else if (strcmp(topic, "writeSingleCoil") == 0) opt = writeSingleCoil;
      else if (strcmp(topic, "writeSingleRegister") == 0) opt = writeSingleRegister;
      else if (strcmp(topic, "writeMultipleCoils") == 0) opt = writeMultipleCoils;
      else if (strcmp(topic, "writeMultipleRegisters") == 0) opt = writeMultipleRegisters;
      if (opt != None)
      {
          uint16_t address, qty;
          payload[length] = 0;
          if (sscanf((const char *)payload, "%u,%u", &address, &qty) == 2)
          {
              if (opt == writeMultipleCoils || opt == writeMultipleRegisters)
              {
                  client.publish("error", "Parse Error, Non support format for Multiple write, please use <address>|<raw value>");
                  printLog(LOG_ERROR, "Invalid argument format, address: %u, qty: %u\n", address, qty);
                  return true;
              }
          } else if (sscanf((const char *)payload, "%u%c", &address, &qty) == 2 && qty == '|')
          {
              if (opt != writeMultipleCoils && opt != writeMultipleRegisters)
              {
                  client.publish("error", "Parse Error, This format only support for Multiple write, please use <address>|<raw value>");
                  printLog(LOG_ERROR, "Invalid argument format, address: %u, qty: %u\n", address, qty);
                  return true;
              }
              uint16_t *p = (uint16_t *)(strchr((const char *)payload, '|')+1);
              qty = (length - ((byte *)p-payload)) / sizeof(uint16_t);
              for (int i = 0; i < qty; i++)
                  node.setTransmitBuffer(i, p[i]);
              printLog(LOG_INFO, "Parse argument, address: %u, qty: %u\n", address, qty);
          } else {
              client.publish("error", "Parse Error");
              printLog(LOG_ERROR, "Parse argument failed, address: %u, qty: %u\n", address, qty);
              return true;
          }
          uint8_t result;
          if (opt == readInputRegisters)
          {
              result = node.readInputRegisters(address, qty);
              processRTUResult(result, address, qty, __LINE__);
          } else if (opt == readCoils)
          {
              result = node.readCoils(address, qty);
              processRTUResult(result, address, qty, __LINE__);
          } else if (opt == readDiscreteInputs)
          {
              result = node.readDiscreteInputs(address, qty);
              processRTUResult(result, address, qty, __LINE__);
          } else if (opt == writeSingleCoil)
          {
              result = node.writeSingleCoil(address, qty);
              processRTUResult(result, address, qty, __LINE__);
          } else if (opt == writeSingleRegister)
          {
              result = node.writeSingleRegister(address, qty);
              processRTUResult(result, address, qty, __LINE__);
          } else if (opt == writeMultipleCoils)
          {
              result = node.writeMultipleCoils(address, qty);
              processRTUResult(result, address, qty, __LINE__);
          } else if (opt == writeMultipleRegisters)
          {
              result = node.writeMultipleRegisters(address, qty);
              processRTUResult(result, address, qty, __LINE__);
          }
          return true;
        }
    }
    return false;
}

void on_mqtt_connected()
{
    sendMeta();
}

void loop() {
    uint32_t currentMillis = millis();
    static uint32_t last_query = 0;

    if (check_connect(mqtt_cls, &client, on_mqtt_connected)) {//确保连上服务器，否则一直等待。
        msg_buf[0] = 0;
        char *p = msg_buf;
        while (Serial.available())
        {
            *p++ = Serial.read();
            if (p >= msg_buf + sizeof(msg_buf)) break;
        }
        if (p != msg_buf)
        {
//            Serial.printf("Send %d", p-msg_buf);
            //Serial.write(msg_buf, p-msg_buf);
            printLog(LOG_INFO, "Received ASCII data length: %d\n", (unsigned int)(p - msg_buf));
            client.publish("data_callback", (uint8_t *)msg_buf, (unsigned int)(p - msg_buf), false);
        }
        client.loop();//MUC接收数据的主循环函数。
#ifdef RS485_IRACC
        if (config.query_interval != 0 && (last_query == 0 || currentMillis - last_query >= config.query_interval))
        {
            last_query = currentMillis;
            uint32_t tmpACIDs = config.query_acids;
            uint8_t startACID = 0xff;
            uint8_t endACID = 0;
            if (config.query_acids > 0)
            {
                for (int i = 0; i < 32 && tmpACIDs != 0; i++)
                {
                    if (tmpACIDs & 0x1)
                    {
                        if (startACID == 0xff) startACID = i;
                        endACID = i;
                    }
                    tmpACIDs >>= 1;
                }
                uint8_t doEndACID = startACID + 3 >= endACID ? endACID : startACID + 3;
                while (true)
                {
//                    printLog(LOG_DEBUG, "Query AC %d to %d\n", startACID, doEndACID);
                    HAVC_getStatus(startACID, doEndACID - startACID + 1);
                    startACID = doEndACID + 1;
                    if (doEndACID == endACID) break;
                    doEndACID = startACID + 3 >= endACID ? endACID : startACID + 3;
                }                
            }
        }
#endif
        static uint32_t last_send_state = 0;
        if (currentMillis - last_send_state >= 60000)
        {
            long rssi = WiFi.RSSI();
            last_send_state = currentMillis;
            sprintf(msg_buf, "{\"rssi\":%ld,\"version\":\"%s\",\"boot\":%ld}", rssi, VERSION, time(NULL));
            client.publish("status", msg_buf);
        }
        digitalWrite(PIN_LED_WIFI_OK, HIGH);
    } else
    {
        digitalWrite(PIN_LED_WIFI_OK, LOW);
    }
}
