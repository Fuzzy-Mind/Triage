#include <WiFi.h>
extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}
#include <AsyncMqttClient.h>
#include <HardwareSerial.h>

#define WIFI_SSID "ErtuncOzcanKat2"
#define WIFI_PASSWORD "ertarge2017"

// Raspberry Pi Mosquitto MQTT Broker
#define MQTT_HOST IPAddress(46, 101, 94, 104)
// For a cloud MQTT broker, type the domain name
//#define MQTT_HOST "triageserver.com"
#define MQTT_PORT 1883

// Temperature MQTT Topic
#define MQTT_PUB_TEMP "sensorboard/temperature"
#define MQTT_PUB_SPO2 "sensorboard/spo2"
#define MQTT_PUB_PULSERATE "sensorboard/pulserate"
#define MQTT_PUB_PERFUSIONINDEX "sensorboard/perfusionindex"

#define MQTT_PUB_NIBPSYSTOLIC "nibp/systolic"
#define MQTT_PUB_NIBPDIASTOLIC "nibp/diastolic"
#define MQTT_PUB_NIBPMAP "nibp/map"
#define MQTT_PUB_NIBPPR "nibp/pr"
#define MQTT_PUB_NIBPSTATUS "nibp/status"

// Temperature value
float temp = 0;
float spo2 = 0;
float perfusionindex = 0;
byte sensorData[16];
byte nibpData[16];
byte incomingByte = 0;
byte nibpIncoming = 0;

uint16_t sSpo2 = 0;
uint16_t sPulseRate = 0;
uint16_t sPerfusionIndex = 0;
uint16_t sTemp = 0;

uint16_t systolic = 0;
uint16_t diastolic = 0;
uint16_t meanap = 0;
uint16_t pr = 0;
String nibpStatus = "";

AsyncMqttClient mqttClient;

HardwareSerial SerialPort(1);

TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

unsigned long previousMillis = 0;   // Stores last time temperature was published
const long interval = 10000;        // Interval at which to publish sensor readings

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void setup() {

  Serial.begin(115200);
  SerialPort.begin(9600, SERIAL_8N1, 4, 2); // 4 rx - 2 tx
  Serial2.begin(9600);

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  //mqttClient.onSubscribe(onMqttSubscribe);
  //mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  // If your broker requires authentication (username and password), set them below
  mqttClient.setCredentials("triage", "ak47");
  connectToWifi();
}

void loop() {
  unsigned long currentMillis = millis();
  Serial2.flush();
  SerialPort.flush();
  
  memset(sensorData, 0, 16);
  if(Serial2.available()>0){
    incomingByte = Serial2.read();
    if(incomingByte == 164){
      //Serial.println("Paket Basi");
      Serial2.readBytes(sensorData, 9);
      if(sensorData[8] == 74){
        //Serial.println("Paket Dogru");
        
        sSpo2 = (sensorData[0]<<8)+sensorData[1];
        sPulseRate = (sensorData[2]<<8)+sensorData[3];
        sPerfusionIndex = (sensorData[4]<<8)+sensorData[5];
        sTemp = (sensorData[6]<<8)+sensorData[7];
        if(sTemp<16){sTemp = 0;}

        temp = float(sTemp)/100;
        spo2 = float(sSpo2)/10;
        perfusionindex = float(sPerfusionIndex)/1000;
        
      }
      else{
        Serial.println("Paket Hatalı");
        for(int i=0; i<9; i++){
          Serial.println(sensorData[i], HEX);
        }            
      }
    }
  }
  
  memset(nibpData, 0, 16);
  if(SerialPort.available()>0){
    nibpIncoming = SerialPort.read();
    if(nibpIncoming == 0xf1){
      SerialPort.readBytes(nibpData, 9);
      /*for(int i=0; i<9; i++){
        Serial.print("Nibp Gelen : ");
        Serial.println(nibpData[i]);
      }*/
      if(nibpData[8] == 0x1f){
        systolic = (nibpData[1] << 8) | (nibpData[0]);
        diastolic = (nibpData[3] << 8) | (nibpData[2]);
        meanap = (nibpData[5] << 8) | (nibpData[4]);
        pr = (nibpData[7] << 8) | (nibpData[6]);
        nibpStatus = "Measurement Successful";
      }
      else if(nibpData[8] == 0x1e){
        systolic = 0;
        diastolic = 0;
        meanap = 0;
        pr = 0;
        nibpStatus = "Measurement Failed";

      }
      else{
        systolic = 0;
        diastolic = 0;
        meanap = 0;
        pr = 0;
        nibpStatus = "Wrong Package";
      }
    }
  }
  
  // Every X number of seconds (interval = 10 seconds) 
  // it publishes a new MQTT message
  if (currentMillis - previousMillis >= interval) {
    // Save the last time a new reading was published
    previousMillis = currentMillis;
    // New temperature readings
    // Temperature in Celsius degrees
    
    /**/

    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_TEMP, 1, true, String(temp).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId: ", MQTT_PUB_TEMP);
    Serial.println(packetIdPub1);
    Serial.printf("Message: %.2f /n", temp);

    uint16_t packetIdPub2 = mqttClient.publish(MQTT_PUB_SPO2, 1, true, String(spo2).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId: ", MQTT_PUB_SPO2);
    Serial.println(packetIdPub2);
    Serial.printf("Message: %.2f /n", spo2);

    uint16_t packetIdPub3 = mqttClient.publish(MQTT_PUB_PULSERATE, 1, true, String(sPulseRate).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId: ", MQTT_PUB_PULSERATE);
    Serial.println(packetIdPub3);
    Serial.printf("Message: %d /n", sPulseRate);

    uint16_t packetIdPub4 = mqttClient.publish(MQTT_PUB_PERFUSIONINDEX, 1, true, String(perfusionindex).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId: ", MQTT_PUB_PERFUSIONINDEX);
    Serial.println(packetIdPub4);
    Serial.printf("Message: %.2f /n", perfusionindex);

    if(nibpStatus != ""){
      uint16_t packetIdPub5 = mqttClient.publish(MQTT_PUB_NIBPSTATUS, 1, true, nibpStatus.c_str());
      Serial.println(nibpStatus);
  
      uint16_t packetIdPub6 = mqttClient.publish(MQTT_PUB_NIBPSYSTOLIC, 1, true, String(systolic).c_str());
      Serial.print("Systolic : ");
      Serial.println(systolic);
      
      uint16_t packetIdPub7 = mqttClient.publish(MQTT_PUB_NIBPDIASTOLIC, 1, true, String(diastolic).c_str());
      Serial.print("Diastolic : ");
      Serial.println(diastolic);
     
      uint16_t packetIdPub8 = mqttClient.publish(MQTT_PUB_NIBPMAP, 1, true, String(meanap).c_str());
      Serial.print("Mean Arterial Pressure : ");
      Serial.println(meanap);
      
      uint16_t packetIdPub9 = mqttClient.publish(MQTT_PUB_NIBPPR, 1, true, String(pr).c_str());
      Serial.print("Pulse Rate : ");
      Serial.println(pr);
    }
  }
}
