#include <DHT.h>
#include <DHT_U.h>

#include <ESP8266mDNS.h>

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>

// Load the following vars from a config file:
// WIFI_SSID, WIFI_PASSWORD, HOSTNAME, MQTT_SERVER, AUTO_OFF_MS,
// PIR_STATE_TOPIC, RELAY_STATE_TOPIC
#include "config.h"

#define DHT_PIN 5
#define PIR_PIN 4

#define DHTTYPE DHT22

unsigned long last_motion_reported = 0;
unsigned long last_temp_reported = 0;
unsigned long last_reconnect = 0;

int last_motion_state = 0;

WiFiClient espClient;
PubSubClient mqttClient(espClient);
DHT dht(DHT_PIN, DHTTYPE);

void setup() {
  Serial.begin(115200);
  Serial.println("Booting");

  setupWiFi();
  setupOTA();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  connectMQTT();
  pinMode(PIR_PIN, INPUT);
  updateMotion(false);
  dht.begin();
}

void loop() {
  unsigned long currentMS = millis();

  if (WiFi.status() != WL_CONNECTED && (currentMS > last_reconnect + 10000)){
    last_reconnect = currentMS;
    setupWiFi();
  }
  if (!mqttClient.connected() && (currentMS > last_reconnect + 10000)){
    last_reconnect = currentMS;
    connectMQTT();
  }

  ArduinoOTA.handle();
  mqttClient.loop();

  int pir_val = digitalRead(PIR_PIN);

  /*
  if (pir_val != last_motion_state || currentMS > last_motion_reported + MOTION_REPORT_INT_MS){
    updateMotion(pir_val == HIGH);
    last_motion_reported = currentMS;
  }
  */

  if (currentMS > last_temp_reported + TEMP_REPORT_INT_MS){
    float h = dht.readHumidity();
    float f = dht.readTemperature(true);
    updateTemp(f, h);
    last_temp_reported = currentMS;
  }
}

void updateMotion(bool bOn){
  Serial.print("Updating mqtt motion ");
  Serial.print((bOn ? "ON" : "OFF"));
  Serial.println();
  mqttClient.publish(PIR_STATE_TOPIC, (bOn ? "ON" : "OFF"));
}

void updateTemp(float temp, float humidity){
  String tempMessage = "{\"temperature\": "+String(temp,4)+", \"humidity\": "+String(humidity,4);
  tempMessage += "}";
  char* buff = "";//"{\"temperature\":"+String(temp,4)+", \"humidity\":"+String(humidity,4)+"}";
  tempMessage.toCharArray(buff, tempMessage.length()+1);
  mqttClient.publish(TEMP_STATE_TOPIC, buff);
}

void setupWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.hostname(HOSTNAME);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  WiFi.waitForConnectResult();
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
}

void connectMQTT() {
  // Not sure if this will crash without a working WiFi connection
  Serial.println("Attempting connection to MQTT server...");
  mqttClient.setServer(MQTT_SERVER, 1883);
  if (mqttClient.connect(HOSTNAME, MQTT_USERNAME, MQTT_PASSWORD)){
    Serial.println("Connected to MQTT server!");
  }
  else {
    Serial.println("Couldn't connect to MQTT server, error code: ");
    Serial.println(mqttClient.state());
    if (mqttClient.state() < 0){
      delay(5000);
      connectMQTT();
    }
  }
}

void setupOTA() {
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
}
