#include <Arduino.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <GyverNTP.h>
#include <HTTPClient.h>
#include <Mhz19.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <Wire.h>

#include "Credentials.h"
#include "esp_wifi.h"


IPAddress server(192, 168, 31, 71);
WiFiClient client;
PubSubClient mqtt(client);

[[nodiscard]] inline bool isWiFiConnected() {
  return WiFi.status() == WL_CONNECTED;
}

void Callback(char* topic, byte* payload, unsigned int length) {
  char json[length];
  memcpy(json, payload, length);
  
  StaticJsonDocument<300> doc;
  DeserializationError error = deserializeJson(doc, json);

  if (error) {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    return;
  }
}

void MqttHandler(void* pvParameters) {
  while (1) {
    if (isWiFiConnected()) {
      if (!mqtt.connected()) {
        while (!mqtt.connected()) {
          if (mqtt.connect("co2_sensor", MQTT_LOGIN.c_str(), MQTT_PASSWORD.c_str()), 1, 0, "CO2 sensor Andrey's room") {
            break;
          }
        }
      }
      mqtt.loop();
    }
    vTaskDelay(pdMS_TO_TICKS(30));
  }
  vTaskDelete(NULL);
}

void CheckInternerConnection(void* pvParameters) {
  WiFi.mode(WIFI_STA);
  esp_wifi_set_ps(WIFI_PS_MAX_MODEM);
  WiFi.begin(SSID, PASSWORD);

  delay(3000);

  while (1) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("Reconnecting to WiFi...");
      // WiFi.disconnect();
      WiFi.reconnect();   
    }
    else {
      Serial.println("WiFi is okay");
    }
    vTaskDelay(pdMS_TO_TICKS(3000));
  }
  vTaskDelete(NULL);
}

void PubishCO2Data(void* pvParameters) {
  Mhz19 sensor;
  HardwareSerial hw_serial(1);

  hw_serial.begin(9600, SERIAL_8N1, 16, 17);

  sensor.begin(&hw_serial);
  sensor.setMeasuringRange(Mhz19MeasuringRange::Ppm_5000);
  sensor.disableAutoBaseCalibration();

  vTaskDelay(pdMS_TO_TICKS(20000));

  while (1) {
    int carbonDioxide = sensor.getCarbonDioxide();

    if (carbonDioxide >= 0 && isWiFiConnected() && mqtt.connected()) {
      mqtt.publish("co2_sensor/value",
                    String("{\"co2_ppm\": " + String(carbonDioxide) + "}").c_str(),
                    true);
      vTaskDelay(pdMS_TO_TICKS(29000));
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
  }

  vTaskDelete(NULL);
}

void setup() {
  Serial.begin(115200);

  mqtt.setServer(server, 1883);
  mqtt.setCallback(Callback);

  xTaskCreatePinnedToCore(
    CheckInternerConnection,
    "InternetConnection",
    4096,
    NULL,
    1,
    NULL,
    0
  );

  xTaskCreatePinnedToCore(
    MqttHandler,
    "MqttPublisher",
    4096,
    NULL,
    1,
    NULL,
    0
  );

  xTaskCreatePinnedToCore(
    PubishCO2Data,
    "PubishCO2Data",
    4096,
    NULL,
    1,
    NULL,
    0
  );

}

void loop() {

}
