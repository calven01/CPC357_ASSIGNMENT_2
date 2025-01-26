#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_NeoPixel.h>
#include "DHT.h"

// Wi-Fi and MQTT Credentials
const char* WIFI_SSID = "challz";           // Your WiFi SSID
const char* WIFI_PASSWORD = "qwertyuioo";   // Your WiFi password
const char* MQTT_SERVER = "34.31.142.0";   // GCP MQTT server IP
const int MQTT_PORT = 1883;                 // Non-TLS communication port
const char* MQTT_TOPIC = "iot";    // MQTT topic for telemetry data

// Pin Definitions
#define DHTPIN 42                          // DHT11 sensor pin
#define DHTTYPE DHT22                      // DHT22 type
#define LED_PIN 21                         // LED pin for alerts
#define MOTION_SENSOR_PIN 4                // PIR sensor pin
#define GAS_SENSOR_PIN A2                  // MQ2 gas sensor pin
#define NEOPIXEL_PIN 46                    // Onboard NeoPixel pin
#define NUMPIXELS 1
#define GAS_THRESHOLD 1000                 // Gas detection threshold

// Initialize Components
DHT dht(DHTPIN, DHTTYPE);
Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
WiFiClient espClient;
PubSubClient client(espClient);

// Timer Variables
unsigned long lastMsgTime = 0;

void setup_wifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  while (!client.connected()) {
    Serial.println("Attempting MQTT connection...");
    if (client.connect("ESP32Client")) {
      Serial.println("Connected to MQTT server");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" Retrying in 5 seconds...");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(MQTT_SERVER, MQTT_PORT);

  pinMode(LED_PIN, OUTPUT);
  pinMode(MOTION_SENSOR_PIN, INPUT);
  pixels.begin();
  pixels.clear();

  dht.begin();
  Serial.println("System initialized");
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long curTime = millis();
  if (curTime - lastMsgTime > 1000) { // Send telemetry every second
    lastMsgTime = curTime;

    // Read Sensor Data
    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();
    bool motionDetected = digitalRead(MOTION_SENSOR_PIN);
    int gasValue = analogRead(GAS_SENSOR_PIN);

    // Publish telemetry to MQTT
    char payload[256];
    snprintf(payload, sizeof(payload), "{\"temperature\":%.2f,\"humidity\":%.2f,\"motion\":%d,\"gas\":%d}",
             temperature, humidity, motionDetected, gasValue);
    client.publish(MQTT_TOPIC, payload);

    Serial.println(payload);

    // LED and NeoPixel Alerts
    if (motionDetected || gasValue > GAS_THRESHOLD) {
      digitalWrite(LED_PIN, HIGH);
      pixels.setPixelColor(0, pixels.Color(255, 0, 0)); // Red for alert
    } else {
      digitalWrite(LED_PIN, LOW);
      pixels.clear();
    }
    pixels.show();
  }
}
