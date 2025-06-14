#include <Arduino.h>
#include <WiFi.h>
#include "esp32-hal-ledc.h"
#include <Firebase_ESP_Client.h>
#include <addons/TokenHelper.h>
#include <addons/RTDBHelper.h>

// WiFi Credentials
#define WIFI_SSID "TUF"
#define WIFI_PASSWORD "mot99pass1"

// Firebase Credentials
#define API_KEY ""
#define DATABASE_URL ""
#define USER_EMAIL ""
#define USER_PASSWORD ""

// Pins
#define trigPin1 5
#define echoPin1 18
#define trigPin2 17
#define echoPin2 16
#define ledPin 23
#define ldrPin 22

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

SemaphoreHandle_t countMutex;
int vehicleCount = 0;
bool lightHoldActive = false;
unsigned long vehicleDetectedTime = 0;

int getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);  
  return duration * 0.034 / 2;
}

void connectWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\nWiFi connected!");
}

void Firebase_Store(String path, String msg) {
  if (Firebase.RTDB.setString(&fbdo, path, msg)) {
    Serial.println("Firebase update: " + path + " = " + msg);
  } else {
    Serial.print("Firebase error: ");
    Serial.println(fbdo.errorReason());
  }
}

// Task 1: Entry Sensor
void SensorEntryTask(void *pvParameters) {
  for (;;) {
    int distance = getDistance(trigPin1, echoPin1);
    if (distance > 2 && distance < 15) {
      xSemaphoreTake(countMutex, portMAX_DELAY);
      vehicleCount++;
      xSemaphoreGive(countMutex);
      vehicleDetectedTime = millis();
      lightHoldActive = true;
      Firebase_Store("/Streetlight/Vehicle", "Detected");
      Serial.println("Vehicle Entered");
      vTaskDelay(pdMS_TO_TICKS(1000)); // debounce
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

// Task 2: Exit Sensor
void SensorExitTask(void *pvParameters) {
  for (;;) {
    int distance = getDistance(trigPin2, echoPin2);
    if (distance > 2 && distance < 15) {
      xSemaphoreTake(countMutex, portMAX_DELAY);
      if (vehicleCount > 0) vehicleCount--;
      bool noVehicles = (vehicleCount == 0);
      xSemaphoreGive(countMutex);
      if (noVehicles) {
        Firebase_Store("/Streetlight/Vehicle", "None");
        Serial.println("Vehicle Exited");
      }
      vTaskDelay(pdMS_TO_TICKS(1000)); // debounce
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

// Task 3: LED Control (LDR + Vehicle Count)
void LedTask(void *pvParameters) {
  ledcSetup(0, 5000, 8); // Channel 0, 5kHz PWM, 8-bit resolution
  ledcAttachPin(ledPin, 0);

  for (;;) {
    int ldrValue = analogRead(ldrPin);
    int brightness = map(ldrValue, 0, 4095, 50, 255);

    xSemaphoreTake(countMutex, portMAX_DELAY);
    int count = vehicleCount;
    xSemaphoreGive(countMutex);

    if (ldrValue < 1000) {
      Firebase_Store("/Streetlight/LDR", "Night");
      if (count > 0 || lightHoldActive) {
        ledcWrite(0, brightness);  
        lightHoldActive = millis() - vehicleDetectedTime <= 5000;
      } else {
        ledcWrite(0, 50);  
      }
    } else {
      Firebase_Store("/Streetlight/LDR", "Day");
      ledcWrite(0, 0); 
      lightHoldActive = false;
    }

    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// Task 4: Accident Detection
void AccidentMonitorTask(void *pvParameters) {
  for (;;) {
    xSemaphoreTake(countMutex, portMAX_DELAY);
    int count = vehicleCount;
    unsigned long timeSinceDetect = millis() - vehicleDetectedTime;
    xSemaphoreGive(countMutex);

    if (count > 0) {
      if (timeSinceDetect >= 2 * 60 * 1000 && timeSinceDetect < 5 * 60 * 1000) {
        Firebase_Store("/Streetlight/Accident", "Warning.");
      }
      else if (timeSinceDetect >= 5 * 60 * 1000) {
        Firebase_Store("/Streetlight/Accident", "True");
      }
    } else {
      Firebase_Store("/Streetlight/Accident", "False");
    }

    vTaskDelay(pdMS_TO_TICKS(10000)); 
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(ldrPin, INPUT);

  connectWiFi();

  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  countMutex = xSemaphoreCreateMutex();


  xTaskCreatePinnedToCore(SensorEntryTask, "EntrySensor", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(SensorExitTask, "ExitSensor", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(LedTask, "LEDControl", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(AccidentMonitorTask, "AccidentMonitor", 4096, NULL, 1, NULL, 1);
}

void loop() {
  // RTOS so loop empty
  // Smart Street light + Traffic detection by Team Arduino
}
