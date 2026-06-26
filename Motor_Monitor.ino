

#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <MPU6050.h>
#include <ArduinoJson.h>

// ======================================================
//                  WIFI CONFIG
// ======================================================

const char* WIFI_SSID = "Your_WiFi_Name";
const char* WIFI_PASSWORD = "WIFI_PASSWORD";

// ======================================================
//                MQTT CONFIGURATION
// ======================================================

// Use your Mosquitto broker IP
const char* MQTT_BROKER = "broker.hivemq.com";

// Common MQTT Port
const int MQTT_PORT = 1883;

// MQTT Topic
const char* MQTT_TOPIC = "pdm/motor/features";

// ======================================================
//                  PIN DEFINITIONS
// ======================================================

#define PIN_CURRENT      34
#define PIN_TEMP         35

#define PIN_LED_OK       25
#define PIN_LED_FAULT    26
#define PIN_BUZZER       27

// ======================================================
//                 INDUSTRIAL LIMITS
// ======================================================

#define VIB_WARNING      0.5
#define VIB_DANGER       1.0

#define TEMP_WARNING     75
#define TEMP_DANGER      90

#define CURR_WARNING     2.5
#define CURR_DANGER      4.0

// ======================================================
//                  SAMPLE SETTINGS
// ======================================================

#define SAMPLE_SIZE      128

float samples[SAMPLE_SIZE];

// ======================================================
//                    OBJECTS
// ======================================================

WiFiClient espClient;

PubSubClient client(espClient);

MPU6050 mpu;

// ======================================================
//                WIFI CONNECTION
// ======================================================

void connectWiFi() {

  Serial.println("Connecting to WiFi");

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {

    delay(500);

    Serial.print(".");
  }

  Serial.println("");

  Serial.println("WiFi Connected");
}

// ======================================================
//               MQTT CONNECTION
// ======================================================

void connectMQTT() {

  while (!client.connected()) {

    Serial.println("Connecting to MQTT...");

    String clientID = "ESP32Client-";
    clientID += String(random(0xffff), HEX);

    if (client.connect(clientID.c_str())) {

      Serial.println("MQTT Connected");
    }

    else {

      Serial.print("MQTT Failed, rc=");
      Serial.println(client.state());

      delay(2000);
    }
  }
}

// ======================================================
//                  READ CURRENT
// ======================================================

float readCurrent() {

  int raw = analogRead(PIN_CURRENT);

  return (raw / 4095.0) * 5.0;
}

// ======================================================
//               READ TEMPERATURE
// ======================================================

float readTemperature() {

  int raw = analogRead(PIN_TEMP);

  return 20.0 + (raw / 4095.0) * 80.0;
}

// ======================================================
//                 COMPUTE RMS
// ======================================================

float computeRMS(float data[], int size) {

  float sum = 0;

  for (int i = 0; i < size; i++) {

    sum += data[i] * data[i];
  }

  return sqrt(sum / size);
}

// ======================================================
//              COMPUTE MEAN
// ======================================================

float computeMean(float data[], int size) {

  float sum = 0;

  for (int i = 0; i < size; i++) {

    sum += data[i];
  }

  return sum / size;
}

// ======================================================
//             COMPUTE STANDARD DEV
// ======================================================

float computeStdDev(float data[],
                    int size,
                    float mean) {

  float variance = 0;

  for (int i = 0; i < size; i++) {

    variance += pow(data[i] - mean, 2);
  }

  variance /= size;

  return sqrt(variance);
}

// ======================================================
//               COMPUTE KURTOSIS
// ======================================================

float computeKurtosis(float data[],
                      int size) {

  float mean = computeMean(data, size);

  float stdDev = computeStdDev(data,
                               size,
                               mean);

  if (stdDev == 0)
    return 0;

  float sum = 0;

  for (int i = 0; i < size; i++) {

    sum += pow(data[i] - mean, 4);
  }

  return (sum / size) / pow(stdDev, 4);
}

// ======================================================
//            DETERMINE MOTOR STATUS
// ======================================================

String determineStatus(float vib_rms,
                       float curr,
                       float temp) {

  if (
      vib_rms > VIB_DANGER ||
      curr > CURR_DANGER ||
      temp > TEMP_DANGER
     ) {

    return "DANGER";
  }

  if (
      vib_rms > VIB_WARNING ||
      curr > CURR_WARNING ||
      temp > TEMP_WARNING
     ) {

    return "WARNING";
  }

  return "NORMAL";
}

// ======================================================
//                  ALERT SYSTEM
// ======================================================

void triggerAlert(String status) {

  if (status == "NORMAL") {

    digitalWrite(PIN_LED_OK, HIGH);

    digitalWrite(PIN_LED_FAULT, LOW);

    digitalWrite(PIN_BUZZER, LOW);
  }

  else if (status == "WARNING") {

    digitalWrite(PIN_LED_OK, LOW);

    digitalWrite(PIN_LED_FAULT, HIGH);

    digitalWrite(PIN_BUZZER, HIGH);

    delay(150);

    digitalWrite(PIN_BUZZER, LOW);
  }

  else {

    digitalWrite(PIN_LED_OK, LOW);

    digitalWrite(PIN_LED_FAULT, HIGH);

    for (int i = 0; i < 5; i++) {

      digitalWrite(PIN_BUZZER, HIGH);
      delay(80);

      digitalWrite(PIN_BUZZER, LOW);
      delay(80);
    }
  }
}

// ======================================================
//                       SETUP
// ======================================================

void setup() {

  Serial.begin(115200);

  Wire.begin(21, 22);

  mpu.initialize();

  pinMode(PIN_LED_OK, OUTPUT);

  pinMode(PIN_LED_FAULT, OUTPUT);

  pinMode(PIN_BUZZER, OUTPUT);

  connectWiFi();

  client.setServer(MQTT_BROKER,
                   MQTT_PORT);

  connectMQTT();

  Serial.println("System Ready");
}

// ======================================================
//                        LOOP
// ======================================================

void loop() {

  if (!client.connected()) {

    connectMQTT();
  }

  client.loop();

  // ===================================================
  //        COLLECT MPU6050 VIBRATION SAMPLES
  // ===================================================

  for (int i = 0; i < SAMPLE_SIZE; i++) {

    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    mpu.getMotion6(&ax, &ay, &az,
                   &gx, &gy, &gz);

    float ax_g = ax / 16384.0;
    float ay_g = ay / 16384.0;
    float az_g = az / 16384.0;

    // Magnitude
    samples[i] = sqrt(
                    ax_g * ax_g +
                    ay_g * ay_g +
                    az_g * az_g
                  );

    delay(2);
  }

  // ===================================================
  //          FEATURE EXTRACTION
  // ===================================================

  float vib_rms = computeRMS(samples,
                             SAMPLE_SIZE);

  float kurtosis = computeKurtosis(samples,
                                   SAMPLE_SIZE);

  float curr_rms = readCurrent();

  float temp = readTemperature();

  String status = determineStatus(
                    vib_rms,
                    curr_rms,
                    temp);

  // ===================================================
  //                 ALERT SYSTEM
  // ===================================================

  triggerAlert(status);

  // ===================================================
  //                  JSON PAYLOAD
  // ===================================================

  StaticJsonDocument<256> doc;

  doc["vib_rms"] = vib_rms;

  doc["kurtosis"] = kurtosis;

  doc["curr_rms"] = curr_rms;

  doc["temp"] = temp;

  doc["motor_status"] = status;

  char payload[256];

  serializeJson(doc, payload);

  // ===================================================
  //                 MQTT PUBLISH
  // ===================================================

  client.publish(MQTT_TOPIC, payload);

  // ===================================================
  //               SERIAL DEBUG OUTPUT
  // ===================================================

  Serial.println("================================");

  Serial.println(payload);

  Serial.println("================================");

  delay(1000);
}
