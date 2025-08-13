#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <MPU6050.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <LoRa.h>

// --- Sensor Objects ---
Adafruit_BMP085 bmp;
MPU6050 mpu;
TinyGPSPlus gps;
SoftwareSerial gpsSerial(2, 3); // RX, TX

// --- Pins ---
#define LORA_SS 10
#define LORA_RST 9
#define LORA_DIO0 2
#define PYRO_PIN 7
#define BUZZER_PIN 8

// --- Altitude ---
float baseAltitude = 0;
bool baseCaptured = false;
unsigned long deployDelay = 2000;
unsigned long lastDeployCheck = 0;

// --- Setup ---
void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);

  pinMode(PYRO_PIN, OUTPUT);
  digitalWrite(PYRO_PIN, LOW);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  Wire.begin();

  // --- BMP180 Check ---
  if (!bmp.begin()) {
    buzzError(1);
    while (1); // halt
  }

  // --- MPU6050 Check ---
  mpu.initialize();
  if (!mpu.testConnection()) {
    buzzError(2);
    while (1); // halt
  }

  // --- GPS Check ---
  unsigned long gpsTimeout = millis();
  while (!gps.location.isUpdated() && millis() - gpsTimeout < 5000) {
    while (gpsSerial.available()) {
      gps.encode(gpsSerial.read());
    }
  }
  if (!gps.location.isValid()) {
    buzzError(3);
    while (1); // halt
  }

  // --- LoRa Check ---
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(433E6)) {
    buzzError(4);
    while (1); // halt
  }

  Serial.println("Setup complete.");
}

// --- Loop ---
void loop() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  float altitude = bmp.readAltitude();
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  if (!baseCaptured && gps.location.isValid()) {
    baseAltitude = altitude;
    baseCaptured = true;
  }

  String msg = "";
  msg += "ALT:" + String(altitude, 2);
  msg += ",AX:" + String(ax);
  msg += ",AY:" + String(ay);
  msg += ",AZ:" + String(az);
  if (gps.location.isValid()) {
    msg += ",LAT:" + String(gps.location.lat(), 6);
    msg += ",LON:" + String(gps.location.lng(), 6);
  } else {
    msg += ",LAT:N/A,LON:N/A";
  }

  LoRa.beginPacket();
  LoRa.print(msg);
  LoRa.endPacket();

  Serial.println(msg);

  if (millis() - lastDeployCheck > deployDelay) {
    lastDeployCheck = millis();

    float currentAlt = bmp.readAltitude();
    static float lastAlt = currentAlt;

    if ((lastAlt - currentAlt) > 10) {
      deployParachute();
    }

    lastAlt = currentAlt;
  }

  delay(500);
}

// --- Deploy Pyro ---
void deployParachute() {
  digitalWrite(PYRO_PIN, HIGH);
  delay(500);
  digitalWrite(PYRO_PIN, LOW);
  Serial.println("Parachute Deployed!");
}

// --- Buzzer Error Pattern ---
void buzzError(int code) {
  for (int i = 0; i < code; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(200);
    digitalWrite(BUZZER_PIN, LOW);
    delay(200);
  }
  delay(1000); // pause after error pattern
}
