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

//Global variables which need time for calculation
float vx, vy, vz;
float yaw;

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
  // while (!gps.location.isUpdated() && millis() - gpsTimeout < 12000) {
  while (!gps.location.isUpdated()) {
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

  static unsigned long lastTime = millis();
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  float altitude = bmp.readAltitude();
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, nullptr, nullptr, nullptr);

  float ax_mps2 = (ax / 16384.0) * 9.80665;
  float ay_mps2 = (ay / 16384.0) * 9.80665;
  float az_mps2 = (az / 16384.0) * 9.80665;

  vx += ax_mps2 * dt;
  vy += ay_mps2 * dt;
  vz += az_mps2 * dt;

  float pitch = atan2(ax_mps2, sqrt(ay_mps2 * ay_mps2 + az_mps2 * az_mps2)) * 180 / PI;
  float roll  = atan2(ay_mps2, az_mps2) * 180 / PI;
  yaw += (gz / 131.0) * dt;

  if (!baseCaptured && gps.location.isValid()) {
    baseAltitude = altitude;
    baseCaptured = true;
  }

  String msg = "";
  msg += String(vx, 6) + "," + String(vy, 6) + "," + String(vz, 6) + ",";
  msg += String(ax_mps2, 6) + "," + String(ay_mps2, 6) + "," + String(az_mps2, 6) + ",";
  msg += String(altitude, 2) + "," + String(bmp.readTemperature(), 2) + "," + String(bmp.readPressure()) + ",";
  msg += String(pitch, 6) + "," + String(yaw, 6) + "," + String(roll, 6) + ",";
  msg += String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6);


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
  // Serial.println("Parachute Deployed!");
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
