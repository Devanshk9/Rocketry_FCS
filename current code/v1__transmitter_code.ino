#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <MPU6050.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <LoRa.h>

// --- Sensor Objects ---
Adafruit_BMP085 bmp;
MPU6050 mpu;
TinyGPSPlus gps;
SoftwareSerial gpsSerial(8, 7); // RX, TX

//Global variables which need time for calculation
float vx, vy, vz;
float yaw;

// --- Pins ---
#define LORA_SS 10
#define LORA_RST 9
#define LORA_DIO0 2
#define PYRO_PIN A1
#define BUZZER_PIN A2

// --- Altitude ---
float baseAltitude = 0;
bool baseCaptured = false;
unsigned long deployDelay = 2000;
unsigned long lastDeployCheck = 0;

// --- Setup ---
void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);

  // pinMode(PYRO_PIN, OUTPUT);
  // digitalWrite(PYRO_PIN, LOW);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  Wire.begin();

  // --- LoRa Check ---
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(433E6)) {
    buzzError(4);
    Serial.println("LoRa initialization failed");
    while (1); // halt
  }

  // LoRa.beginPacket();
  // LoRa.print("LoRa intialized");
  // LoRa.endPacket();

  Serial.println("LoRa initialized");



  // --- BMP180 Check ---
  if (!bmp.begin()) {
    buzzError(1);
    while (1); // halt
  }
  // LoRa.beginPacket();
  // LoRa.print("BMP intialized");
  // LoRa.endPacket();

  Serial.println("BMP initialized");


  // --- MPU6050 Check ---
  mpu.initialize();
  while(!mpu.testConnection()) {
    Serial.println("MPU Not getting initialized");
    buzzError(2);
     // halt
  }

  // LoRa.beginPacket();
  // LoRa.print("MPU intialized");
  // LoRa.endPacket();

  Serial.println("MPU initialized");


  // --- GPS Check ---
  // unsigned long gpsTimeout = millis();
  // while (!gps.location.isUpdated() && millis() - gpsTimeout < 5000) {
  // // while (!gps.location.isUpdated()) {
  //   Serial.println("Oooooooooo");
  //   while (gpsSerial.available()) {
  //     Serial.println("Aaaaaaaaaaaaa");
  //     gps.encode(gpsSerial.read());
  //   }
  // }
 
  // while(!gps.location.isValid()){
  //   Serial.println("Waiting for Signal Mate");
  // }
  bool gpsFixAcquired = false;

  while (!gpsFixAcquired) {
    if (gpsSerial.available() > 0) {
      if(gps.encode(gpsSerial.read())){
      if (gps.location.isValid()) {
        gpsFixAcquired = true;
        Serial.println("GPS fix acquired!");
        Serial.print("Lat: "); Serial.println(gps.location.lat(), 6);
        Serial.print("Lng: "); Serial.println(gps.location.lng(), 6);
      } 
      else {
        Serial.println("Waiting for GPS fix...");
        delay(500); // Optional: reduce serial spam
      }}
    }
    else{
      Serial.println("Not available");
      delay(500);
    }

    
  }


  // LoRa.beginPacket();
  // LoRa.print("GPS intialized");
  // LoRa.endPacket();

  Serial.println("GPS initialized");

  

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


  // LoRa.beginPacket();
  // LoRa.print(msg);
  // LoRa.endPacket();

  Serial.println(msg);

  // if (millis() - lastDeployCheck > deployDelay) {
  //   lastDeployCheck = millis();

  //   float currentAlt = bmp.readAltitude();
  //   static float lastAlt = currentAlt;

  //   if ((lastAlt - currentAlt) > 10) {
  //     deployParachute();
  //   }

  //   lastAlt = currentAlt;
  // }

  delay(500);
}

// --- Deploy Pyro ---
// void deployParachute() {
//   digitalWrite(PYRO_PIN, HIGH);
//   delay(500);
//   digitalWrite(PYRO_PIN, LOW);
//   // Serial.println("Parachute Deployed!");
// }

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
