#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <MPU6050.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <LoRa.h>

bool parachuteDeployed = false;
unsigned long startTime = 0;
const unsigned long deployTime = 13000;

Adafruit_BMP085 bmp;
MPU6050 mpu;
TinyGPSPlus gps;
SoftwareSerial gpsSerial(2, 3);
SoftwareSerial unoSerial(4, 5);

#define LORA_SS 10
#define LORA_RST 9
#define LORA_DIO0 2
#define PYRO_PIN 7
#define BUZZER_PIN 8

#define LORA_FREQUENCY 433.2E6
#define LORA_SYNC_WORD 0x34
#define LORA_SPREADING_FACTOR 10
#define LORA_SIGNAL_BANDWIDTH 125E3

float vx = 0, vy = 0, vz = 0, yaw = 0;
float ax_offset = 0, ay_offset = 0, az_offset = 0;

void calibrateMPU() {
  Serial.println("Calibrating MPU... Do not move sensor.");
  long ax_sum = 0, ay_sum = 0, az_sum = 0;
  int16_t ax, ay, az, gx, gy, gz;
  const int num_readings = 1000;

  for (int i = 0; i < num_readings; i++) {
    mpu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, nullptr, nullptr, nullptr);
    float ax_mps2 = (ax / 16384.0) * 9.80665;
    float ay_mps2 = (ay / 16384.0) * 9.80665;
    float az_mps2 = (az / 16384.0) * 9.80665;

    float rocket_ax = ax_mps2;
    float rocket_ay = az_mps2;
    float rocket_az = ay_mps2;

    ax_sum += rocket_ax;
    ay_sum += rocket_ay;
    az_sum += rocket_az;
    delay(1);
  }

  ax_offset = (float)ax_sum / num_readings;
  ay_offset = (float)ay_sum / num_readings;
  az_offset = (float)az_sum / num_readings;
  Serial.println("Calibration complete.");
}

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600);
  unoSerial.begin(38400);

  pinMode(LORA_SS, OUTPUT);
  digitalWrite(LORA_SS, HIGH);
  pinMode(PYRO_PIN, OUTPUT);
  digitalWrite(PYRO_PIN, LOW);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  Wire.begin();

  if (!bmp.begin()) { while (1); }
  mpu.initialize();
  if (!mpu.testConnection()) { while (1); }
  calibrateMPU();

  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQUENCY)) { while (1); }
  LoRa.setSyncWord(LORA_SYNC_WORD);
  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
  LoRa.setSignalBandwidth(LORA_SIGNAL_BANDWIDTH);

  startTime = millis();
}

void loop() {
  while (gpsSerial.available())
    gps.encode(gpsSerial.read());

  static unsigned long lastTime = 0;
  static float lastAltitude = 0;
  unsigned long currentTime = millis();

  if (lastTime == 0) {
    lastTime = currentTime;
    lastAltitude = bmp.readAltitude();
    return;
  }

  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  float altitude = bmp.readAltitude();
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, nullptr, nullptr, nullptr);

  float ax_mps2 = (ax / 16384.0) * 9.80665;
  float ay_mps2 = (ay / 16384.0) * 9.80665;
  float az_mps2 = (az / 16384.0) * 9.80665;
  float ay_temp = ay_mps2; ay_mps2 = az_mps2; az_mps2 = ay_temp;

  vx += (ax_mps2 - ax_offset) * dt;
  vy += (ay_mps2 - ay_offset) * dt;
  vz = (altitude - lastAltitude) / dt;
  lastAltitude = altitude;

  float pitch = atan2(ax_mps2, sqrt(ay_mps2 * ay_mps2 + az_mps2 * az_mps2)) * 180 / PI;
  float roll = atan2(ay_mps2, az_mps2) * 180 / PI;
  yaw += (gz / 131.0) * dt;

  String msg = "";
  msg += String(altitude, 2) + "," + String(pitch, 2) + "," + String(roll, 2) + "," + String(yaw, 2) + ",";
  msg += String(ax_mps2 - ax_offset, 2) + "," + String(ay_mps2 - ay_offset, 2) + "," + String(az_mps2 - az_offset, 2) + ",";
  msg += String(vx, 2) + "," + String(vy, 2) + "," + String(vz, 2) + ",";
  msg += String(bmp.readTemperature(), 2) + "," + String(bmp.readPressure()) + ",";
  if (gps.location.isValid()) {
    msg += String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6);
  } else {
    msg += "0.0,0.0";
  }

  LoRa.beginPacket();
  LoRa.print(msg);
  LoRa.endPacket();

  unoSerial.println(msg);
  Serial.println(msg);
  delay(100);
}
