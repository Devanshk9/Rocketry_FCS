#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <MPU6050.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <LoRa.h>

/* --- Deployment Timer ---
bool parachuteDeployed = false;
unsigned long startTime = 0;
const unsigned long deployTime = 13000; // 13 seconds */
bool transmissionEnabled = false;
float maxAltitude = 0.0;
float baseAltitude = 0.0;
int consecutiveDecreaseCounter = 0;
const int DECREASE_THRESHOLD = 4; // Fire after 4 consecutive drops
const float ARMING_ALTITUDE = 50.0; // 50 meters above ground
bool isArmed = false;
bool parachuteDeployed = false;

// --- Sensor Objects ---
Adafruit_BMP085 bmp;
MPU6050 mpu;
TinyGPSPlus gps;
SoftwareSerial gpsSerial(7,8); // tx rx
// SoftwareSerial unoSerial(4,5);

// --- Global variables ---
float vx = 0, vy = 0, vz = 0;
float yaw = 0;
float ax_offset = 0, ay_offset = 0, az_offset = 0;

// --- Pins ---
#define LORA_SS 10
#define LORA_RST 9
#define LORA_DIO0 2
#define PYRO_PIN A1
#define BUZZER_PIN A2

// --- LoRa Parameters (MUST MATCH RECEIVER) ---
#define LORA_FREQUENCY 433.2E6
#define LORA_SYNC_WORD 0x34
#define LORA_SPREADING_FACTOR 10
#define LORA_SIGNAL_BANDWIDTH 125E3

// --- MPU Calibration Function (Y-axis vertical) ---
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
  Serial.print("Offsets -> X: "); Serial.print(ax_offset, 2);
  Serial.print(" | Y: "); Serial.print(ay_offset, 2);
  Serial.print(" | Z: "); Serial.println(az_offset, 2);
}

// --- Setup ---
void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600);
  //unoSerial.begin(38400); // <-- 4. CORRECTED: Match TX/RX code baud rate

  pinMode(LORA_SS, OUTPUT);
  digitalWrite(LORA_SS, HIGH);
  pinMode(PYRO_PIN, OUTPUT);
  digitalWrite(PYRO_PIN, LOW);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  Wire.begin();

  Serial.println("Initialization in progress..");

  // --- Configure LoRa ---
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("LoRa init failed!");
    while (1);
  } else {
    Serial.println("LoRa successfully ON");
  }

  LoRa.setSyncWord(LORA_SYNC_WORD);
  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
  LoRa.setSignalBandwidth(LORA_SIGNAL_BANDWIDTH);

  // --- BMP180 Check ---
  
  if (!bmp.begin()) {
    Serial.println("BMP issue");
      LoRa.beginPacket();
      LoRa.print("BMP issue");
      LoRa.endPacket();
    buzzError(1);
    while (1);
  } else {
    Serial.println("BMP successfully ON");
  }

  // --- MPU6050 Check ---
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU issue");
          LoRa.beginPacket();
      LoRa.print("MPU issue");
      LoRa.endPacket();
    buzzError(2);
    while (1);
  } else {
    Serial.println("MPU successfully ON");
  }
    // --- Calibrate MPU ---
  calibrateMPU();


  
  // --- GPS Check (from working old code) ---
  bool gpsFixAcquired = false;

    while (!gpsFixAcquired) {
      if (gpsSerial.available() > 10) {
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


  /*Serial.println("Setup complete. Starting 13s deploy timer.");
  startTime = millis(); // Start the timer */

  delay(250);
  baseAltitude = bmp.readAltitude();
  maxAltitude = baseAltitude; // The max altitude so far is the ground
  Serial.print("Base altitude set: "); Serial.println(baseAltitude);
  
  Serial.println("Setup complete. Waiting for launch.");

}

// --- Loop ---
void loop() {
  // Feed the GPS
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String receivedString = "";
    while (LoRa.available())
      receivedString += (char)LoRa.read();
    receivedString.trim();

    if (receivedString.equalsIgnoreCase("START")) {
      transmissionEnabled = true;
      Serial.println("Transmission ENABLED");
    } else if (receivedString.equalsIgnoreCase("STOP")) {
      transmissionEnabled = false;
      Serial.println("Transmission DISABLED");
    }
  }

  if(transmissionEnabled){
    while (gpsSerial.available() > 0) {
      gps.encode(gpsSerial.read());
    }

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

    // --- Sensor Reading & Calculations ---
    float altitude = bmp.readAltitude();
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, nullptr, nullptr, nullptr);

    float ax_mps2 = (ax / 16384.0) * 9.80665;
    float ay_mps2 = (ay / 16384.0) * 9.80665;
    float az_mps2 = (az / 16384.0) * 9.80665;

    float ay_temp = ay_mps2;
    ay_mps2 = az_mps2;
    az_mps2 = ay_temp;

    vx += (ax_mps2 - ax_offset) * dt;
    vy += (ay_mps2 - ay_offset) * dt;
    vz = (altitude - lastAltitude) / dt;
    lastAltitude = altitude;

    float pitch = atan2(ax_mps2, sqrt(ay_mps2 * ay_mps2 + az_mps2 * az_mps2)) * 180 / PI;
    float roll  = atan2(ay_mps2, az_mps2) * 180 / PI;
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

    // --- 1. Send String over LoRa ---
    LoRa.beginPacket();
    LoRa.print(msg);
    LoRa.endPacket();

    // --- 2. Send String to Uno Logger ---
    //unoSerial.println(msg);
    
    // --- 3. Print String to Serial Monitor ---
    Serial.println(msg);
  // --- NEW: Apogee Deployment Logic ---
    if (!parachuteDeployed) { // Only run if not already deployed
      
      float currentAlt = bmp.readAltitude();

      // Step 1: Arm the system if we are 50m above ground
      if (!isArmed && currentAlt > (baseAltitude + ARMING_ALTITUDE)) {
        isArmed = true;
        Serial.println("DEPLOYMENT SYSTEM ARMED");
      }

      // Step 2: Run apogee check only if armed
      if (isArmed) {
        if (currentAlt > maxAltitude) {
          // We are still climbing
          maxAltitude = currentAlt;
          consecutiveDecreaseCounter = 0; // Reset counter
        } else {
          // We are lower than the peak
          consecutiveDecreaseCounter++;
          Serial.print("Apogee counter: "); Serial.println(consecutiveDecreaseCounter);
        }

        // Step 3: Check if we have hit the trigger threshold
        if (consecutiveDecreaseCounter >= DECREASE_THRESHOLD) {
          Serial.println("APOGEE DETECTED. DEPLOYING PARACHUTE!");
          deployParachute();
          parachuteDeployed = true; // Fire only once
        }
      }
    }
  }
  
  /* --- Deployment Logic (13s Timer) ---
  if (!parachuteDeployed && (millis() - startTime > deployTime)) {
    parachuteDeployed = true;
    Serial.println("TIMER ELAPSED. DEPLOYING PARACHUTE!");
    deployParachute();
  } */

  delay(1000);
} 

// --- Deploy Pyro ---
void deployParachute() {
  digitalWrite(PYRO_PIN, HIGH);
  delay(4000); // 4 second burn
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
  delay(1000);
}
