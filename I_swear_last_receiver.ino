#include <SPI.h>
#include <LoRa.h>

// Define LoRa settings
#define LORA_FREQUENCY 433E6
#define LORA_SYNC_WORD 0x12
#define LORA_SPREADING_FACTOR 12
#define LORA_SIGNAL_BANDWIDTH 250E3

// Define data structure to match the new sensor data format
struct SensorData {
  int16_t Vx, Vy, Vz;
  int16_t Ax, Ay, Az;
  int16_t Ox, Oy, Oz;
  int16_t altitude;
  int16_t temperature;
  int32_t pressure;
  // unsigned long timestamp;
  // int16_t timeMilliSeconds;
} sensorData;

// Array to store received data
uint8_t receivedData[28];  // Adjusted size to match the structure

void setup() {
  Serial.begin(9600);
  while (!Serial); // Wait for serial port to connect (required for native USB port)

  Serial.println("Initializing LoRa...");
  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("Starting LoRa failed!");
    while (1); // Infinite loop to halt execution if initialization fails
  }

  // Set LoRa parameters
  LoRa.setSyncWord(LORA_SYNC_WORD);
  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
  LoRa.setSignalBandwidth(LORA_SIGNAL_BANDWIDTH);

  // Initialize sensor data
  sensorData = {0};
}

void loop() {
  // Try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // Read packet data into the array
    for (int i = 0; i < packetSize && i < sizeof(receivedData); i++) {
      receivedData[i] = LoRa.read();
    }

    // Extract sensor data from the received byte array
    
    // Velocities
    sensorData.Vx = (receivedData[0] << 8) | receivedData[1];
    sensorData.Vy = (receivedData[2] << 8) | receivedData[3];
    sensorData.Vz = (receivedData[4] << 8) | receivedData[5];

    // Acceleration
    sensorData.Ax = (receivedData[6] << 8) | receivedData[7];
    sensorData.Ay = (receivedData[8] << 8) | receivedData[9];
    sensorData.Az = (receivedData[10] << 8) | receivedData[11];

    // Orientation
    sensorData.Ox = (receivedData[12] << 8) | receivedData[13];
    sensorData.Oy = (receivedData[14] << 8) | receivedData[15];
    sensorData.Oz = (receivedData[16] << 8) | receivedData[17];

    // Altitude, Temperature, Pressure
    sensorData.altitude = (receivedData[18] << 8) | receivedData[19];
    sensorData.temperature = (receivedData[20] << 8) | receivedData[21];
    sensorData.pressure = (receivedData[22] << 8) | receivedData[23];

    // Time values
    // sensorData.timeMilliSeconds = millis();

    // Example: Print sensor data
    Serial.print(sensorData.Vx);
    Serial.print(",");
    Serial.print(sensorData.Vy);
    Serial.print(",");
    Serial.print(sensorData.Vz);

    Serial.print(",");
    Serial.print(sensorData.Ax);
    Serial.print(",");
    Serial.print(sensorData.Ay);
    Serial.print(",");
    Serial.print(sensorData.Az);

    Serial.print(",");
    Serial.print(sensorData.Ox);
    Serial.print(",");
    Serial.print(sensorData.Oy);
    Serial.print(",");
    Serial.print(sensorData.Oz);

    Serial.print(",");
    Serial.print(sensorData.altitude);
    
    Serial.print(",");
    Serial.print(sensorData.temperature);
    
    Serial.print(",");
    Serial.print(sensorData.pressure);

    Serial.println(",");
    // Serial.println(sensorData.timeMilliSeconds);
  }
}
