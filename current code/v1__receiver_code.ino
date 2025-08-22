#include <SPI.h>
#include <LoRa.h>

// Define LoRa settings
#define LORA_FREQUENCY 433E6
#define LORA_SYNC_WORD 0x12
#define LORA_SPREADING_FACTOR 12
#define LORA_SIGNAL_BANDWIDTH 250E3

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
}

void loop() {
  // Try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String incoming = "";

    while (LoRa.available()) {
      incoming += (char)LoRa.read();
    }

    Serial.println(incoming);
  }
}
