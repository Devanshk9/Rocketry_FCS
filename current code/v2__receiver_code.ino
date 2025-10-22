#include <SPI.h>
#include <LoRa.h>

#define LORA_SS 10
#define LORA_RST 9
#define LORA_DIO0 2
#define LORA_FREQUENCY 433.2E6
#define LORA_SYNC_WORD 0x34
#define LORA_SPREADING_FACTOR 10
#define LORA_SIGNAL_BANDWIDTH 125E3

void setup() {
  Serial.begin(115200);
  while (!Serial);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQUENCY)) { while (1); }
  LoRa.setSyncWord(LORA_SYNC_WORD);
  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
  LoRa.setSignalBandwidth(LORA_SIGNAL_BANDWIDTH);
  Serial.println("Receiver Ready");
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String receivedString = "";
    while (LoRa.available())
      receivedString += (char)LoRa.read();
    Serial.println(receivedString);
  }
}
