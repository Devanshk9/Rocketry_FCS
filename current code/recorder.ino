#include <SPI.h>
#include <SD.h>

// SD card is on the hardware SPI pins (10, 11, 12, 13)
const int chipSelect = 10;

// Timeout variables ⏱️
unsigned long lastReceiveTime;
const unsigned long TIMEOUT_PERIOD = 5000; // 5 seconds
bool isLogging = true; // State variable to track if we are actively logging

void setup() {
  // Use the hardware serial to receive data from the Nano
  Serial.begin(38400); 

  // --- SD Card Initialization ---
  if (!SD.begin(chipSelect)) {
    // This will only be seen if connected to a PC
    Serial.println("SD Card initialization failed!");
    while (1); // Halt
  }

  // Create a new file for this session
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.println("--- New Flight Log ---");
    dataFile.close();
    Serial.println("SD card initialized. Awaiting data from Nano...");
  } else {
    Serial.println("Error creating log file!");
    while(1); // Halt
  }
  
  // Start the timer
  lastReceiveTime = millis();
}

void loop() {
  // Only run the loop if we are in the logging state
  if (isLogging) {
    // Check if data is available from the Nano
    if (Serial.available() > 0) {
      // Data received, so reset the timer
      lastReceiveTime = millis();
      
      String receivedData = Serial.readStringUntil('\n');

      File dataFile = SD.open("datalog.txt", FILE_WRITE);
      if (dataFile) {
        dataFile.println(receivedData);
        dataFile.close();
      }
    }

    // Check if the timeout period has been exceeded
    if (millis() - lastReceiveTime > TIMEOUT_PERIOD) {
      isLogging = false; // Stop logging

      // Finalize the log file
      File dataFile = SD.open("datalog.txt", FILE_WRITE);
      if (dataFile) {
        dataFile.println("--- Log complete: Nano disconnected ---");
        dataFile.close();
      }
      
      // Print the completion message to the serial monitor
      Serial.println("\nNano signal lost. Writing file complete.");

      while(1); // Halt the program
    }
  }
}
