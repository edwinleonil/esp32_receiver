/*
 *  ESP32 Receiver using ESP-NOW
 *  Prints out the received upVal and downVal in the Serial Monitor
 */

#include <esp_now.h>
#include <WiFi.h>

// Structure matching the transmitter
typedef struct struct_message {
  int upVal;
  int downVal;
} struct_message;

// Create a struct_message to hold the incoming data
struct_message incomingData;

// Callback function that will be executed when data is received
void onDataRecv(const uint8_t * mac, const uint8_t *incomingDataBytes, int len) {
  // Copy incoming bytes into our structure
  memcpy(&incomingData, incomingDataBytes, sizeof(incomingData));

  Serial.print("Data received from: ");
  Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X\n", 
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  Serial.print("upVal: ");
  Serial.print(incomingData.upVal);
  Serial.print("  |  downVal: ");
  Serial.println(incomingData.downVal);
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register the receive callback function
  esp_now_register_recv_cb(onDataRecv);

  Serial.println("Receiver ready.");
}

void loop() {
  // Do nothing in the main loop - data is processed in onDataRecv callback
  delay(1000);
}
