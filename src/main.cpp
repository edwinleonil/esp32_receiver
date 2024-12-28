/*
 *  RECEIVER (ESP32 #2)
 *  - speedVal in range: -255..255
 *    * >0 => forward
 *    * <0 => backward
 *    *  0 => stop
 *
 *  Uses TWO pins for PWM on the motor driver (IN1 and IN2), no separate ENABLE pin.
 */

#include <esp_now.h>
#include <WiFi.h>

typedef struct struct_message {
  int16_t speedVal;  // -255..255
} struct_message;

struct_message incomingData;

// Assign two GPIO pins for the motor driver inputs
const int in1Pin = 19;  // PWM channel 0
const int in2Pin = 18;  // PWM channel 1

// LEDC (PWM) setup
// We'll use two different channels, same frequency, same resolution:
const int pwmCh1   = 0;      // for in1Pin
const int pwmCh2   = 1;      // for in2Pin
const int pwmFreq  = 5000;   // 5 kHz
const int pwmRes   = 8;      // 8-bit => 0..255 duty cycle

// Callback: triggered when data is received via ESP-NOW
void onDataRecv(const uint8_t* mac, const uint8_t* incomingDataBytes, int len) {
  memcpy(&incomingData, incomingDataBytes, sizeof(incomingData));

  int16_t speed = incomingData.speedVal;
  Serial.print("Received speedVal: ");
  Serial.println(speed);

  if (speed > 0) {
    // FORWARD: IN1 = PWM, IN2 = 0
    ledcWrite(pwmCh1, speed);         // duty cycle = speed (1..255)
    ledcWrite(pwmCh2, 0);            // no output on IN2
    Serial.println("Motor FORWARD");
  }
  else if (speed < 0) {
    // BACKWARD: IN2 = PWM, IN1 = 0
    ledcWrite(pwmCh1, 0);
    ledcWrite(pwmCh2, abs(speed));    // duty cycle = abs(speed) (1..255)
    Serial.println("Motor BACKWARD");
  }
  else {
    // STOP: IN1 = 0, IN2 = 0
    ledcWrite(pwmCh1, 0);
    ledcWrite(pwmCh2, 0);
    Serial.println("Motor STOPPED");
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  // Configure PWM channels
  ledcSetup(pwmCh1, pwmFreq, pwmRes);
  ledcSetup(pwmCh2, pwmFreq, pwmRes);

  // Attach channels to the pins
  ledcAttachPin(in1Pin, pwmCh1);
  ledcAttachPin(in2Pin, pwmCh2);

  // Start both at 0 (stopped)
  ledcWrite(pwmCh1, 0);
  ledcWrite(pwmCh2, 0);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register the receive callback
  esp_now_register_recv_cb(onDataRecv);

  Serial.println("Receiver ready. Using two PWM pins (no enable).");
}

void loop() {
  // Everything is handled in onDataRecv.
  delay(1000);
}
