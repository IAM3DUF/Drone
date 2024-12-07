#include "HardwareSerial.h"
#include "sbus.h"
#include <esp_now.h>
#include <WiFi.h>

//*************************************Pin Assignments**************************************
// DO NOT USE PINS 6, 7, 8, 9, 10, OR 11
// Potentiometer assignments:
// Roll - Right Joystick X-axis (pin ROLL_PIN)
// Pitch - Right Joystick Y-axis (pin PITCH_PIN)
// Yaw - Left Joystick Y-axis (pin YAW_PIN)
// Throttle - Left Joystick X-axis (pin THROTTLE_PIN)
// Electromagnet Toggle - EM_PIN
// Arming Toggle - ARM_PIN

const int GREEN_LED_PIN = 13;
const int RED_LED_PIN = 12;

const int ARM_PIN = 36;
const int THROTTLE_PIN = 34; 
const int YAW_PIN = 39; 
const int EM_PIN = 14; 

const int ROLL_PIN = 35; 
const int PITCH_PIN = 32;

int throttleValue = 1500;
int electromagnetToggleCount = 0;

// Deadzone calibration offsets
int rollZero = 0;
int pitchZero = 0;
int yawZero = 0;

//*************************************ESP-NOW Transmitter Globals**************************************
uint8_t broadcastAddress[] = {0xd0, 0xef, 0x76, 0x58, 0xc8, 0x30};

typedef struct {
    int roll = 1500;    // Roll command (range: 1000-2000)
    int pitch = 1500;   // Pitch command (range: 1000-2000)
    int yaw = 1500;     // Yaw command (range: 1000-2000)
    int throttle = 1000; // Throttle command (range: 1000-2000)
    bool electromagnet = false; // Electromagnet status
    bool arm = false;           // Arming status
} dataStruct;

dataStruct controllerData;

esp_now_peer_info_t peerInfo;

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    // Serial.print("\r\nLast Packet Send Status:\t");
    // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");

    digitalWrite(GREEN_LED_PIN, status == ESP_NOW_SEND_SUCCESS);
    digitalWrite(RED_LED_PIN, status != ESP_NOW_SEND_SUCCESS);
}

//*************************************Setup Function**************************************
void setup() {
    Serial.begin(115200);

    pinMode(THROTTLE_PIN, INPUT);
    pinMode(YAW_PIN, INPUT);
    pinMode(ROLL_PIN, INPUT);
    pinMode(PITCH_PIN, INPUT);
    pinMode(ARM_PIN, INPUT);
    pinMode(EM_PIN, INPUT);

    pinMode(GREEN_LED_PIN, OUTPUT);
    pinMode(RED_LED_PIN, OUTPUT);

    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    esp_now_register_send_cb(onDataSent);
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }

    // Calibrate deadzone
    rollZero = analogRead(ROLL_PIN);
    pitchZero = analogRead(PITCH_PIN);
    yawZero = analogRead(YAW_PIN);

    Serial.println("Calibration complete");
    Serial.print("Roll Zero: "); Serial.println(rollZero);
    Serial.print("Pitch Zero: "); Serial.println(pitchZero);
    Serial.print("Yaw Zero: "); Serial.println(yawZero);
}

//*************************************Main Loop**************************************
void loop() {
    controllerData.arm = digitalRead(ARM_PIN);
    controllerData.roll = mapStickInput(ROLL_PIN, rollZero);
    controllerData.pitch = 3000 - mapStickInput(PITCH_PIN, pitchZero); // Invert pitch
    controllerData.yaw = 3000 - mapStickInput(YAW_PIN, yawZero);       // Invert yaw
    controllerData.throttle = mapThrottleInput(THROTTLE_PIN);

    if (!digitalRead(EM_PIN)) {
        if (electromagnetToggleCount >= 20) {
            controllerData.electromagnet = !controllerData.electromagnet;
            electromagnetToggleCount = 0;
        }
    }
    electromagnetToggleCount = min(electromagnetToggleCount + 1, 20);

    // // Read value printing
    // Serial.print("ARM: ");
    // Serial.println(controllerData.arm);

    // Serial.print("Roll: ");
    // Serial.println(controllerData.roll);

    // Serial.print("Pitch: ");
    // Serial.println(controllerData.pitch);

    // Serial.print("Yaw: ");
    // Serial.println(controllerData.yaw);

    // Serial.print("Throttle: ");
    // Serial.println(controllerData.throttle);

    // Serial.print("Electromagnet: ");
    // Serial.println(controllerData.electromagnet);

    esp_now_send(broadcastAddress, (uint8_t *)&controllerData, sizeof(controllerData));
    delay(50);
}

//*************************************Helper Functions**************************************
int mapStickInput(int pin, int zeroValue) {
    int value = analogRead(pin);

    // Handle deadzone
    if (abs(value - zeroValue) <= 25) {
        return 1500; // Neutral position
    }else{
      // Map relative to zeroValue
      if (value > zeroValue) {
          return map(value, zeroValue, 4095, 1500, 2000); // Above center
      } else {
          return map(value, 0, zeroValue, 1000, 1500); // Below center
      }
    }
}

int mapThrottleInput(int pin) {
    int value = analogRead(pin);
    value = map(value, 0, 4095, 1000, 2000);

    if (value > 1600) {
        throttleValue = min(throttleValue + 50, 2000);
    } else if (value < 1400) {
        throttleValue = max(throttleValue - 50, 1000);
    }

    return throttleValue;
}
