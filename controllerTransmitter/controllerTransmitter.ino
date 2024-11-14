#include "HardwareSerial.h"
#include "sbus.h"
#include <esp_now.h>
#include <WiFi.h>

//*************************************Potentiometer/Input Pin Globals**************************************
//----------DO NOT USE PINS 6 7 8 9 10 OR 11
//Roll - RY
//Pitch - RX
//Yaw - LY
//Throttle - LX
//Electromagnet - EM
//Arming - ARM

int greenLED = 13; //Was 23
int redLED = 12; //Was 22

int ARM = 36;//Was 36

int LX = 34; //Was 34
int LY = 39; //Was 35
//int LB = 0;

int RightX = 35; //Was 32
int RY = 32; //Was 33
//int RB = 0;

int EM = 0;

//****************************************ESP NOW Transmitter Globals**************************************

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0xd0, 0xef, 0x76, 0x58, 0xc8, 0x30}; // Replace with the MAC address of the receiver ESP32

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  int roll=1500;
  int pitch=1500;
  int yaw=1500;
  int throttle=885;
  bool em_enable=false;
  bool arm_enable=false;
} struct_message;

// Create a struct_message called myData
struct_message myData;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");

  if(status==ESP_NOW_SEND_SUCCESS){
    digitalWrite(greenLED, 1);
    digitalWrite(redLED, 0);
  }else{
    digitalWrite(redLED, 1);
    digitalWrite(greenLED, 0);
  }
}

//*************************************************************************************************
 
void setup() {
  //*************************************Potentiometer/Input Pin Setup**************************************
  pinMode(LX, INPUT);
  pinMode(LY, INPUT);
  //pinMode(LB, INPUT);

  pinMode(RightX, INPUT);
  pinMode(RY, INPUT);
  //pinMode(RB, INPUT);

  pinMode(ARM, INPUT);
  pinMode(EM, INPUT);

  pinMode(greenLED, OUTPUT);
  pinMode(redLED, OUTPUT);
  //****************************************ESP NOW Reciever Setup**************************************
  
  // Init Serial Monitor
  Serial.begin(115200);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Transmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  //*************************************************************************************************
}
 
void loop() {
  
  //Read ARM Enable value
  myData.arm_enable = digitalRead(ARM);
  Serial.print("ARM: ");
  //Serial.println(digitalRead(ARM));
  Serial.println(myData.arm_enable);

  //Get Roll, Pitch, Yaw, and Throttle values from potentiometers
  myData.roll = readStick(RY);
  Serial.print("Roll: ");
  //Serial.println(analogRead(RY));
  Serial.println(myData.roll);

  myData.pitch = readStick(RightX);
  Serial.print("Pitch: ");
  Serial.println(myData.pitch);

  myData.yaw = readStick(LY);
  Serial.print("Yaw: ");
  //Serial.println(analogRead(LY));
  Serial.println(myData.yaw);

  myData.throttle = readStick(LX);
  Serial.print("Throttle: ");
  Serial.println(myData.throttle);

  //Read Electromagnet Enable value 
  //myData.em_enable = digitalRead(EM);
  Serial.print("EM: ");
  Serial.println(myData.em_enable);
  
  
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

  delay(50);

}

int convertRangeAnalog(int input) {
    int input_min = 0;
    int input_max = 4095;
    int output_min = 1000; //Theoretical min is 885
    int output_max = 2000; //Theoretical mas is 2115

    int output = output_min + ((input - input_min) * (output_max - output_min)) / (input_max - input_min);
    return output;
}

int readStick(int pin){
  int value = convertRangeAnalog(analogRead(pin));

  if(pin == LX){
    return value;

  }else{
    if(pin == LY || pin == RY){
      value=3000-value;
    }
    
    if(value<1470 && value>1420){
      return 1500;
    }

    return value;
  }
}