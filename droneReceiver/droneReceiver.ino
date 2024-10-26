#include "HardwareSerial.h"
#include "sbus.h"
#include <esp_now.h>
#include <WiFi.h>

//*************************************Output Pin Globals**************************************

//Electromagnet - EM
int EM = 0;

//****************************************SBUS Transmitter Globals**************************************

// Create a HardwareSerial object
HardwareSerial SerialPort(1);  // Use UART1 (TX2 on ESP32)
// Initialize SBUS transmitter
bfs::SbusTx sbus_tx(&SerialPort, 16, 17, true);  // RX pin 16, TX pin 17, inverted signal (true)
  
bfs::SbusData sbusData; //Create object to store values to transfer into sbus TX

//****************************************ESP NOW Reciever Globals**************************************

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
  int roll=1500;
  int pitch=1500;
  int yaw=1500;
  int throttle=1000;
  bool em_enable=false;
  bool arm_enable=false;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// callback function that will be executed when data is received
void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));


  updateTelem(myData.roll, myData.pitch, myData.yaw, myData.throttle, myData.arm_enable);
  

  Serial.print("ARM: ");
  Serial.println(myData.arm_enable);

  Serial.print("Roll: ");
  Serial.println(myData.roll);

  Serial.print("Pitch: ");
  Serial.println(myData.pitch);

  Serial.print("Yaw: ");
  Serial.println(myData.yaw);

  Serial.print("Throttle: ");
  Serial.println(myData.throttle);

  Serial.print("EM: ");
  Serial.println(myData.em_enable);
  digitalWrite(EM, myData.em_enable);

}

//*************************************************************************************************

void setup(){
  //*************************************Output Pin Setup**************************************
  pinMode(EM, OUTPUT);
  
  //****************************************SBUS Transmitter Setup**************************************

  Serial.begin(115200);
  while (!Serial) {}
  /* Begin the SBUS communication */
  sbus_tx.Begin();

  //1977 in code maxxed out beta flight at 2115
  //8 in code minimummed beta flight to 885
  //8-1977
  //885-2115

  // Set SBUS channel values to 0 for default
  sbusData.ch[0] = 0;  // Roll
  sbusData.ch[1] = 0;  // Pitch
  sbusData.ch[2] = 0;  // Throttle
  sbusData.ch[3] = 0;  // Yaw
  sbusData.ch[4] = 0;  //ARM??
  sbusData.ch[5] = 0;
  sbusData.ch[6] = 0;
  sbusData.ch[7] = 0;
  sbusData.ch[8] = 0;
  sbusData.ch[9] = 0;
  sbusData.ch[10] = 0;
  sbusData.ch[11] = 0;
  sbusData.ch[12] = 0;
  sbusData.ch[13] = 0;
  sbusData.ch[14] = 0;
  sbusData.ch[15] = 0;

  // Optional: Set failsafe, lost frame, and additional channels
  sbusData.failsafe = false;
  sbusData.lost_frame = false;
  sbusData.ch17 = false;
  sbusData.ch18 = false;

  //Uncomment to print out all sbus channel values
  // for (int8_t i = 0; i < sbusData.NUM_CH; i++) {
  //       Serial.print(sbusData.ch[i]);
  //       Serial.print("\t");
  //     }

  // Pass the data to the SbusTx object
  sbus_tx.data(sbusData);

  //Write SBUS data to transmit
  sbus_tx.Write();

  //****************************************ESP NOW Reciever Setup**************************************

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);

  pinMode(26, OUTPUT); //Can this be deleted?? Doesnt seem to do anything

  //*************************************************************************************************
}

void loop(){ 

  


}

void updateTelem(int roll, int pitch, int yaw, int throttle, bool arm){
  sbusData.ch[0] = convertRange(roll);  // Roll
  sbusData.ch[1] = convertRange(pitch);  // Pitch
  sbusData.ch[2] = convertRange(throttle);  // Throttle
  sbusData.ch[3] = convertRange(yaw);   // Yaw
  
  if(arm){
    sbusData.ch[4] = convertRange(2000);   // aux1
  }else{
    sbusData.ch[4] = convertRange(1000);   // aux1
  }

  // Pass the data to the SbusTx object
  sbus_tx.data(sbusData);

  //Write SBUS data to transmit
  sbus_tx.Write();
  //delay(100);

}

int convertRange(int input) {
    int input_min = 885; //Theoretical min is 885
    int input_max = 2115; //Theoretical mas is 2115
    int output_min = 8;
    int output_max = 1977;

    int output = output_min + ((input - input_min) * (output_max - output_min)) / (input_max - input_min);
    return output;
}

