#include "HardwareSerial.h"
#include "sbus.h"
#include <esp_now.h>
#include <WiFi.h>

//*********************************** SBUS Protocol Explanation ***********************************//
// The SBUS protocol is a serial communication protocol developed by Futaba for RC systems. 
// It is widely used to transmit control signals from receivers to devices like flight controllers.

// Key Characteristics:
// - **11-bit Resolution:** Each channel value is encoded as an 11-bit integer, allowing values between 0 and 2047.
// - **Up to 16 Channels:** SBUS supports up to 16 standard channels, plus 2 additional digital channels (ch17 & ch18).
// - **High-Speed Serial Communication:** SBUS operates at 100,000 baud rate (asynchronous serial).
// - **Inverted Signal:** SBUS uses an inverted signal logic, meaning idle state is 0V (low) and active state is 3.3V/5V (high).

// SBUS Packet Structure (25 bytes total per frame):
// - Byte 0: Start byte (always 0x0F).
// - Bytes 1-22: 16 channels encoded as 11-bit values packed into 22 bytes (bitwise concatenation).
// - Byte 23: Flags byte:
//       - Bit 0: Failsafe (1 = failsafe activated).
//       - Bit 1: Lost Frame (1 = lost connection to the transmitter).
//       - Bit 2-7: Reserved for future use.
// - Byte 24: End byte (always 0x00).

// SBUS Channel Mapping:
// - Each channel value represents a control signal, typically mapped to PWM (Pulse Width Modulation) signals.
// - Typical RC PWM range: 1000 µs (minimum) to 2000 µs (maximum).
// - SBUS practical value range: 8 (minimum) to 1977 (maximum) to ensure compatibility with PWM signals.

// Example Usage in a Drone:
// - Channels are mapped to control axes like roll, pitch, yaw, and throttle.
// - Additional channels are used for auxiliary functions (arming, mode switching, etc.).

// Advantages of SBUS:
// - Compact: All channels are transmitted over a single signal line.
// - High Resolution: 11-bit resolution ensures smooth and precise control.
// - Low Latency: Fast updates at 100 Hz (10 ms per frame).

// Note: The SBUS protocol is digital and does not use traditional PWM signals directly; 
// flight controllers like Betaflight handle the conversion to PWM as needed.
//***********************************************************************************************//

//************************************* Output Pin Globals **************************************
const int EM_PIN = 0; // Electromagnet output pin

//************************************* SBUS Transmitter Globals **************************************
HardwareSerial SerialPort(1); // Use UART1 (TX2 on ESP32)
bfs::SbusTx sbus_tx(&SerialPort, 16, 17, true); // RX pin 16, TX pin 17, inverted signal (true)
bfs::SbusData sbusData; // Stores SBUS values to transmit

//************************************* ESP-NOW Receiver Globals **************************************
typedef struct {
    int roll = 1500;
    int pitch = 1500;
    int yaw = 1500;
    int throttle = 1500;
    bool em_enable = false;
    bool arm_enable = false;
} dataStruct;

dataStruct receivedData;

// Callback function executed when data is received
void onDataReceived(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
    memcpy(&receivedData, incomingData, sizeof(receivedData));
    updateTelemetry(receivedData.roll, receivedData.pitch, receivedData.yaw, receivedData.throttle, receivedData.arm_enable);
    digitalWrite(EM_PIN, receivedData.em_enable);
}

//************************************* Setup Function **************************************
void setup() {
    Serial.begin(115200);

    // Set up electromagnet pin
    pinMode(EM_PIN, OUTPUT);

    // Initialize SBUS transmitter
    sbus_tx.Begin();

    // Initialize default SBUS channel values
    for (int i = 0; i < 16; i++) sbusData.ch[i] = 0;
    sbusData.failsafe = false;
    sbusData.lost_frame = false;
    sbusData.ch17 = false;
    sbusData.ch18 = false;
    sbus_tx.data(sbusData);
    sbus_tx.Write();

    // Set device as a Wi-Fi station
    WiFi.mode(WIFI_STA);

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Register ESP-NOW receive callback
    esp_now_register_recv_cb(onDataReceived);
}

//************************************* Loop Function **************************************
void loop() {
    // The telemetry update and SBUS transmission are handled in the callback
}

//************************************* Helper Functions **************************************
void updateTelemetry(int roll, int pitch, int yaw, int throttle, bool arm) {
    sbusData.ch[0] = convertRange(roll);    // Roll
    sbusData.ch[1] = convertRange(pitch);   // Pitch
    sbusData.ch[2] = convertRange(throttle); // Throttle
    sbusData.ch[3] = convertRange(yaw);     // Yaw
    sbusData.ch[4] = arm ? convertRange(2000) : convertRange(1000); // Arm (aux1)

    sbus_tx.data(sbusData);
    sbus_tx.Write();
}

int convertRange(int input) {
    const int input_min = 885;  // Theoretical minimum: 885
    const int input_max = 2115; // Theoretical maximum: 2115
    const int output_min = 8;
    const int output_max = 1977;

    return output_min + ((input - input_min) * (output_max - output_min)) / (input_max - input_min);
}
