# Drone Control System Documentation

This project involves building a drone control system using an **ESP32 microcontroller** interfaced with the **SpeedyBee F405 flight controller**. Communication between the ESP32 and SpeedyBee is established using the **SBUS protocol** to transmit and receive data, allowing the ESP32 to send commands and control the drone’s orientation.

---

## Table of Contents
1. [Overview](#overview)  
2. [System Components](#system-components)  
3. [Communication Setup](#communication-setup)  
4. [Code Explanation](#code-explanation)  
5. [How to Use](#how-to-use)  
6. [Future Improvements](#future-improvements)

---

## Overview

The drone system involves:
- **ESP32** for processing inputs and sending orientation commands.
- **SpeedyBee F405 flight controller** to manage drone stabilization and motor control.
- **SBUS Protocol** for communication between ESP32 and SpeedyBee.
- **ESP-NOW Protocol** for wireless data transmission (e.g., joystick controls) to the ESP32.
  
This setup allows real-time orientation and throttle control of the drone, along with the ability to control auxiliary features like an **electromagnet**.

---

## System Components

- **ESP32**  
  Used as the main processor for interpreting commands, transmitting data via SBUS, and receiving control inputs via ESP-NOW.

- **SpeedyBee F405 Flight Controller**  
  The flight controller manages motor output and flight stabilization based on the SBUS data it receives from the ESP32.

- **SBUS Protocol**  
  Used for fast, serial communication between ESP32 and SpeedyBee to control roll, pitch, yaw, throttle, and arming.

- **ESP-NOW**  
  A lightweight wireless communication protocol used for receiving joystick inputs from another ESP32 module.

- **Electromagnet**  
  Auxiliary component controlled via digital input to the ESP32, allowing for additional payload manipulation.

---

## Communication Setup

1. **SBUS Communication**  
   - The ESP32 sends orientation and throttle data to the SpeedyBee F405 via the SBUS protocol.
   - The data is structured into specific channels for roll, pitch, yaw, throttle, and arm state.

2. **ESP-NOW Wireless Communication**  
   - An ESP32-based joystick controller sends inputs like throttle, pitch, and roll values to the drone’s ESP32 via ESP-NOW.
   - Each message contains the control values in a structured message format.

---

## Code Explanation

### 1. **SBUS Communication**
The **SBUS transmitter** sends flight control data from the ESP32 to the SpeedyBee F405.  

- **SbusTx Initialization:**
  ```cpp
  bfs::SbusTx sbus_tx(&SerialPort, 16, 17, true);  // UART1 with RX/TX pins
  ```
- **Setting SBUS Channels:**
  ```cpp
  sbusData.ch[0] = convertRange(roll);  // Roll
  sbusData.ch[1] = convertRange(pitch); // Pitch
  sbusData.ch[2] = convertRange(throttle); // Throttle
  sbusData.ch[3] = convertRange(yaw);   // Yaw
  ```

### 2. **ESP-NOW Communication**
- **Receiving data from joystick:**
  ```cpp
  void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
    memcpy(&myData, incomingData, sizeof(myData));
    updateTelem(myData.roll, myData.pitch, myData.yaw, myData.throttle, myData.arm_enable);
  }
  ```

### 3. **Handling Inputs from Joystick**
- **Analog readings from potentiometers** are mapped to control values within SBUS limits:
  ```cpp
  int convertRangeAnalog(int input) {
      int output = output_min + ((input - input_min) * (output_max - output_min)) / (input_max - input_min);
      return output;
  }
  ```

### 4. **Electromagnet Control**
- The electromagnet state is controlled via a digital pin based on the received command:
  ```cpp
  digitalWrite(EM, myData.em_enable);
  ```

---

## How to Use

1. **Wiring Setup:**
   - Connect the **ESP32 TX/RX pins** to the SpeedyBee F405’s SBUS input.
   - Ensure **electromagnet control** is connected to the specified output pin.
   - Use **ESP32 joystick module** to transmit inputs via ESP-NOW.

2. **Upload the Code:**
   - Upload the provided code to the ESP32 modules (one for the drone and one for the joystick).

3. **Power On the Drone:**
   - Ensure all components (ESP32, SpeedyBee, electromagnet) are powered.
   - Use the joystick to send control commands wirelessly.

4. **Flight Control:**
   - Adjust the joystick to control the drone’s **throttle, roll, pitch, and yaw**.
   - Use the **arming button** to arm or disarm the drone.

---

## Future Improvements

- **Add PID Tuning:**
  - Optimize the drone's stability by fine-tuning the flight controller parameters.

- **Extend Range:**  
  - Use **LoRa modules** to increase communication range beyond the limits of ESP-NOW.

- **Implement Safety Features:**  
  - Add **failsafe conditions** to automatically disarm the drone if connection is lost.

- **Improve Control Precision:**  
  - Use **IMU sensors** on the joystick to get smoother and more accurate input readings.

---

This project aims to demonstrate how low-cost microcontrollers and wireless communication protocols can be used to build an advanced drone control system.
