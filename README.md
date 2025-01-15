# Drone Control System Documentation

This project involves building a drone control system using an **ESP32 microcontroller** interfaced with the **SpeedyBee F405 flight controller**. Communication between the ESP32 and SpeedyBee is established using the **SBUS protocol** to transmit and receive data, allowing the ESP32 to send commands and control the drone’s orientation. Additionally, a **PID control algorithm** is implemented for enhanced flight stability and precision. The ESP32 utilizes its dual-core architecture with **FreeRTOS** to handle tasks in parallel, where one core processes controller inputs and the other performs PID calculations for motor control.

---

## Table of Contents
1. [Overview](#overview)  
2. [System Components](#system-components)  
3. [ESP32 FreeRTOS Implementation](#esp32-freertos-implementation)  
4. [Communication Setup](#communication-setup)  
5. [PID Controller Integration](#pid-controller-integration)  
6. [How to Use](#how-to-use)  
7. [Future Improvements](#future-improvements)

---

## Overview

The drone system involves:
- **ESP32** for processing inputs, implementing PID control algorithms, and utilizing **FreeRTOS** for parallel task management.
- **SpeedyBee F405 flight controller** to manage drone stabilization and motor control.
- **SBUS Protocol** for communication between ESP32 and SpeedyBee.
- **ESP-NOW Protocol** for wireless data transmission (e.g., joystick controls) to the ESP32.
  
This setup allows real-time orientation and throttle control of the drone, with PID control ensuring optimal flight dynamics.

---

## System Components

- **ESP32**  
  Utilizes dual-core processing and FreeRTOS to split tasks into two main responsibilities:
  1. **Core 1** handles wireless inputs received from the handheld controller via ESP-NOW.
  2. **Core 2** processes these inputs using the PID algorithm and generates corrected control signals for the SpeedyBee.

- **SpeedyBee F405 Flight Controller**  
  The flight controller manages motor output and flight stabilization based on the SBUS data it receives from the ESP32.

- **SBUS Protocol**  
  Used for fast, serial communication between ESP32 and SpeedyBee to control roll, pitch, yaw, throttle, and arming.

- **ESP-NOW**  
  A lightweight wireless communication protocol used for receiving joystick inputs from another ESP32 module.

- **IMU Sensors** (Inertial Measurement Unit)  
  Integrated into the SpeedyBee F405, providing accelerometer and gyroscope data to measure orientation and angular velocity for feedback in the PID controller.

---

## ESP32 FreeRTOS Implementation

The ESP32's dual-core processor enables parallel task execution using **FreeRTOS**, where specific tasks are assigned to each core for efficient performance.

### Task Distribution Across Cores
1. **Core 1 (Input Processing):**
   - Receives directional commands from the handheld controller via ESP-NOW.
   - Parses the incoming data and updates control variables such as desired pitch, roll, yaw, and throttle.
   - Shares these updated setpoints with **Core 2** through task-safe data structures (e.g., FreeRTOS queues).

2. **Core 2 (PID Processing and SBUS Output):**
   - Processes the setpoints received from Core 1.
   - Executes the PID control algorithm for each axis (pitch, roll, yaw, and throttle).
   - Sends the processed SBUS signals to the SpeedyBee F405 flight controller.

### FreeRTOS Task Example
```cpp
// Core 1: Controller Input Task
void inputTask(void *parameter) {
    while (1) {
        // Receive ESP-NOW data (e.g., joystick inputs)
        receiveJoystickData(); 
        // Update shared setpoints (use FreeRTOS queues or mutex)
        updateSetpoints();
        vTaskDelay(pdMS_TO_TICKS(20));  // 20ms delay
    }
}

// Core 2: PID and SBUS Output Task
void pidTask(void *parameter) {
    while (1) {
        // Process setpoints and calculate PID outputs
        calculatePIDforAllAxes();
        // Send updated control signals via SBUS
        sendSbusOutput();
        vTaskDelay(pdMS_TO_TICKS(20));  // 20ms delay
    }
}

void setup() {
    // Create FreeRTOS tasks
    xTaskCreatePinnedToCore(inputTask, "InputTask", 2048, NULL, 1, NULL, 0);  // Core 0
    xTaskCreatePinnedToCore(pidTask, "PIDTask", 2048, NULL, 1, NULL, 1);     // Core 1
}
```

In this implementation:
- **`inputTask`** on Core 1 handles joystick inputs received via ESP-NOW and updates control setpoints.
- **`pidTask`** on Core 2 processes the PID algorithm based on the setpoints and generates SBUS commands for the flight controller.
- **Task delays** ensure consistent execution cycles (20ms for 50Hz updates).

---

## Communication Setup

### 1. **SBUS Communication**
   - The ESP32 sends orientation and throttle data to the SpeedyBee F405 via the SBUS protocol.
   - Data channels include roll, pitch, yaw, throttle, and arm state.

### 2. **ESP-NOW Wireless Communication**
   - An ESP32-based joystick controller sends inputs like throttle, pitch, and roll values to the drone’s ESP32 via ESP-NOW.
   - The joystick commands are mapped to SBUS-compatible control ranges and further refined by the PID algorithm.

---

## PID Controller Integration

### Purpose of the PID Controller
The **PID controller** ensures that the drone maintains stable flight and responds accurately to joystick commands. It minimizes the error between the desired setpoint (e.g., roll, pitch, yaw angles) and the actual measurements from the SpeedyBee’s IMU sensors.

### Workflow
1. **Input Processing:** Core 1 updates setpoints based on joystick inputs.
2. **Error Calculation:** Core 2 calculates the error between setpoints and sensor feedback:
   ```
   error = setpoint - measured_value
   ```
3. **PID Computation:**
   ```
   output = Kp * error + Ki * integral + Kd * derivative
   ```
4. **SBUS Output:** Core 2 sends the corrected outputs to the SpeedyBee F405 via SBUS.

---

## How to Use

1. **Wiring Setup:**
   - Connect the **ESP32 TX/RX pins** to the SpeedyBee F405’s SBUS input.
   - Ensure all components (IMU, motors, ESP32) are properly powered.

2. **Upload the Code:**
   - Upload the FreeRTOS-based code to the ESP32 modules.

3. **Tuning PID:**
   - Use the included PID tuning script to iteratively adjust `Kp`, `Ki`, and `Kd`.

4. **Flight Control:**
   - Use the joystick to control the drone’s throttle, roll, pitch, and yaw.

---

## Future Improvements

- **Dynamic PID Adjustment:**
  - Implement real-time tuning based on flight conditions (e.g., wind).

- **Extended Sensor Integration:**
  - Add external sensors (e.g., GPS, altimeter) for autonomous navigation.

- **Enhanced Safety Features:**
  - Introduce failsafe modes, such as auto-landing during signal loss.

- **Optimize ESP-NOW Range:**
  - Use LoRa or other long-range communication protocols.

---

This project showcases how FreeRTOS, dual-core ESP32 processing, and PID control algorithms can enable precise and stable drone control.

