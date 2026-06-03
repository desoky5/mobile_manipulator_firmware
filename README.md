# Mobile Manipulator Firmware

> STM32F401-based embedded firmware for a differential-drive mobile robot — featuring quadrature encoder odometry, MPU6050 IMU interfacing, PID closed-loop motor control, and a bidirectional UART interface feeding a full ROS2 navigation stack (Nav2 / SLAM) running on Raspberry Pi 5.

---

## Overview

This repository contains the embedded C firmware for the mobile base of a mobile manipulator platform. The firmware runs on an **STM32F401CCUx (Black Pill)** microcontroller and handles all real-time sensing and actuation:

- Reads quadrature encoders via hardware timer capture to compute per-wheel RPM
- Reads orientation data from an **MPU6050 IMU over I2C**
- Runs independent **PID speed controllers** for each wheel
- Streams odometry and IMU data back to the Raspberry Pi 5 over **UART**
- Receives velocity commands from the ROS2 serial bridge node and executes them via PWM

The Raspberry Pi 5 runs a full **ROS2 Jazzy** stack: LiDAR scanning, obstacle detection, sensor fusion (EKF), and Nav2 / SLAM-based autonomous navigation.

---

## System Architecture

![System Architecture](https://github.com/desoky5/mobile_manipulator_firmware/blob/b1478070116e158ab456e9f4953c57a290e537c5/img/System%20Architecture.jpg)

---

## Hardware Platform

| Component | Details |
|---|---|
| MCU | STM32F401CCUx (Black Pill, ARM Cortex-M4 @ 84 MHz) |
| SBC | Raspberry Pi 5 |
| Motors | DC motors with quadrature encoders (differential drive) |
| IMU | MPU6050 (6-DOF, I2C) |
| LiDAR | RPLiDAR (USB → Raspberry Pi) |
| MCU–SBC Link | UART (TX/RX) |
| IDE | STM32CubeIDE |
| HAL | STM32 HAL / LL Drivers |

---

## Firmware Architecture

### Timer Peripheral Allocation

| Timer | Function |
|---|---|
| TIM3 | Quadrature encoder — Left wheel |
| TIM4 | Quadrature encoder — Right wheel |
| TIM2 | PWM output — H-bridge motor driver (left & right) |
| TIM5 | PID control tick (periodic interrupt) |

### Key Firmware Modules

#### Quadrature Encoder Interface
- TIM3 and TIM4 configured in **encoder mode (TI1+TI2)** for 4× resolution edge counting.
- Delta counts are sampled on each PID tick and converted to RPM using a constant derived from encoder CPR and sampling period.
- Counters reset each tick to avoid accumulation rollover.

#### MPU6050 IMU (I2C)
- Gyroscope and accelerometer data read over I2C on each control tick.
- Raw angular velocity and linear acceleration are packetized and included in the UART telemetry frame sent back to the Raspberry Pi.
- Used by the `robot_localization` EKF node on the Pi for sensor-fused odometry.

#### PID Motor Speed Controller
- Independent PID loops run per wheel on the TIM5 periodic interrupt.
- Error computed between target RPM (from UART command) and measured RPM (from encoder delta).
- Output clamped and mapped to TIM2 PWM duty cycle driving the H-bridge.
- Shared state variables declared `volatile`; critical sections guard multi-byte reads.

#### Bidirectional UART Interface (USART2)
- **RX:** Receives velocity commands from the ROS2 serial bridge node (`/cmd_vel` → per-wheel RPM targets).
- **TX:** Transmits telemetry frames containing raw odometry (encoder counts) and IMU data back to the Pi.
- The serial bridge node on the Pi publishes this data as `/odom` and `/imu` ROS2 topics.

---

## ROS2 Stack (Raspberry Pi 5)

| Node | Role |
|---|---|
| `rplidar_node` | Publishes `/scan` from RPLiDAR over USB |
| `obstacle_detector.py` | Consumes `/scan`, publishes `/cmd_vel` |
| `UART Serial Bridge` | Bridges `/cmd_vel` to STM32; publishes `/odom` and `/imu` from STM32 telemetry |
| `robot_localization` EKF | Fuses `/odom` + `/imu` → `/odometry/filtered` |
| Nav2 / SLAM | Autonomous navigation and map building using filtered odometry + `/scan` |

> ROS2 companion package: **Under Integration**
---

## Getting Started

### Prerequisites

- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) (v1.13+)
- STM32F401CCUx (Black Pill) board
- ST-Link V2 programmer or USB DFU
- IAR Embedded Workbench (optional — EWARM project files included)

### Build & Flash

1. Clone the repository:
   ```bash
   git clone https://github.com/desoky5/mobile_manipulator_firmware.git
   ```
2. Open STM32CubeIDE → `File → Import → Existing Project` → select the cloned directory.
3. Build: `Project → Build All`
4. Flash via ST-Link: `Run → Debug` or `Run → Run`

### Pin Configuration

Open `mobile_manp.ioc` in STM32CubeMX to view or modify the full peripheral and pin assignment configuration.

---

## Project Status

| Feature | Status |
|---|---|
| Quadrature encoder interfacing (TIM3/TIM4) | ✅ Complete |
| PID closed-loop motor speed control | ✅ Complete |
| PWM motor output (TIM2) | ✅ Complete |
| UART bidirectional command + telemetry (USART2) | ✅ Complete |
| MPU6050 IMU over I2C | ✅ Complete |
| ROS2 serial bridge node | ✅ Complete |
| RPLiDAR + obstacle detection node |🔄 In Progress|
| `robot_localization` EKF sensor fusion |🔄 In Progress|
| Nav2 autonomous navigation | 🔄 In Progress |
| SLAM / mapping | 🔄 In Progress |

---

## Author

**Omar Desoky**
Mechatronics & Robotics Engineering — Egypt-Japan University of Science and Technology (E-JUST)

[![GitHub](https://img.shields.io/badge/GitHub-omar--desoky-181717?logo=github)](https://github.com/omar-desoky)
[![LinkedIn](https://img.shields.io/badge/LinkedIn-Connect-0A66C2?logo=linkedin)](https://www.linkedin.com/in/omar-desoky/)

---

## License

This project is open source. See [LICENSE](LICENSE) for details.
