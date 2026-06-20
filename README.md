<div align="center">

# Autonomous Surveillance and Navigation Bot

### Distributed 6WD Edge Surveillance Rover — UIU CSE 4326 (Group 4)

[![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi%204-c51a4a?logo=raspberrypi&logoColor=white)](https://www.raspberrypi.com/)
[![MCU](https://img.shields.io/badge/MCU-ESP32%20%7C%20Arduino%20Uno-00979d?logo=arduino&logoColor=white)](#hardware-components)
[![ML](https://img.shields.io/badge/ML-TensorFlow%20Lite-FF6F00?logo=tensorflow&logoColor=white)](https://www.tensorflow.org/lite)
[![Model](https://img.shields.io/badge/Model-SSD%20MobileNet%20v2-blue)](#machine-learning-stack)
[![Course](https://img.shields.io/badge/Course-CSE%204326-purple)](https://www.uiu.ac.bd/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

**United International University · B.Sc. CSE · Section K · Mr. Fahim Hafiz**

*A three-tier heterogeneous robot that runs SSD MobileNet v2 vision at 18–22 FPS on a Raspberry Pi 4 while keeping motor control on dedicated real-time MCUs.*

[Features](#-key-features) · [Architecture](#-system-architecture) · [Quick Start](#-quick-start) · [Paper](#-documentation) · [Team](#-team)

</div>

---

## Project Gallery

<table>
<tr>
<td width="50%"><img src="readme_assets/rover_front.jpg" alt="Front sensor deck"/></td>
<td width="50%"><img src="readme_assets/rover_back.jpg" alt="Rear compute deck"/></td>
</tr>
<tr>
<td align="center"><sub><b>Sensor deck</b> — ultrasonic array, IR modules, wiring harness</sub></td>
<td align="center"><sub><b>Compute deck</b> — Raspberry Pi 4, Logitech webcam, power distribution</sub></td>
</tr>
</table>

> Add `rover_front.jpg` and `rover_back.jpg` to `readme_assets/` for gallery images. Full IEEE paper figures live in `docs/overleaf/image/`.

---

## Table of Contents

- [Overview](#overview)
- [Key Features](#-key-features)
- [System Architecture](#-system-architecture)
- [Hardware Components](#hardware-components)
- [Machine Learning Stack](#machine-learning-stack)
- [Wiring Reference](#-wiring-reference)
- [Software Stack](#software-stack)
- [Quick Start](#-quick-start)
- [Repository Structure](#-repository-structure)
- [Usage](#usage)
- [Results](#results)
- [Limitations & Future Work](#limitations--future-work)
- [Team](#-team)
- [Documentation](#-documentation)
- [License](#license)

---

## Overview

**Autonomous Surveillance and Navigation Bot** is a 6-Wheel Drive (6WD) indoor security rover developed for *Microprocessors and Microcontrollers Laboratory* (**CSE 4326**, Section **K**) at **United International University (UIU)**, Dhaka, Bangladesh.

Single-board robots often fail when vision inference, web streaming, and motor PWM compete on one processor. This project separates workloads across **three compute tiers**:

| Tier | Hardware | Responsibility |
|------|----------|----------------|
| Orchestration | Raspberry Pi 4 | Flask HUD, OpenCV stream, SSD MobileNet v2 inference, GPS/compass, A* planning |
| Aggregation | ESP32 | 360° proximity, climate/gas sensing, OLED HUD, command safety gate |
| Execution | Arduino Uno | Real-time L298N motor control, skid-steer logic |

**Submission:** Research Journal Paper · **Date:** 20 June 2026 · **Group:** 4

---

## Key Features

- **18–22 FPS** edge object detection (`person`, `chair`, `table`, etc.) via TensorFlow Lite / ONNX Runtime
- **Sub-45 ms** dashboard-to-motor command latency
- **Hardware safety override** — ESP32 blocks drive commands inside 30 cm proximity envelope
- **Zero-radius 6WD skid steering** using three ganged L298N drivers
- **Live web HUD** — MJPEG vision feed, GIS map, indoor blueprint A* routing, 100 ms telemetry
- **Total BOM ≈ BDT 6,684** (excluding university-provided Pi 4)

---

## System Architecture

```
┌──────────────────────────────────────────────────────────────┐
│  ORCHESTRATION — Raspberry Pi 4 Model B (4 GB)               │
│  Flask · OpenCV · SSD MobileNet v2 · NEO-6M GPS · HMC5883L   │
└────────────────────────────┬─────────────────────────────────┘
                             │ USB serial @ 115200 bps
┌────────────────────────────▼─────────────────────────────────┐
│  AGGREGATION — ESP32 DevKit V1                               │
│  HC-SR04 ×3 · IR ×3 · DHT11 · MQ-2 · SSD1306 OLED            │
└────────────────────────────┬─────────────────────────────────┘
                             │ UART (5 V → 3.3 V divider)
┌────────────────────────────▼─────────────────────────────────┐
│  EXECUTION — Arduino Uno R3 · 3× L298N · 6WD skid steer      │
└──────────────────────────────────────────────────────────────┘
```

Telemetry flows **upstream** every 100 ms; validated drive commands flow **downstream**.

---

## Hardware Components

| Component | Role | Qty |
|-----------|------|-----|
| Raspberry Pi 4 Model B (4 GB) | Host node | 1 |
| ESP32 DevKit V1 | Sensor aggregator + safety | 1 |
| Arduino Uno R3 | Motor controller | 1 |
| NEO-6M GPS | Outdoor localization | 1 |
| HMC5883L Compass | Heading correction | 1 |
| L298N Dual H-Bridge | Motor driving | 3 |
| HC-SR04 Ultrasonic | Obstacle ranging | 3 |
| Digital IR Proximity | Near-field detection | 3 |
| SSD1306 OLED | Local diagnostics | 1 |
| DHT11 / MQ-2 | Climate / gas monitoring | 1 each |
| Logitech Webcam | Front vision | 1 |
| 6WD Chassis + 18650 pack | Mobility + power | 1 |

---

## Machine Learning Stack

| Layer | Technology |
|-------|------------|
| Runtime | **TensorFlow Lite** (local edge inference, no cloud dependency) |
| Model | **SSD MobileNet v2** — Single Shot Detector + depthwise-separable backbone |
| Weights | **COCO** pre-trained classes |
| Input | **300 × 300** RGB tensor via OpenCV |
| Deployment | ONNX Runtime session on Pi (`detect.onnx`) |
| Performance | **18–22 FPS**, decoupled from motor control loop |

Detections trigger policy actions (e.g. **person → emergency halt**) without blocking the Arduino real-time loop.

---

## Wiring Reference

<details>
<summary><b>Raspberry Pi 4 ↔ GPS & Compass</b></summary>

| Signal | NEO-6M | HMC5883L |
|--------|--------|----------|
| VCC | Pin 1 (3.3 V) | Pin 17 (3.3 V) |
| GND | Pin 6 | Pin 9 |
| Data | TX→Pin 10, RX←Pin 8 | SDA↔Pin 3, SCL↔Pin 5 |

</details>

<details>
<summary><b>ESP32 sensor pins</b></summary>

| Sensor | Pins |
|--------|------|
| Left / Right / Rear HC-SR04 | GPIO 5/18 · 26/27 · 15/23 |
| Left / Right / Rear IR | GPIO 14 · 12 · 13 |
| DHT11 | GPIO 4 |
| MQ-2 | GPIO 33 (ADC) |

</details>

<details>
<summary><b>ESP32 ↔ Arduino (voltage divider)</b></summary>

- ESP32 GPIO 17 (TX2) → Arduino Pin 11 (RX)
- Arduino Pin 12 (TX) → ESP32 GPIO 16 (RX2) via **1 kΩ / 2 kΩ** divider (5 V → 3.33 V)
- Common ground across all tiers

</details>

Pin tables, ASCII interconnect diagram, and assembly steps are in the [IEEE paper](docs/overleaf/main.tex).

---

## Software Stack

| Layer | Stack |
|-------|-------|
| Arduino Uno | C++ · `SoftwareSerial` |
| ESP32 | C++ · `ArduinoJson`, `DHT`, `Adafruit_SSD1306`, `Wire` |
| Raspberry Pi | Python 3 · Flask, OpenCV, ONNX Runtime, pynmea2, smbus2, pyserial |
| Dashboard | HTML/CSS/JS · Leaflet.js |
| Planning | Custom **A\*** on 100×100 occupancy grid from uploaded floorplans |

---

## Quick Start

### Prerequisites

- Raspberry Pi 4 with camera, I2C, and serial enabled
- Arduino IDE (+ ESP32 board package)
- Python 3.9+

### 1. Clone

```bash
git clone https://github.com/RaihanRafi12/Autonomous-Surveillance-and-Navigation-Bot.git
cd Autonomous-Surveillance-and-Navigation-Bot
```

### 2. Flash firmware

| Board | Sketch |
|-------|--------|
| Arduino Uno R3 | `firmware/arduino_uno/uno_motor_controller.ino` |
| ESP32 DevKit V1 | `firmware/esp32/esp32_telemetry.ino` |

### 3. Raspberry Pi host

```bash
cd raspberry_pi_host
pip install -r requirements.txt
# Place detect.onnx and coco_labels.txt in this directory
python3 app.py
```

Open `http://<pi-ip>:5000` for the control console.

---

## Repository Structure

```
Autonomous-Surveillance-and-Navigation-Bot/
├── firmware/
│   ├── arduino_uno/          # Real-time motor control
│   └── esp32/                # Sensor aggregation + OLED
├── raspberry_pi_host/
│   ├── app.py                # Flask server + vision + A*
│   ├── requirements.txt
│   ├── coco_labels.txt
│   ├── templates/index.html  # Web HUD
│   └── static/               # Uploaded floorplans
├── docs/overleaf/            # IEEE two-column paper (LaTeX)
│   ├── main.tex
│   ├── IEEEtran.cls
│   └── image/                # Paper figures
├── readme_assets/            # README gallery images
├── LICENSE
└── README.md
```

---

## Usage

| Mode | How |
|------|-----|
| Manual drive | On-screen D-pad → ESP32 safety check → Arduino motors |
| Outdoor nav | Double-tap GIS map to lock GPS waypoint |
| Indoor nav | Upload floorplan → tap start (A) and goal (B) → A* path |
| Vision | Live annotated MJPEG at `/video_feed` |
| Telemetry | REST `/api/telemetry` polled every 200 ms |

---

## Results

Field-tested in a **2,500 sq. ft.** facility:

| Metric | Value |
|--------|-------|
| Vision FPS | 18–22 |
| Command latency | < 45 ms |
| Safety envelope | 30 cm (auto STOP) |
| Telemetry rate | 100 ms |

---

## Limitations & Future Work

- IR sensors false-trigger under strong ambient light or near glass
- GPS unavailable indoors — compass-assisted dead reckoning used
- **Planned:** 2D LiDAR + SLAM for large-facility autonomous mapping

---

## Team

**Group 4 — CSE 4326 (Section K)**

| Name | Student ID | Email |
|------|------------|-------|
| MD. Shafatul Haque Mahim | 0112320084 | mmahim2320084@bscse.uiu.ac.bd |
| Raihan Raf | 0112320164 | rrafi2320164@bscse.uiu.ac.bd |
| Farin Ferdous | 0112320238 | fferdous2320238@bscse.uiu.ac.bd |
| Abdullah Al Mamun | 0112320163 | amamun2320163@bscse.uiu.ac.bd |
| Abid Afzal Galib | 0112320086 | agalib2320086@bscse.uiu.ac.bd |

**Course Instructor:** Mr. Fahim Hafiz

---

## Documentation

- **[IEEE Paper Source](docs/overleaf/main.tex)** — compile with `pdflatex main.tex` inside `docs/overleaf/`
- **[Figure checklist](docs/overleaf/image/README.md)** — required PNG/JPG assets for the paper
- **[GitHub Repository](https://github.com/RaihanRafi12/Autonomous-Surveillance-and-Navigation-Bot)**

### References

1. TensorFlow Lite — lightweight edge inference runtime  
2. A. Howard *et al.*, MobileNets, arXiv:1704.04861, 2017  
3. W. Liu *et al.*, SSD: Single Shot MultiBox Detector, ECCV 2016  
4. Lin *et al.*, Microsoft COCO, ECCV 2014  

---

## License

This project is released under the [MIT License](LICENSE).

<div align="center">

*Built by Team MissionFail — UIU CSE · June 2026*

</div>
