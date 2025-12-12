# COMP0221 Flocking System & Visualiser

This repository contains the coursework implementation for **COMP0221 (Networking & RTOS)**. It consists of an ESP32 firmware for distributed flocking control via LoRa/WiFi and a Python-based 3D MQTT visualiser.

## 📂 Repository Structure

```
├── CW1/
│   ├── CW1/         # ESP32 Firmware (ESP-IDF project)
│   └── report/      # LaTeX Coursework Report
├── COMP0221-MQTT-Visualiser/
│   ├── visualiser.py       # Main 3D Visualiser
│   ├── data_logger.py      # Terminal-based Data Logger
│   ├── requirements.txt    # Python dependencies
│   └── ...
└── environment.yml  # Conda environment file
```

## 🛠️ Prerequisites

1.  **Hardware**: ESP32 Development Boards (e.g., TTGO LoRa32)
2.  **Software**:
    *   **ESP-IDF v5.x**: [Installation Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/)
    *   **Anaconda / Miniconda**: For Python environment management.

---

## 🚀 Setup Guide

### 1. Python Environment (`comp0221`)

Create the required Conda environment for the visualiser:

```bash
# Create environment from file
conda env create -f environment.yml

# Activate environment
conda activate comp0221
```

*Alternatively, using pip:*
```bash
pip install -r COMP0221-MQTT-Visualiser/requirements.txt
```

### 2. Firmware Configuration (ESP32)

⚠️ **IMPORTANT SECURITY STEP**: Before compiling, you **MUST** configure your credentials.

1.  Open `CW1/CW1/components/common/include/config.h`.
2.  **WiFi Settings**: Update `HOME_WIFI_SSID` and `HOME_WIFI_PASS`.
3.  **Eduroam**: If using Eduroam, update `EDUROAM_IDENTITY` and `EDUROAM_PASSWORD`.
4.  **Team ID**: Set `MY_TEAM_ID` to your assigned group number.
5.  **Security Key**: The system uses AES-128 CMAC. Ensure `TEAM_KEY_BYTES` matches the key used by your team and the visualiser.

### 3. Visualiser Configuration

1.  Open `COMP0221-MQTT-Visualiser/visualiser.py`.
2.  Update the `SHARED_KEY` variable (lines ~42) to match the `TEAM_KEY_BYTES` in your firmware `config.h`.

---

## 🏃‍♂️ Running the System

### Step 1: Flash the Firmware

Navigate to the firmware directory and build/flash each drone:

```bash
cd CW1/CW1

# Build, Flash and Monitor
idf.py build flash monitor
```

*Note: You may need to specify the port, e.g., `-p COM3` or `-p /dev/ttyUSB0`.*

### Step 2: Run the Visualiser

Ensure your computer has internet access to connect to the MQTT broker (`broker.emqx.io`).

```bash
conda activate comp0221
cd COMP0221-MQTT-Visualiser

# Run the 3D Visualiser
python visualiser.py
```

### Step 3: Run the Data Logger (Optional)

To record statistics and packet loss data to a CSV file:

```bash
python data_logger.py
```

---

## 📊 What to Expect

1.  **OLED Display**: The ESP32 screen will show the drone's status (Packets Rx/Tx, State).
2.  **Visualiser**:
    *   You should see 3D arrows representing each drone in the swarm.
    *   **Blue/Red/Green arrows** indicate different drones.
    *   **Movement**: Drones should move according to the flocking algorithm (separation, alignment, cohesion).
3.  **Data Flow**:
    *   **LoRa**: Drones exchange position data at 2Hz.
    *   **MQTT**: Telemetry is sent to the broker and displayed on the PC.

## ❓ Troubleshooting

*   **"MQTT Connect Failed"**: Check your internet connection and firewall settings. The broker `broker.emqx.io` port `1883` must be accessible.
*   **"CMAC Mismatch"**: Ensure `TEAM_KEY` in `config.h` matches `SHARED_KEY` in `visualiser.py`.
*   **Build Errors**: Ensure you are using a compatible ESP-IDF version (v5.x recommended). Clean the build with `idf.py fullclean` if weird errors persist.

---

*Verified for submission: 2025-12-12*