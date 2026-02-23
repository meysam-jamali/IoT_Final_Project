# Smart Shoes - IoT Wearable Sensor System

A comprehensive IoT project that transforms ordinary footwear into intelligent, data-driven smart shoes. This system captures environmental, physiological, and motion data from the wearer using embedded sensors, processes it via an ESP32 microcontroller, and transmits it to cloud platforms for real-time monitoring and analysis.

<img width="1335" height="847" alt="image" src="https://github.com/user-attachments/assets/b8e03e15-f7d0-4877-805d-8ad76427f7f2" />

## 🌟 Features

- **Environmental Sensing**
    - Temperature & Humidity (DHT22)
    - Atmospheric Pressure Simulation
- **Motion & Gait Analysis**
    - 6-Axis Motion Tracking (MPU6050: Accelerometer + Gyroscope)
    - Gait Detection (Walking, Running, Stopped states)
    - Step-based Activity Classification
- **Physiological Monitoring**
    - Heart Rate Simulation (context-aware BPM)
- **Location Intelligence**
    - GPS Positioning (NEO6M Module simulation)
    - Speed, Altitude, and Distance Calculation (Haversine formula)
    - Real-time Location Tracking
- **Device Management**
    - Battery Level Simulation
    - Time & Date Stamping
- **Cloud Integration**
    - Data Publishing via MQTT to Ubidots
    - Remote Monitoring Dashboard
- **Mobile Interface**
    - Android Application for Real-Time Data Visualization

## 🔧 Hardware Components

| Component | Function |
| --- | --- |
| **ESP32 Development Board** | Main microcontroller with Wi-Fi/Bluetooth |
| **DHT22 Sensor** | Measures temperature and humidity inside the shoe |
| **MPU6050 IMU** | Captures acceleration and rotational movement |
| **NEO6M GPS Module** | Provides geolocation and speed data |
| **Potentiometer** | Simulates pressure sensor for gait detection |
| **LEDs (Green, Yellow, Red)** | Visual indicators for walking, running, stopped states |

> *Note: In this implementation, a Wokwi simulation environment was used for development and testing.*
> 

## ⚙️ Software Stack

- **Microcontroller Firmware**: Arduino C++
- **Core Libraries**:
    - `WiFi.h` – Wi-Fi connectivity
    - `PubSubClient.h` – MQTT protocol for cloud communication
    - `TinyGPS++.h` – GPS data parsing (NMEA sentences)
    - `DHT.h` – DHT22 sensor interface
    - `Wire.h`, `MPU6050.h` – I²C communication with IMU
- **Cloud Platform**: [Ubidots IoT](https://ubidots.com/) for data visualization and storage
- **Frontend**: Custom Android app for live monitoring

## 📦 Installation & Setup

### Prerequisites

- Arduino IDE or VS Code with PlatformIO
- ESP32 board support installed
- Internet connection
- Ubidots account and API token

### Configuration Steps

1. **Clone the Repository**
    
    ```bash
    git clone <https://github.com/meysam-jamali/IoT_Smart_Shoes.git>
    cd smart-shoes
    ```
    
2. **Update Credentials**
Open `Sketch.ino` and configure:
    
    ```cpp
    const char* ssid = "YOUR_WIFI_SSID";
    const char* password = "YOUR_WIFI_PASSWORD";
    
    const char* mqtt_server = "industrial.api.ubidots.com";
    const char* token = "BBUS-YOUR_UBIDOTS_TOKEN_HERE";
    ```
    
3. **Install Required Libraries**
Ensure these libraries are installed via Library Manager:
    - DHT sensor library by Adafruit
    - Adafruit Unified Sensor
    - PubSubClient
    - TinyGPS++
    - Wire
    - MPU6050 by Electronic Cats
4. **Upload Code**
Select your ESP32 board in Arduino IDE and upload the sketch.
5. **Connect to Ubidots**
After booting, the device connects to Wi-Fi and starts publishing data under the device label `smart-shoes`.

## 📊 Cloud Dashboard (Ubidots)

Data is streamed in real time to Ubidots and visualized on an interactive dashboard showing:

- Live temperature, humidity, and pressure gauges
- GPS location map
- Gait status indicator
- Heart rate monitor
- Battery level
- Motion vectors (acceleration X/Y/Z)
- Rotational data (gyroscope X/Y/Z)
- Distance traveled counter

📌 *Dashboard screenshots available in the `/docs` folder.*

## 📱 Android Application

A companion Android app displays all sensor readings in real time, enabling mobile health and fitness tracking. The app connects directly to the Ubidots API and updates every 2 seconds.

Features:

- Real-time graphs and value displays
- Location tracking on map
- Gait state notifications
- Low battery alerts
- Historical data view

## 🧪 Usage Example

Once powered:

1. The system initializes all sensors.
2. It connects to Wi-Fi and establishes an MQTT session with Ubidots.
3. Sensor data is collected every 2 seconds and transmitted securely.
4. Gait changes are detected automatically based on pressure and motion patterns:
    - **Stopped** → LEDs off except red
    - **Walking** → Green LED on
    - **Running** → Yellow LED on
5. Simulated heart rate adjusts dynamically based on activity mode.

## 📘 Technical Details

### GPS Simulation (Custom Chip)

A simulated NEO6M GPS chip sends NMEA sentences through UART at 9600 baud. The firmware parses these using TinyGPS++ to extract:

- Latitude / Longitude
- Speed over ground
- UTC Time & Date
- Altitude

### Haversine Distance Calculation

The distance between two GPS points is calculated using the spherical law of cosines:

```cpp
double haversine(double lat1, double lon1, double lat2, double lon2);
```

This enables accurate estimation of total distance traveled during use.

### Power Management

Battery level simulates gradual discharge (~0.5% per cycle), resetting when depleted to model recharging behavior.

## 📁 Project Structure

```
/smart-shoes/
├── firmware/
│   ├── Sketch.ino                # Main Arduino sketch
│   └── lib/                       # Local libraries (if any)
├── docs/
│   ├── gps-neo6m.chip.c           # Custom GPS simulator logic
│   ├── gps-neo6m.chip.json        # Wokwi chip definition
│   └── images/                    # Diagrams and screenshots
├── README.md                      # This file
└── LICENSE
```

## 🤝 Contributing

Contributions are welcome! Please feel free to submit issues, feature requests, or pull requests.

1. Fork the project
2. Create your feature branch (`git checkout -b feature/improved-gait-detection`)
3. Commit your changes (`git commit -am 'Add new algorithm'`)
4. Push to the branch (`git push origin feature/improved-gait-detection`)
5. Open a Pull Request

## 📄 License

MIT License

Copyright (c) 2025 Meysam Jamali, Mohammad Amin Rezaei

Permission is hereby granted...

*(See full LICENSE file for details)*

---

Made with ❤️ for wearable tech, health monitoring, and IoT innovation.

For research, education, and prototyping purposes.
