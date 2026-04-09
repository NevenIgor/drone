# Visual Odometry Drone System

ROS 2-based autonomous drone system with visual odometry, IMU fusion, and mission planning capabilities.

## 📋 Overview

This project implements a complete visual odometry system for autonomous drone navigation using:
- **Visual Odometry (VO)** - Position estimation from camera images
- **IMU Fusion** - Accurate yaw estimation from ICM-20948 sensor
- **Mission Planning** - Waypoint navigation and auto missions
- **GPS Bridge** - MSP protocol integration for flight controller communication

## 🏗️ Architecture

```
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│   simple_vo     │────▶│   mission_node   │────▶│    msp_bridge   │
│  (Visual Odom)  │     │  (Mission Ctrl)  │     │  (GPS/UART API) │
└─────────────────┘     └──────────────────┘     └─────────────────┘
         ▲                                                │
         │                                                ▼
┌─────────────────┐                               ┌──────────────┐
│ icm20948_driver │                               │Flight Controller│
│    (IMU)        │                               │   (Betaflight) │
└─────────────────┘                               └──────────────┘
```

## 📦 Packages

### 1. simple_vo
Visual odometry node with IMU fusion for position estimation.

**Features:**
- ORB feature detection and tracking
- IMU-assisted motion detection
- Yaw estimation from IMU data
- Position estimation from visual features
- TF2 transform broadcasting

**Topics:**
- Subscribes: `/image_raw` (sensor_msgs/Image), `/imu/data` (sensor_msgs/Imu)
- Publishes: `/odom` (nav_msgs/Odometry)

**Parameters:**
- `camera_fps` (default: 30.0)
- `feature_count` (default: 200)
- `scale` (default: 0.001)
- `motion_threshold` (default: 0.1)
- `use_imu` (default: true)

### 2. mission_node
Mission control and waypoint navigation system.

**Features:**
- Waypoint mission execution
- Auto RTL (Return to Launch)
- Multiple flight modes: MANUAL, AUTO, RTL, LOITER
- Real-time position monitoring
- Mission status reporting

**Topics:**
- Subscribes: `/odom` (nav_msgs/Odometry), `/mission/command` (std_msgs/String)
- Publishes: `/mission/target` (geometry_msgs/PoseStamped), `/mission/mode` (std_msgs/String), `/mission/status` (std_msgs/String)

**Parameters:**
- `wp_radius` (default: 2.0) - Waypoint acceptance radius in meters
- `rtl_altitude` (default: 5.0) - RTL altitude in meters

**Commands:**
- `TAKEOFF` - Initiate takeoff sequence
- `LAND` - Initiate landing sequence
- `RTL` - Return to launch point
- `MISSION_START` - Start predefined mission
- `MISSION_STOP` - Stop current mission

### 3. msp_bridge
MSP (MultiWii Serial Protocol) bridge for flight controller communication.

**Features:**
- UART communication with flight controller
- GPS coordinate conversion (Lat/Lon ↔ Local XY)
- Flight mode switching
- Telemetry data parsing

**Topics:**
- Subscribes: `/odom` (nav_msgs/Odometry), `/mission/target` (geometry_msgs/PoseStamped), `/mission/mode` (std_msgs/String)
- Publishes: GPS and telemetry data

**Parameters:**
- `uart_port` (default: /dev/ttyUSB0)
- `baudrate` (default: 115200)
- `base_lat`, `base_lon`, `base_alt` - Home position coordinates
- `kp` (default: 0.5) - Control gain

### 4. icm20948_driver
ICM-20948 IMU sensor driver.

**Features:**
- I2C communication with ICM-20948 sensor
- Accelerometer and gyroscope data reading
- High-frequency data publishing (200Hz)
- Configurable scale parameters

**Topics:**
- Publishes: `/imu/data` (sensor_msgs/Imu)

**Parameters:**
- `i2c_bus` (default: /dev/i2c-1)
- `i2c_address` (default: 0x68)
- `accel_scale` (default: 8.0)
- `gyro_scale` (default: 1000.0)

## 🔧 Installation

### Prerequisites
- ROS 2 Humble Hawksbill or later
- Ubuntu 22.04 LTS
- OpenCV with contrib modules
- Python 3.10+
- C++17 compiler

### Build Instructions

```bash
# Clone the repository
cd ~/ros2_ws/src
git clone <repository_url>

# Install dependencies
rosdep install --from-paths . --ignore-src -r -y

# Build the workspace
cd ~/ros2_ws
colcon build --symlink-install

# Source the environment
source install/setup.bash
```

## 🚀 Usage

### Launch All Nodes

```bash
ros2 launch src launch/all_nodes.launch.py
```

### Launch Individual Nodes

```bash
# IMU Driver
ros2 run icm20948_driver icm20948_node

# Visual Odometry
ros2 run simple_vo simple_vo_node

# Mission Control
ros2 run mission_node mission_node

# MSP Bridge
ros2 run msp_bridge msp_bridge
```

### Example Mission Command

```bash
# Send takeoff command
ros2 topic pub /mission/command std_msgs/msg/String "{data: 'TAKEOFF'}"

# Send waypoint mission
ros2 topic pub /mission/command std_msgs/msg/String "{data: 'MISSION_START'}"
```

## 🔬 Testing

Each package includes standard ROS 2 tests:

```bash
# Run tests for all packages
colcon test

# View test results
colcon test-result --all
```

## 📁 Project Structure

```
.
├── src/
│   ├── simple_vo/           # Visual odometry (Python)
│   │   ├── simple_vo/
│   │   │   └── simple_vo_node.py
│   │   ├── test/
│   │   ├── package.xml
│   │   └── setup.py
│   ├── mission_node/        # Mission control (Python)
│   │   ├── mission_node/
│   │   │   ├── mission_node.py
│   │   │   └── odom_emulator.py
│   │   ├── test/
│   │   ├── package.xml
│   │   └── setup.py
│   ├── msp_bridge/          # MSP protocol bridge (C++)
│   │   ├── src/
│   │   │   └── msp_bridge.cpp
│   │   ├── package.xml
│   │   └── CMakeLists.txt
│   └── icm20948_driver/     # IMU driver (C++)
│       ├── src/
│       │   └── icm20948_node.cpp
│       ├── package.xml
│       └── CMakeLists.txt
├── build/
├── install/
├── log/
└── README.md
```

## ⚙️ Configuration

### Camera Setup
Ensure your camera is properly connected and calibrated. Modify `camera_fps` parameter based on your camera specifications.

### IMU Calibration
For accurate yaw estimation, calibrate the ICM-20948 sensor before first use:
```bash
# Place drone on flat surface
# Run calibration routine (if available)
```

### Flight Controller Setup
1. Connect flight controller via USB/UART
2. Configure Betaflight/Cleanflight for MSP protocol
3. Set correct UART port and baudrate in `msp_bridge` parameters

### Base Station Coordinates
Set your home position coordinates for accurate GPS conversion:
```python
base_lat: 55.7558  # Moscow example
base_lon: 37.6173
base_alt: 150.0
```

## 🛠️ Troubleshooting

### Common Issues

**Camera not detected:**
```bash
ls /dev/video*
# Should show /dev/video0 or similar
```

**IMU communication failure:**
```bash
sudo i2cdetect -y 1
# Should show device at 0x68
```

**UART permission denied:**
```bash
sudo usermod -a -G dialout $USER
# Log out and back in
```

**VO drift:**
- Increase `feature_count` parameter
- Ensure good lighting conditions
- Check camera focus and calibration

## 📄 License

MIT License - See individual package.xml files for specific license declarations.

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## 📞 Support

For issues and questions, please open an issue on the GitHub repository.

## 🙏 Acknowledgments

- ROS 2 Community
- OpenCV Contributors
- Betaflight/Cleanflight Projects
- ICM-20948 Datasheet and Reference Implementation
