
# Automated Nerf Ball Launcher

## Project Overview
This project uses an ESP32 to control an automated Nerf ball launcher. The launcher includes motors for aiming and a firing mechanism, all controlled through an Arduino-based system. This streamlined codebase orchestrates all aspects of the launcher's operation, from initialization to real-time control, ensuring precise and responsive behavior suitable for interactive applications.

## Features
- **Motor Control**: Manages three motors for aiming and firing using DRV8837 drivers.
- **Serial and LED Feedback**: Provides setup feedback through serial communication and visual status indicators using an Adafruit NeoPixel.
- **Sensors and Control**: Utilizes an MPU6050 for orientation data, influencing PID controllers that stabilize and direct the launcher's aim.
- **Launch Sequence**: Implements a state-based launch control for precise timing and execution of firing sequences.
- **Command Processing**: Accepts real-time serial commands to adjust aim and initiate firing, with an optional target tracking mode to follow moving targets.

## Hardware Requirements
- ESP32 Dev Module
- DRV8837 Motor Drivers
- MPU6050 Accelerometer and Gyroscope
- Adafruit NeoPixel LED

## Software and Libraries
1. **Arduino IDE**
2. **ESP32 Board Library** (v2.0.17)
   - Add `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json` to the Additional Board Manager URLs.
3. **Required Libraries**:
   - PID by Brett Beauregard
   - Adafruit NeoPixel by Adafruit
   - I2Cdev (Download from [GitHub](https://github.com/jrowberg/i2cdevlib))

## Setup Instructions
1. **Library Installation**:
   - Install the necessary libraries through the Arduino Library Manager or manually from GitHub.
2. **Board Configuration**:
   - Select "ESP32C3 Dev Module" as the target board.
   - Configure the board settings as specified in the code comments.
3. **Connection**:
   - Connect the components as per the wiring diagram provided in the repository.

## Flashing and Initial Configuration
1. **Flashing the ESP32**:
   - Connect the ESP32 to your computer and select the appropriate COM port.
   - Hold the BOOT button on the ESP32 when connecting for the first time to enter flashing mode.
2. **Initial Setup**:
   - Follow the serial output to ensure components are initialized correctly.

## Usage
- The launcher can be controlled via serial commands. Detailed command documentation is available in the repository.

## Contribution
Contributors are welcome to propose improvements or add new features. Please fork the repository and submit pull requests for review.

## License
This project is released under the MIT License. Full license details are available in the LICENSE file in the repository.
