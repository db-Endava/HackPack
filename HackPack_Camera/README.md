
# Vision and Control of Automated Nerf Ball Launcher

## Project Overview
This project uses an ESP32 cam module to host a web interface that sends commands to control an automated Nerf ball launcher.

## Features
- **Private Access Point**: Hosts a private WiFi access point.
- **Web Interface**: Includes a web server for display of the video feed and control buttons for system motion.
- **Communication**: Communicates with simple UART packets to move the launcher to new coordinates.
- **Facial Recognition**: Uses machine vision to track the location of a person's face within the frame.

## Hardware Requirements
- ESP32 CAM Module
- Standard peripherals for ESP32 CAM (e.g., camera, power supply)

## Software and Libraries
1. **Arduino IDE**
2. **ESP32 Board Library** (v2.0.17)
   - Add `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json` to the Additional Board Manager URLs.
   - Ensure the version 2.0.17 is installed for stable operation.
3. **Libraries**:
   - ESP Camera
   - Img Converters
   - ESP HTTP Server
   - Other dependencies as outlined in the project code.

## Setup Instructions
1. **Board Configuration**:
   - Use "AI Thinker ESP32-CAM" as the target board.
2. **Connecting to the Web Interface**:
   - Connect your PC/mobile to the "HackPack" WiFi network.
   - Navigate to `http://hackpack.local` or `192.168.4.XXX` (replace XXX as needed) to access the web interface.

## Usage
- Interact with the Nerf ball launcher through the web interface. Controls for targeting, shooting, and moving the launcher are provided on the webpage.
- The system uses face recognition technology to track faces and can be commanded to target or follow individuals automatically.

## Contribution
Contributors are welcome to propose improvements or add new features. Please fork the repository and submit pull requests for review.

## License
This project is released under the MIT License. Full license details are available in the LICENSE file in the repository.
