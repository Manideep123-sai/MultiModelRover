# MultiModelRover

## Overview
MultiModelRover is a full-stack robotics project featuring a custom-built rover with hardware control, a Python server backbone, and a modern web interface. The rover can be operated in automated obstacle avoidance mode or manual control via Bluetooth, and supports complex navigational logging like shortest-path return.

## Project Structure
This repository contains the essential files needed to build, program, and control the rover:

*   **`pins.txt`**: Detailed hardware wiring guide. It contains pin mappings for the Arduino, Bluetooth module (HC-05), Servo Motor, Ultrasonic Sensor (HC-SR04), L298N Motor Driver, and Power/Battery configurations.
*   **`rovercode.txt`**: The Arduino firmware (effectively an `.ino` file). This script handles motor actuation, obstacle avoidance logic using the ultrasonic sensor, and direct Bluetooth command processing.
*   **`server.py`**: The Python backend. Built with Flask and PySerial, it bridges communication between the frontend interface and the rover. It also calculates movement durations and track paths to execute features like "exact return" and "shortest return".
*   **`index.html`**: The frontend User Interface. A sleek, glassmorphic layout that provides a D-Pad for manual control, a terminal for sequence programming, calibration tools, unified path logging, and experimental voice-activated controls.

## Setup Instructions

### 1. Hardware Assembly
Read the `pins.txt` file out carefully to properly wire all electronic components to your Arduino. 
Once the hardware is connected, copy the contents of `rovercode.txt` into the Arduino IDE and upload it to your board.

### 2. Connect via Bluetooth
Power on the Rover. Pair your HC-05 Bluetooth module with your Windows PC (the default PIN is usually `1234` or `0000`). Find out the COM port assigned to the Bluetooth device by Windows. 
If necessary, update the `COM_PORT = "COM6"` line in `server.py` to match the exact COM port.

### 3. Run the Backend Server
Ensure you have Python installed, along with the required libraries:
```bash
pip install flask flask-cors pyserial
```

Then, run the server:
```bash
python server.py
```

### 4. Open the Interface
Open `index.html` in your preferred web browser. Click the **Connect** button on the interface to establish the link with the Python backend. You can now drive your rover!
