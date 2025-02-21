# Motor-Control-with-PID-using-ROS2-and-Arduino

## Prerequisites

Before starting, ensure you have the following installed and set up:

- **Operating System**: Ubuntu 22.04
- **ROS2 Distribution**: Humble
- **Python**: Installed with ROS2
- **Arduino IDE**: For uploading the firmware to the Arduino
- **Colcon**: For building ROS2 packages (`sudo apt install python3-colcon-common-extensions`)
- **Permissions**: Add your user to the `dialout` group to access serial devices
  ```bash
  sudo usermod -a -G dialout $USER
  sudo chmod 666 /dev/ttyACM0
  ```
- **Hardware**:
  - Arduino board
  - Motor driver (e.g., L298N, BTS7960, etc.)
  - Encoded DC motor
  - Power supply

---

## Project Structure
```
buggy_ws/
│── src/
│   ├── motor_control/
│   │   ├── scripts/
│   │   │   ├── motor_publisher.py
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│── install/
│── build/
│── log/
```

---

## Installation and Setup

### 1. Create a ROS2 Workspace
```bash
mkdir -p ~/buggy_ws/src
cd ~/buggy_ws
colcon build --symlink-install
source install/setup.bash
```

### 2. Clone and Set Up the Repository
```bash
cd ~/buggy_ws/src
git clone https://github.com/NitheshS2030/Motor-Control-with-PID-using-ROS2-and-Arduino.git
cd ~/buggy_ws
colcon build --symlink-install
source install/setup.bash
```

### 3. Upload Arduino Code
1. Open Arduino IDE
2. Load the `motor_control.ino` script
3. Select the correct board and port
4. Upload the sketch to the Arduino

---

## Running the System

### 1. Ensure the Serial Device is Recognized
```bash
ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
```
If you see `/dev/ttyACM0`, you're good to proceed.

### 2. Run the ROS2 Node
```bash
cd ~/buggy_ws
source install/setup.bash
ros2 run motor_control motor_publisher
```

### 3. Send Commands to the Robot
Once the `motor_publisher` node is running, use the keyboard to control the robot:
- **`w`** - Move Forward
- **`s`** - Move Backward
- **`a`** - Turn Left
- **`d`** - Turn Right

The Arduino serial monitor should display the corresponding direction once per key press.

---

## Troubleshooting

### 1. `ModuleNotFoundError: No module named 'motor_control.scripts'`
**Solution:** Ensure your Python scripts are inside `motor_control/scripts/` and are executable:
```bash
chmod +x ~/buggy_ws/src/motor_control/scripts/motor_publisher.py
colcon build --symlink-install
source install/setup.bash
```

### 2. `FileNotFoundError: No such file or directory: 'dev/ttyACM0'`
**Solution:** Check if the Arduino is connected and detected:
```bash
ls /dev/ttyACM*
```
If not found, try reconnecting the Arduino or using a different USB port.

### 3. `screen /dev/ttyACM0 9600` Terminating Immediately
**Solution:** Ensure no other program (including ROS2) is using the serial port. Restart the Arduino if needed.

### 4. Motors Not Responding to Commands
- Ensure wiring is correct
- Verify the motor driver is receiving signals from the Arduino
- Check the power supply to the motor driver

---

## Conclusion
This project successfully integrates an encoded motor with PID control using ROS2 and Arduino. The keyboard input in the terminal sends movement commands (`w`, `a`, `s`, `d`), which the Arduino interprets and drives the motor accordingly. The motor direction is displayed once per key press in the Arduino serial monitor.

For any issues, refer to the troubleshooting section or open an issue in the GitHub repository.

Happy coding!
