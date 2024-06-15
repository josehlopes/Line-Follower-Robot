# Line-Follower-Robot

Line-Follower Robot with Obstacle Avoidance

This project implements a line-following robot that can also avoid obstacles using ultrasonic sensors. The robot's control is done through a PID algorithm to keep the robot on the line and avoid obstacles when necessary.

## Components Used

- Arduino Uno
- Ultrasonic Sensor (HC-SR04)
- DC Motors
- H-Bridge (L298N or similar)
- Line Sensors (LDR, TCRT5000, or similar)
- Resistors
- Breadboard and connecting wires

## Pin Configuration

- **Trigger**: Pin 13 (Ultrasonic)
- **Echo**: Pin 12 (Ultrasonic)
- **MotorL**: Pin 3 (Left Motor)
- **MotorR**: Pin 4 (Right Motor)
- **SensorL**: Pin A0 (Left Line Sensor)
- **SensorR**: Pin A1 (Right Line Sensor)

## Installation

1. Clone the repository:
    ```sh
    git clone https://github.com/your-username/line-follower-robot.git
    ```

2. Open the file in the Arduino IDE.

3. Connect the components according to the described pin configuration.

4. Upload the code to your Arduino.

## Usage

After uploading the code to the Arduino, place the robot on the black line and turn on the system. The robot should follow the line and automatically avoid obstacles.

## Contributions

Contributions are welcome! Feel free to open issues or submit pull requests.

## License

This project is licensed under the MIT License. See the LICENSE file for more details.

---