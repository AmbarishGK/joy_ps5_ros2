# Joystick Control for PS5

## Overview

This project provides joystick control functionality for the PS5 DualSense controller. It integrates motion sensors, touchpad input, and standard joystick controls. Additionally, it leverages the `joy` node for handling general controller inputs. The data from these inputs are published on separate topics as well as combined into a single topic for convenience.

## Features

- **Motion Sensors**: Captures accelerometer and gyroscope data.
- **Touchpad Input**: Processes and publishes touchpad interactions.
- **Joystick Controls**: Handles standard joystick inputs.
- **Combined Data**: Publishes all data in a unified format.

## Published Topics

1. **`motion_data`**: Publishes motion sensor data including accelerometer and gyroscope readings.
2. **`touchpad_data`**: Publishes touchpad input data.
3. **`joy_data`**: Publishes standard joystick control inputs.
4. **`dual_sense_ext_topic`**: Publishes a combined dataset of motion, touchpad, and joystick controls.

## Prerequisites

- ROS 2 Humble
- DualSense controller (PS5)
- `joy` node installed

## Installation

1. Clone the repository:
   ```bash
   git clone <repository_url>
   cd <repository_name>
   ```
2. Build the package:
   ```bash
   colcon build
   ```
3. Source the setup file:
   ```bash
   source install/setup.bash
   ```
4. Run each script seperatly
   ```bash
   ros2 run <package> <script>
   ```

## Usage

1. Connect the DualSense controller to your system via Bluetooth or USB.
2. Launch file still being prepared
<!-- 2. Run the launch file to start all nodes:
   ```bash
   ros2 launch <package_name> <launch_file>.launch.py -->
   <!-- ``` -->
3. Echo the topics to view data:
   ```bash
   ros2 topic echo /motion_data
   ros2 topic echo /touchpad_data
   ros2 topic echo /joy_data
   ros2 topic echo /dual_sense_ext_data
   ```

## Node Details

- **Motion Sensor Publisher**: Captures and publishes accelerometer and gyroscope data.
- **Touchpad Publisher**: Processes and publishes touchpad interactions.
- **Joystick Publisher**: Handles and publishes joystick inputs.
- **Combined Publisher**: Combines motion, touchpad, and joystick data into a single message.

## Future Work

- Add support for additional controller features.
- Improve the data processing pipeline for higher accuracy.
- Extend compatibility to other controller types.
- Add haptic force feedback

## Contributing

Contributions are welcome! Please fork the repository, create a feature branch, and submit a pull request. I am looking for more feature enchaning by using normalization of inputs from the joystick.

## License

This project is licensed under the [MIT License](LICENSE). Or something that lets you use it.

