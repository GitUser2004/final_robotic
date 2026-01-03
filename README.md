# final_robotic

this repository explains the "Autonomous Mobile Robot for Indoor Medicine Delivery Using Color-Based Vision and Micro-ROS", and implementation of a low-cost autonomous mobile robot for indoor medicine delivery in hospital-like environments. The proposed system integrates color-based computer vision, wireless communication, and reactive navigation to autonomously reach designated rooms identified by color markers while safely avoiding dynamic obstacles. The robot architecture is based on an ESP32 microcontroller, an ESP32-CAM vision system using OpenCV, and a modular control structure compatible with micro-ROS. The delivery task is initiated through a numeric command sent from a user interface, which is mapped to a target color corresponding to a specific room. The robot continuously processes visual data to detect the target color and estimate distance based on detected circular markers, while obstacle avoidance is handled through a low-cost scanning distance sensor. Experimental results demonstrate that the robot can successfully navigate toward target locations, adapt to unexpected obstacles, and complete delivery tasks without human intervention. The modular architecture allows scalability and future integration of advanced localization and planning techniques. This work contributes a flexible, affordable solution for autonomous indoor delivery systems, particularly suited for healthcare and educational environments.

## Environment Setup

### Prerequisites

Ensure you have the following installed:

#### Arduino IDE (Micro-ros)

For programming the esp32 DevKit1 and the espcam its necessary to install the followings libraries in Arduino IDE:

```bash
esp_camera.h
WiFi.h
WebServer.h

# all of this libraries are neccesary only for programming the espcam, otherwise there are additional instrucctions inside the code

WiFi.h
Wire.h
VL53L0X.h
ESP32Servo.h
WebServer.h
PubSubClient.h
ArduinoJson.h
# micro-ROS
micro_ros_arduino.h

# this libraries are for programming the principal esp32, it needs to be updated
```

### Python

```bash
# Install this libraries in python for the espcam
pip install opencv-python==4.11.0.86
pip install numpy==2.3.1

# This dependencies are for the "artificial vision" in the recognition of the colors
```

### ROS2 HUMBLE

```bash

sudo apt install ros-humble-desktop
sudo apt install ros-humble-ros-base
sudo apt install ros-dev-tools
# Check ROS2 Humble installation
ros2 --version
```

In the "esp-cam" folder there are two archives, the .ino is for programming the espcam, this only needs to be with power, the .py in the folder is on ly for visualize the camera in live, otherwise is not neccesary, the camera will works anyway.

### Create Workspace

```bash
# Create and build workspace
mkdir -p ~/autonomous_ws/
cd ~/autonomous_ws/
```

the following steps are neccesary for the first exercise
- copy all the files in the path **ros2/src/dqn_robot_nav/dqn_robot_nav/environment.py** is necessary to copy the folder **src** in the workspace **autonomous_ws**
- make a colcon build
```bash
colcon build
source install/setup.bash
```

once al this steps will be complet it will be ready for running

in one terminal and in the workspace
```bash
ros2 run decision_node decision_node
```
and the interface between the misions and the esp32 will be complete

For testing:
```bash
ros2 topic list
```
to see the nodes.

For the last step is needed to running docker for wifi (for this is neccesary to be in the same network)
```bash
docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888 -v6
```


## Interface
The following link is to access to interact with all:
**https://lauch01.github.io/robotic_web/**

To use the interface, the following steps:

1. **Select the room (A, B, C, D)**
2. **Set an hour**


## Extras

In the folders **screenshots**, **3d_models** and **circuit-design** are all the files to build the robot, between .stl, .json and references of components and parts.