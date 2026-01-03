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

# all of this libraries are neccesary only for programming the espcam, otherwise there are additional instrucctions inside the code of 
```

### TurtleBot3 Setup

```bash
# Install TurtleBot3 packages if not already installed
sudo apt install ros-humble-turtlebot3*

# Set TurtleBot3 model
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc

# Set Gazebo model path
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
```

### Create Workspace

```bash
# Create and build workspace
mkdir -p ~/dqn_navigation_ws/
cd ~/dqn_navigation_ws/
```

the following steps are neccesary for the first exercise
- copy all the files of the folder p1 in the workspace **dqn_navigation_ws**
- make a colcon build
```bash
colcon build
source install/setup.bash
```
- in the path **src/dqn_robot_nav/dqn_robot_nav/environment.py** in the line 34 copy the path from the model **src/dqn_robot_nav/models/goal_red/model.sdf** of your devide and replace in the code of the enviroment

once al this steps will be complet it will be ready for training or testing

For training:
- in the **training_node.py** are all the parameters to training a model, the important values are; n_episodes, max_steps_per_episode, in the agent are anoter parameters for a more advanced configuration of the simulation.
  and in the **environment.py** in the line 43 are the actions, that set the action in linear or angular velocity, it can be added or deleted, its important that the same quantity of actions are defined in the line 21 in **train_node.py**, it willl need to be the same.
- once all the parameters are defined it needs to run
  
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
in one terminal and in the workspace
```bash
colcon build
source install/setup.bash
ros2 run dqn_robot_nav train_node
```
and the training will be saved

For testing:
All the workspace is ready to be compiled, it only needs:
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
in one terminal and in the workspace
```bash
colcon build
source install/setup.bash
ros2 run dqn_robot_nav test_node
```


## Exercise 2
for this exercise it is neccesary the following archives of rosbag:
**https://drive.google.com/drive/folders/18W1yZCbDVFbHhpRyzelfaHJ892iqRWMr?usp=drive_link**

To run and visualize the entire TurtleBot3 visual control system, follow these steps in separate terminal windows:

1. **Launch the empty Gazebo environment with TurtleBot3**:
   ```bash
   ros2 launch turtlebot3_gazebo empty_world.launch.py
   ```

2. **Play the Kinect camera recording** (contains the RGB image used by the gesture detector):
   ```bash
   ros2 bag play kinect_data2
   ```

3. **Run the gesture perception node** (detects human pose and publishes commands to `/gesture_command`):
   ```bash
   ros2 run gesture_perception gesture_detector
   ```

4. **Start the micro-ROS agent to communicate with the ESP32** (ensure the ESP32 is connected via USB):
   ```bash
   docker run -it --rm --privileged -v /dev:/dev --network=host microros/micro-ros-agent:humble serial --dev /dev/ttyUSB0 -b 115200
   ```

5. **Monitor the velocity commands received by the robot**:
   ```bash
   ros2 topic echo /cmd_vel
   ```

## Exercise 3

To run and visualize the entire TurtleBot3 visual control system with depth-based safety, follow these steps in separate terminal windows:

1. **Launch the empty Gazebo environment with TurtleBot3**:
   ```bash
   ros2 launch turtlebot3_gazebo empty_world.launch.py
   ```

2. **Play the Kinect camera recording** (provides RGB and depth images for gesture and obstacle detection):
   ```bash
   ros2 bag play kinect_data2
   ```

3. **Run the gesture perception node** (detects human pose, classifies gestures, and computes distance zones):
   ```bash
   ros2 run gesture_perception gesture_detector
   ```

4. **Start the micro-ROS agent to communicate with the ESP32** (ensure the ESP32 is connected via USB):
   ```bash
   docker run -it --rm --privileged -v /dev:/dev --network=host microros/micro-ros-agent:humble serial --dev /dev/ttyUSB0 -b 115200
   ```

5. **Monitor the velocity commands received by the robot** (to see how safety logic modulates speed):
   ```bash
   ros2 topic echo /cmd_vel
   ```