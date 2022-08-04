# UAV_Gimbal

The UAV_Gimbal is a set of packages developed to stabilize a gimbal system through the mountain skyline. The repository packages are describe in the following list:

a. [Altimeter](#altimeter)

b. [Gimbal_controller](#controller)

c. [Perception](#perception)

d. [Particle_fusion](#particle_fusion)

## Prerequisites
We tested the packages in **Ubuntu 18.04** and **ROS Melodic**, the system was run on a **Jetson Nano 2GB**.

## Datasets
The datasets used to evaluate the current system can be downloaded from this [link](https://peridot-sailor-9cd.notion.site/Hierarchical-Sampling-based-Particle-Filter-for-Visual-Inertial-Gimbal-in-the-Wild-Datasets-4c8f3a2d27e54ab0871d930571f365ee).

# Altimeter
The current package measures the altitude above mean sea level (AMSL).

## 1. Prerequisites
The packaged is build to read an BMP180 Sensor. Software requirements are described in the [root folder](README.md) of this repository

## 2. Build altimeter package
Clone the repository and build through catkin_make:

```
  cd ~/catkin_ws/src
  git clone https://github.com/hl49/uav_gimbal.git
  cd ../
  catkin_make
  source ~/catkin_ws/devel/setup.bash
```

Allow execution permission to the *altimeter_node.py* file
```
  cd ~/catkin_ws/src/uav_gimbal/altimeter/scripts
  chmod +x altimeter_node.py
```

## 3. Running the Altimeter node
3.1 In order to run the altimeter node run the command:
```
  rosrun altimeter altimeter_node.py
```

3.2 The BMP180 data is publish in the default topic */altimeter/BMP180*, which is a custom topic with the structure described bellow:
```
  std_msgs/Header header
  float32 altitude
  float32 temperature
```
The *altitud* is given in meters and the *temperature* in Celsius degrees

# Controller
The current package controls the orientation of a gimbal system built with 2 servos [Dynamixel xl430-w250-t](https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/), the hardware interface used to communicate the servos with the Jetson Nano is the [OpenCR 2.0](https://emanual.robotis.com/docs/en/parts/controller/opencr10/).

## 1. Prerequisites
The servos driver and communication packages are required to run the current module. The sofware dependencies are described in the [dynamixel interface respository](https://github.com/csiro-robotics/dynamixel_interface).
The current package requires the sensor information from BNO055 IMU, the sensor driver is found in the[
ros_imu_bno055](https://github.com/BytesRobotics/bno055) repository.

## 2. Build gimbal_controller package
Clone the respository in your default environment folder (in this example catkin_ws):
```
  cd ~/catkin_ws/src
  git clone https://github.com/hl49/uav_gimbal.git
  cd ../
  catkin_make
  source ~/catkin_ws/devel/setup.bash
```

Allow execution permission to the *gimbal_controller_node.py* file
```
  cd ~/catkin_ws/src/uav_gimbal/gimbal_controller/scripts
  chmod +x gimbal_controller_node.py
```
## 3. Running the Gimbal Controller Node
3.1 Connect the *Opencr* to the pc, and turn on the servo power supply.

3.2 Run the BNO055 ros node to read the data from the inertial sensor:
```
rosrun bno055 bno055_ros.py
```
3.3 Run the custom launch file required to initialize the servo driver:
```
roslaunch gimbal_controller dynamixel_interface_controller.launch
```
3.4 Run the controller node:
```
rosrun gimbal_controller gimbal_controller_node.py
```
3.5 Position reference from input topics

The gimbal controller keeps the initial position until a reference position is sent through [perception node](../perception) and [bno055 ros](https://github.com/BytesRobotics/bno055). The gimbal_controller_node subcribes to the topics `/perception_rpy` and `/imu/data`. The first topic provides reference position relative to the skyline processed by the perception node, and the second topic contains the imu measurements published by the ros_imu_bno055 node.
 
# Perception
The following package allows to identify the sky line and a ground plane from outdoor images in real time. It takes a stabilized image as a reference and detects the movement of subsequent frames to send the signal to the servos.

## 1. Prerequisites
Download and install the repositories [HELLO AI WORLD NVIDIA JETSON](https://github.com/dusty-nv/jetson-inference) for running the segmentation neural network and [ROS DEEP LEARNING](https://github.com/dusty-nv/ros_deep_learning) for ROS1 interface. Focus mainly on [Semantic Segmentation with SegNet](https://github.com/dusty-nv/jetson-inference/blob/master/docs/segnet-console-2.md) and how to run a demo with a [Semantic Segmentation live camera](https://github.com/dusty-nv/jetson-inference/blob/master/docs/segnet-camera-2.md). A Raspberry Pi camera (MIPI CSI camera) was used to record our demo video.
The pretrained network model is in the [model](./model) folder. It was used [Skyfinder Dataset](https://cs.valdosta.edu/~rpmihail/skyfinder/) as dataset.

## 2. Build gimbal_perception package
Clone the respository in your default environment folder (in this example catkin_ws):
```
  cd ~/catkin_ws/src
  git clone https://github.com/hl49/uav_gimbal.git
  cd ../
  catkin_make
  source ~/catkin_ws/devel/setup.bash
```
2.1 Include pretrained network modified:
First, copy the content from the [model](./model) folder into the *jetson-inference/python/examples* directory. Then, modify the content of the segnet.ros1.launch according to the following lines:
```
TODO
```

## 3. Running the Gimbal Perception Node
3.1 Run the live camera segmentation network: 
```
roslaunch ros_deep_learning segnet.ros1.launch
```
3.2 Run the perception node:
```
rosrun perception perception_node.py
```

# Particle_Fusion
The following package allows to identify the sky line and a ground plane from outdoor images in real time. It takes a stabilized image as a reference and detects the movement of subsequent frames to send the signal to the servos.

## 1. Prerequisites
Download and install the repositories [HELLO AI WORLD NVIDIA JETSON](https://github.com/dusty-nv/jetson-inference) for running the segmentation neural network and [ROS DEEP LEARNING](https://github.com/dusty-nv/ros_deep_learning) for ROS1 interface. Focus mainly on [Semantic Segmentation with SegNet](https://github.com/dusty-nv/jetson-inference/blob/master/docs/segnet-console-2.md) and how to run a demo with a [Semantic Segmentation live camera](https://github.com/dusty-nv/jetson-inference/blob/master/docs/segnet-camera-2.md). A Raspberry Pi camera (MIPI CSI camera) was used to record our demo video.
The pretrained network model is in the [model](./model) folder. It was used [Skyfinder Dataset](https://cs.valdosta.edu/~rpmihail/skyfinder/) as dataset.

## 2. Build gimbal_perception package
Clone the respository in your default environment folder (in this example catkin_ws):
```
  cd ~/catkin_ws/src
  git clone https://github.com/hl49/uav_gimbal.git
  cd ../
  catkin_make
  source ~/catkin_ws/devel/setup.bash
```
2.1 Include pretrained network modified:
First, copy the content from the [model](./model) folder into the *jetson-inference/python/examples* directory. Then, modify the content of the segnet.ros1.launch according to the following lines:
```
TODO
```

## 3. Running the Gimbal Perception Node
3.1 Run the live camera segmentation network: 
```
roslaunch ros_deep_learning segnet.ros1.launch
```
3.2 Run the perception node:
```
rosrun perception perception_node.py
```
