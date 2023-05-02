# XPLORER Arm Angle Estimator
Package containing code to obtain data from 4 IMUs via the i2C connection in the Raspberry Pi and publish the arm angles.

## Dependecies

You need to have the following packages installed in your system:

[SQUEEZE Custom Messages](https://github.com/ASU-RISE-Lab/squeeze_custom_msgs)

## Package Build

Clone the repository in your colcon workspace and build it

```
cd ~/colcon_ws/src
git clone https://github.com/ASU-RISE-Lab/bno055_raspi_squeeze.git
cd ~/colcon_ws
colcon build --packages-select bno055_raspi_squeeze
```

## IMU Mounting Reference

<img src="resource/IMU Mounting.png" width="600" height="300">

## Circuit Diagram

### **To be Added Soon**

## Launching the node

### Serial Port Opening

In order to open the i2c port, please run the following command:

```
sudo chmod a+rw /dev/i2c-*
```
Run the node by using following command
```
ros2 run bno055_raspi_squeeze imu_node
```
