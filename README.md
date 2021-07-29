# dt100_driver

## Description
ROS driver for the Imagenex DT100 multibeam profiling sonar. The driver:

  1. Launches a Windows XP virtual machine (VM) in headless mode, which then runs DT100.exe automatically on VM up-start. During execution, DT100.exe recieves raw data from the DT100 sonar and then processes these packets using its propriatary beam-forming algorithms. Data packets are then output from DT100.exe in 83P Profile Point format, which is a format propriatary to Imagenex
  2. Data packets from DT100.exe are then parsed by the DT100_relay node, which converts these packets to **sensor_msgs/pointcloud2** messages. These messages are then published on the topic **/DT100_scans**

## Dependency

See the following [link](https://github.com/BEAMRobotics/beam_robotics/wiki/Beam-Robotics-Installation-Guide) for installing dependencies on beam robots.

## Install

For development, use the following commands to download and compile the package.

```
cd ~/catkin_ws/src
git clone git@github.com:BEAMRobotics/dt100_driver.git
cd ..
catkin build
```
## Run the package

Run the launch file:
```
roslaunch dt100_driver dt100_driver.launch
```
## Parameters

in `dt100_driver.launch`, you will see two parameters:

```
<param name="bridge_adapter" value="enp0s31f6" />
<param name="ip_address" value="192.168.0.4" />
```
`bridge_adapter` is mandatory and must match the name of the ethernet adapter on your host machine. With the network configuration assumed by the Windows XP VM, `ip_address` should be left as `192.168.0.4` and has been left as a parameter for debugging.
