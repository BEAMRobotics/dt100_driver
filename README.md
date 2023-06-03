# dt100_driver

This repository contains code for operating the [DT100 multibeam sonar](https://imagenex.com/products/dt100) from Imagenex within ROS by:

  1. Launching a Windows XP virtual machine (VM) which runs DT100.exe automatically on VM up-start. DT100.exe has been configured with the following parameters:
      - Units: m
      - Range: 40m (should be manually set to within expected depths)
      - Gain: 6db (typical for silty, sand conditions)
      - Sound Velocity: 1500m/s
      - Sector Size: 120 degrees
      - Number of Beams: 240
      - Averaging: 5 shot
      - Gain Equalization: off
      - Automatic Gain Control: off
      - Profile Point Setup
          - Profile Point Filter: Maximum Return
          - Minimum Depth (below sonar): 0.1m
          - Maximum Depth (below sonar): 100m
  2. During execution, DT100.exe receives raw data from the DT100 sonar head and then processes these packets using its proprietary beam-forming algorithms. Data packets are then output from DT100.exe in [83P Profile Point format](docs/supporting_documents.pdf), which is a format proprietary to Imagenex
  3. 83P Profile Point data packets from DT100.exe are then parsed by the DT100_relay node, which converts these packets to sensor_msgs/pointcloud2 messages. These messages are then published on the topic /DT100_scans using the following [frame convention](docs/sonar_frame.pdf).

---

## Dependency

See the following [link](https://github.com/BEAMRobotics/beam_robotics/wiki/Beam-Robotics-Installation-Guide) for installing dependencies on beam robots.

---

## Install

For development, use the following commands to download and compile the package.

```shell
cd ~/catkin_ws/src
git clone git@github.com:BEAMRobotics/dt100_driver.git
cd ..
catkin build
```

---

## Run the package

---

Physically connect the DT100 Sonar ethernet cable to an ethernet switch. Physically connect an ethernet cable from the host machine to the same ethernet switch. Run `ipconfig` in a terminal on the host machine to find the name of the ethernet adapter, which is declared as `bridge_adapter` in **Parameters**. Once resolved, run the following commands:

```shell
sudo ifconfig bridge_adapter 192.168.0.4 netmask 255.255.255.0
roslaunch dt100_driver dt100_driver.launch
```

---

## Parameters

in `dt100_driver.launch`, you will see two parameters:

```xml
<param name="bridge_adapter" value="enx000fc910b495" />
<param name="headless" value="true" />
```

`bridge_adapter` is mandatory and must match the name of the ethernet adapter on your host machine. When launching the driver on a robot over ssh, `headless` should be left as `true`, though for tuning `false` may be selected to manually change parameters in the virtual machine if needed.

---

## TODO

- [ ] develop node that interrogates DT100.exe with the VM and dynamically changes operating range and gain

---
