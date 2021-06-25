# dt100_driver

ROS driver for the Imagenex DT100 multibeam profiling sonar. The driver:

  1. launches a Windows XP virtual machine, which then runs DT100.exe automatically upon up-start. During execution, DT100.exe recieves raw data from the DT100 sonar and then processes these packets using its propriatary beam-forming algorithms. Data packets are then output from DT100.exe in 83P point format, which is a format propriatary to Imagenex
  2. Data packets from DT100.exe are then parsed by the DT100_relay node, which converts these packets to **sensor_msgs/pointcloud2**messages. These messages are then published on the topic**/DT100_scans**

UDP is used internally to communicate between the virtual machine and relay node.

TO-DO LIST:

-report when the virtual machine has powered up
-handle virtual machine crashes or loss of connection
-terminate DT100.ex gracefully before closing virtual machine (sonar will not connect without reset if DT100 is abruptly closed)
