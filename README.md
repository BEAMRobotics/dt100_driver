# DT100_driver
ROS driver for the Imagenex DT100 multibeam profiling sonar.  A Windows XP virtual machine is launched that receives raw packets from the sonar.  A relay node is launch that forms scans.  UDP is used internally to communicate between the sonar's virtual machine and the scan forming node.  Scans are published on /DT100_scans

TO-DO LIST:

-report when the virtual machine has powered up
-handle virtual machine crashes or loss of connection
-terminate DT100.ex gracefully before closing virtual machine (sonar will not connect without reset if DT100 is abruptly closed)
