# DT100_driver
ROS driver for the Imagenex DT100 multibeam profiling sonar.  UDP is used internally to communicate between the sonar's virtual machine and the scan forming node.  Scans are published on /dt100_scans

TO-DO LIST:

-report when the virtual machine has powered up
-handle virtual machine crashes or loss of connection
-terminate DT100.ex gracefully before closing virtual machine (sonar will not connect without reset if DT100 is abruptly closed)
