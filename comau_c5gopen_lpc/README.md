# c5gopen_lpc_node

This package implements the node running on the C5GOPEN LPC. The node is a pure c++ applications without any dependencies from ROS and catkin tool.

The node has the role to:
1. Deals with the eORL interface.
2. Communicate with an external PC, through MQTT.

The package comau_c5gopen_lpc is developed and tested on:
- Linux Mint 17.1 Rebecca 32bit (equivalent to Ubuntu 14.04.6 LTS) released on Comau LPC
- cmake 2.8.3
- c++11
- eORL version 2.42.7



# How to run the c5gopen_lpc_node
- install the eORL library (the eORL versione 2.42.7 is supported)
- download the package
- mkdir build
- cd build
- cmake ..
- make
- sudo ./c5gopen_lpc_node


# C5GOPEN MQTT publisher topics

Topics published by the c5gopen_lpc_node:
- "robot/armXXX/real_joints_positions"
- "robot/armXXX/real_joints_velocities"
- "robot/armXXX/target_joints_positions"
- "robot/armXXX/target_joints_velocities"
- "robot/armXXX/real_cartesian_positions"
- "robot/armXXX/target_cartesian_positions"
- "robot/armXXX/motor_currents"

Example to read the topic:

mosquitto_sub -h BROKER_IP_ADDRESS -t robot/arm1/target_joints_trajectory



# C5GOPEN MQTT subscriber topics

Topics subscribed by the c5gopen_lpc_node are defined in the configuration file cfg/c5gopen_cfg, an example to publish a topic with mosquitto_pub is:

mosquitto_pub -h BROKER_IP_ADDRESS -t robot/arm3/target_joints_trajectory -m "123.4567123.4567123.4567123.4567123.4567123.4567123.4567123.4567123.4567123.4567" --repeat 1000 --repeat-delay 1

N.B: the c5gopen_lpc_node expects a 80bytes of mosquitto payload, 8bytes for 10 joints as in the example above.


# Dependencies
- Boost
- cnr_logger (https://github.com/CNR-STIIMA-IRAS/cnr_logger)
- mosquitto 


# WARNING
 This package is under heavy development huge changes can happen.
