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


# Usage

Example of usage:

1) Open an ssh shell on Comau LPC (ssh c5gopen@xxx.xxx.xxx.xxx --- ex. Comau NS16 IP: 192.168.254.245) PASSWORD: c5gopen

2) sudo pkill -9 mdm* (kill graphical interface that could introduce instability in C5Gopen communication)

3) sudo su (pass: c5gopen)

4) activate the proper PDL program on robot Teach Pendant to switch between C5Gopen modalities (LISTEN (mod 0) - ABSOLUTE (mod 4) - RELATIVE (mod 5) - ADDITIVE (mod 7++)), default modality @ robot DRIVEON is: LISTEN. **N.B. For the first trial use the LISTEN mode** 

5) cd <user_path>/<your_ws>/src/comau_c5gopen/comau_c5gopen_lpc/launch 

6) ./launch_c5gopen.sh

7) in **Automatic mode** push robot DRIVEON button (mandatory to update the eORL internal status), if the node is properly working robot will pass from DRIVEOFF to DRIVEON modality, otherwise it will return a "C5GOPEN communication error". In **Manual mode** keep pushing the deadman switch. **N.B. the C5GOpen mode works both with the robot in manual mode and in automatic mode**


**N.B. The LPC non-real time ETH need to be connected in a switch with the service port of the B&R ACOPOS**


# WARNING
 This package is under heavy development huge changes can happen.
