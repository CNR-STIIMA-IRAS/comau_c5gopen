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
- install the eORL library (the eORL version 2.42.7 is supported)
- download the package
- mkdir build
- cd build
- cmake ..
- make

N.B. To transfer the file on the Comau LPC from a remote PC scp -r ./<c5Gopen_folder_package> c5gopen@IP_OF_COMAU_LPC:/home/c5gopen/<YOUR_WS>


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
```
mosquitto_sub -h BROKER_IP_ADDRESS -t robot/arm1/target_joints_trajectory
```

MQTT topics are published as JSON strings, the topics referred to joint values are in the form:
```
{
  "J1" : 0.0, 
  "J2" : 0.0,
  "J3" : 0.0,
  "J4" : 0.0,
  "J5" : 0.0,
  "J6" : 0.0,
  "J7" : 0.0,
  "J8" : 0.0,
  "J9" : 0.0,
  "J10" : 0.0,
  "time" : 123456789123456789
}
```


# C5GOPEN MQTT subscriber topics

Topics subscribed by the c5gopen_lpc_node are defined in the configuration file cfg/c5gopen_cfg. The topic needs to be in the following JSON format:
```
{
  "J1" : 0.0, 
  "J2" : 0.0,
  "J3" : 0.0,
  "J4" : 0.0,
  "J5" : 0.0,
  "J6" : 0.0,
  "J7" : 0.0,
  "J8" : 0.0,
  "J9" : 0.0,
  "J10" : 0.0
}
```

Example to read the topic:
```
mosquitto_pub -h <BROKER_IP_ADDRESS> -t robot/arm1/target_joints_trajectory -m "{\"J1\" : 0.0, \"J2\" : 0.0, \"J3\" : 0.0, \"J4\" : 0.0, \"J5\" : 0.0, \"J6\" : 0.0, \"J7\" : 0.0, \"J8\" : 0.0, \"J9\" : 0.0, \"J10\" : 0.0 }"  --repeat NUMB_REPETITIONS --repeat-delay DELAY_IN_MS
```

# Dependencies

- Boost
- cnr_logger (https://github.com/CNR-STIIMA-IRAS/cnr_logger)
- mosquitto 
- jsoncpp


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


# CONTACT
Enrico Villagrossi: enrico.villagrossi@stiima.cnr.it


