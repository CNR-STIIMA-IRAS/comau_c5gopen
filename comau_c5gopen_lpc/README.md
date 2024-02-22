# c5gopen_lpc_node

This package implements the node running on the C5GOPEN LPC. The node is a pure C++ application without any dependencies from ROS and catkin tool.
At the current state, this package implements 2 C5GOPEN working mode:
- LISTEN mode
- ABSOLUTE mode

The following instructions are referred to the use of the ABSOLUTE mode.

The node has the role to:
1. Deals with the eORL interface.
2. Communicate with an external PC, through MQTT.

The package comau_c5gopen_lpc is developed and tested on:
- Linux Mint 17.1 Rebecca 32bit (equivalent to Ubuntu 14.04.6 LTS) released on COMAU LPC
- cmake 2.8.3
- c++11
- eORL version 2.42.7



# How to run the c5gopen_lpc_node
- install the eORL library (the eORL version 2.42.7 is supported)
- download the package
- cd comau_c5gopen/comau_c5gopen_lpc
- mkdir build
- cd build
- cmake ..
- make

N.B. To transfer the file on the COMAU LPC from a remote PC scp -r ./<c5Gopen_folder_package> c5gopen@IP_OF_COMAU_LPC:/home/c5gopen/<YOUR_WS>


# C5GOPEN MQTT published topics

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


# C5GOPEN MQTT subscribed topic

Topics subscribed by the c5gopen_lpc_node are defined in the configuration [config file](https://github.com/CNR-STIIMA-IRAS/comau_c5gopen/tree/master/comau_c5gopen_lpc/cfg). The topic needs to be in the following JSON format:
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

- cnr_logger 
```
(https://github.com/CNR-STIIMA-IRAS/cnr_logger)
```

- mosquitto 
```
sudo apt install libmosquitto0-dev
```

- jsoncpp
```
sudo apt-get install libjsoncpp-dev
```

# Usage

Example of usage:

1) Open an ssh shell on COMAU LPC (ssh c5gopen@xxx.xxx.xxx.xxx --- ex. COMAU NS16 IP: 192.168.254.245) PASSWORD: c5gopen

2) sudo pkill -9 mdm* (kill graphical interface that could introduce instability in C5Gopen communication)

3) sudo su (pass: c5gopen)

4) activate the proper PDL program on robot Teach Pendant to switch between C5Gopen modalities (LISTEN (mod 0) - ABSOLUTE (mod 4) - RELATIVE (mod 5) - ADDITIVE (mod 7++)), default modality @ robot DRIVEON is: LISTEN. Examples of PDL programs are provided by COMAU in the folder [PDL](https://github.com/CNR-STIIMA-IRAS/comau_c5gopen/blob/master/comau_c5gopen_lpc/cfg/c5gopen_cfg.yaml) **N.B. For the first trial use the LISTEN mode** 

5) cd <user_path>/<your_ws>/src/comau_c5gopen/comau_c5gopen_lpc/launch 

6) ./launch_c5gopen.sh (when the C5GOPEN is activated the default working mode is the LISTEN mode)
**N.B. If necessary, adjust the paths contained in the file launch_c5gopen.sh according to the local path**

8) in **Automatic mode** push robot DRIVEON button (mandatory to update the eORL internal status), if the node is properly working robot will pass from DRIVEOFF to DRIVEON modality, otherwise it will return a "C5GOPEN communication error". In **Manual mode** keep pushing the deadman switch. **N.B. the C5GOpen mode works both with the robot in manual mode and in automatic mode**

9) press the START (green) button on the robot Tech Pendant to run the PDL program activated at step 4. **N.B. at the current state, it is supported only the JOINT ABSOLUTE mode** 

10) Once the ABSOLUTE mode is activated the user can provide trajectories to the comau_c5gopen_lpc node through an MQTT message. The trajectory can be published on the topic robot/arm1/target_joints_trajectory (see the [config file](https://github.com/CNR-STIIMA-IRAS/comau_c5gopen/blob/master/comau_c5gopen_lpc/cfg/c5gopen_cfg.yaml) to change the topic name) as a streaming of robot absolute joint positions.


**N.B. The LPC non-real time ETH need to be connected in a switch with the service port of the B&R ACOPOS**


# WARNING

This package is under heavy development huge changes can happen.


# CONTACT
Enrico Villagrossi: enrico.villagrossi@cnr.it


