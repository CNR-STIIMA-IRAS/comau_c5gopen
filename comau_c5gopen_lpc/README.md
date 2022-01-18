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


# Dependencies
- Boost
- cnr_logger (https://github.com/CNR-STIIMA-IRAS/cnr_logger)
- mqtt_cpp (https://github.com/redboltz/mqtt_cpp)


# WARNING
 This package is under heavy development huge changes can happen.
