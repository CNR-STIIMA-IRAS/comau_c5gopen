# c5gopen_lpc_node

This package implents the node running on the C5GOPEN LPC. The node is a pure c++ applications without any dependencies from ROS and catkin tool.
The node has the role to:
1. deal with the eORL interface.
2. communicate with the ROS comau_hw_interface.

The current package is developed and tested with the eORL version 2.42.7.

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
