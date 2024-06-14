# MAE 148 Cart Pickup Delivery System: Eric Foss, Aaron Marshall, and Taymor Tsepina

This repository contains the original ROS2 Foxy package that is run on the Jetson Nano for Team 8's Final Project. It communicates over the Robocar wifi network with a ROS2 package running on a seperate cart that is hitched to the back of the RC car using a magnetic attachment. This cart is run by a BeagleBone Blue running Ubuntu 20.04, using a sourced ROS2 Foxy installation. The custom built package run on the BeagleBone can be found here: https://github.com/eric-foss/cart . This package is also run alongside third party packages: ucsd_robocar_actuators2, fusion-engine-driver, and ntrip_client. 

## Nodes:
HitchNode: This node begins communication with the cart using a group of publishers and subscribers. A command is sent to the cart to begin spinning to attempt a hitch with the magnetic mount. When a hitch is sensed by the IMU on the BeagleBone Blue, a message is sent back to the RC car and begins communication with ForwardNode.

ForwardNode: This node is "called" by the hitch node when a hitch is sensed, and begins moving the car forward by publishing Twist commands to the vesc controlling the RC Car. Once the car reaches a final destination, movement is ceased and the DispenseNode is "called".

DispenseNode: This node re-establishes communication with the cart, sending commands to a node that controls the servos, dropping the payloads at the reached destination. 

## Outside Packages
Two pre-installed packages on the Docker Image were utilized: ucsd-robocar-actuators for communicating with the on-board vesc using a ROS2 topic, and fusion-engine-driver to obtain coordinates from on-board GPS module. To obtain RTK corrected GPS measurments through Polaris, ntrip_client (https://github.com/LORD-MicroStrain/ntrip_client) was installed and formatted to properly communicate with fusion-engine-driver. 



