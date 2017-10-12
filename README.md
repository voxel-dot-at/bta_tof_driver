bta_tof_driver
===================
### ROS integration for Bluetechnix devices operated by the Bluetechnix 'Time of Flight' BtaTofApi library. ###

## Summary ##

This driver allows to configure your system and ROS to use all Bluetechnix Time of Flight cameras supported by the BltToFApi. It includes an example allowing you to visualize depth data using the rviz viewer included in ROS. It shows you how to use the ToF devices together with ROS and how you can modify operating parameters. Besides the ToF data, we have included a nodelet to capture rgb video from those devices that include a 2D sensor, such as the Argos 3D P320 or Sentis TOF P510. 

To read the docummentation of bta_ros, please visit the wiki page on ROS: http://wiki.ros.org/bta_tof_driver

## Setup ##

Run `install.sh` to download BtaTofAPI from [Bluetechnix_ToF_API_v2](https://support.bluetechnix.at/wiki/Bluetechnix_ToF_API_v2)