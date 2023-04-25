# Llanes_MECC2023
The code that accompanies the MECC 2023 submission titled "Experimental Validation on Aerial Vehicles of Real-Time Motion Planning with Continuous-Time Q-Learning".


This code can be run as a simulation or on hardware through the uam_vehicle_interface package. 

To run as a simulation through software-in-the-loop (SITL):
1. Install and run PX4 SITL (https://docs.px4.io/main/en/simulation/) with Gazebo classic. For our demonstrations we use the vision model to replace GPS.
```
make px4_sitl gazebo-classic_iris_vision
```
2. Run the Micro XRCE Agent as the bridge between ROS2 and PX4.
```
MicroXRCEAgent udp4 -p 8888
```
3. Run the SITL launch file from the uam_vehicle_interface package.
```
ros2 launch uam_vehicle_interface sitl_launch.py
```
4. RViz2 should open automatically and you should see our custom UAM plugin on the right side panel. Disable the kill switch and command the vehicle to takeoff. Once the vehicle takes off successfully and enters loiter mode then you should be able to command a goal pose through the 2D Goal Pose tool on the top panel.

We also have a vehicle launch file for running our code on hardware from an onboard computer connected to a flight controller running PX4.
```
ros2 launch uam_vehicle_interface vehicle_launch.py
```
