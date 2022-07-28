# ros2_hardware_interface_odri
----------------------

## Introduction

This project provides a ros2_control SystemInterface to the master board ODRI.
The particularity of this board is to provide a command interface in position, velocity, torque and gains (Kp, Kd).

This package can be used together with a description package in order to provide the mapping between the robot models
and the actuator. A robot_config.yaml file is needed.

## Installing from source:

```
colcon build --packages-select ros2_hardware_interface_odri
source ./install/setup.bash
ros2 launch ros2_control_odri_bringup system_odri.launch.py
```

## How to use it ?

### ros2_control

In your xacro file (used to generate the urdf file) you can add the following ```hardware``` block in your ros2_control system:
```
<ros2_control name="bolt" type="system">
  <hardware>
    <plugin>ros2_control_odri/SystemOdriHardware</plugin>
       <xacro:property name="prop_bolt_config_yaml" value="$(find ros2_description_bolt)/config/bolt_config.yaml" />
      	    <param name="odri_config_yaml">${prop_bolt_config_yaml}</param>
    	    <param name="desired_starting_position">0.0 0.0 0.0 0.0 0.0 0.0</param>
    	    <param name="default_joint_cmd">FL_HAA 0.0 0.0 0.0 3.0 0.05
    	    	   FR_HAA 0.0 0.0 0.0 3.0 0.05
	   	   FL_KFE 0.0 0.0 0.0 3.0 0.05
		   FL_HFE 0.0 0.0 0.0 3.0 0.05
		   FR_KFE 0.0 0.0 0.0 3.0 0.05
		   FR_HFE 0.0 0.0 0.0 3.0 0.05</param>
	    <param name="default_state_cmd">FL_HAA 0.0 0.0 0.0 3.0 0.05
	    	   FR_HAA 0.0 0.0 0.0 3.0 0.05
		   FL_KFE 0.0 0.0 0.0 3.0 0.05
		   FL_HFE 0.0 0.0 0.0 3.0 0.05
		   FR_KFE 0.0 0.0 0.0 3.0 0.05
		   FR_HFE 0.0 0.0 0.0 3.0 0.05</param>

  </hardware>
  ...
  ...
</ros2_control>
```

The field ```desired_starting_position``` should have the same size than the number of joints.
Here the robot Bolt has 6 joints set to zero.
The field ```desired_joint_cmd``` should provide for each joint a list of ```joint_name position velocity effort Kp Kd```.
They are the first values send to the master ODRI board.
Here the 6 joint commands of the Bolt robot are set to position: 0.0, velocity: 0.0, effort: 0.0, Kp: 3.0, Kd: 0.05

The field ```desired_state_cmd``` should provide for each joint a list of ```joint_name position velocity effort Kp Kd```.
They are the first values that ros2_control will provides before reading the master ODRI board.
Here the 6 joint states of the Bolt robot are set to position: 0.0, velocity: 0.0, effort: 0.0, Kp: 3.0, Kd: 0.05

### In system_odri :

Inclusion of the odri_control_interface header needed to use Odri.


   - start() : Start the robot, set some default values to 0, do the calibration, read sensors data

   - stop() : stop the Masterboard, so stop the robot

   - read() : sed all the sensors data to the computer

   - write() : allow the user to send commands to all actuators


----------------------

### Tests :

The compilation with ```colcon build --packages-select ros2_hardware_interface_odri``` is ok


Tested on Bolt.

# Credits :

 *   Maxime-Ulrich Fansi (04/2022-09/2022) - First working version of gazebo_bolt_ros2_control
 *   Benjamin Amsellem (10/2021-02/2022) - First working version of ros2_hardware_bolt
 *   Paul Rouanet - (03/2021-09/2021) - Building the LAAS Bolt, starting this repo
 *   Olivier Stasse (03/2021 - Supervision)
