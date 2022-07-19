# ros2_control_odri
----------------------

Author : Paul Rouanet

### What it is

A personal version of odri_control_interface demos adapt to Odri ==> /test/.

And an adaptation of Olivier Stasse's codes : rrbot_system_quadruped.hpp and .cpp in rrbot_system_odri.hpp and .cpp (in /include/ and /src/).

system_odri_multi_interface has not been modified by Paul Rouanet

----------------------

### In system_odri :

Inclusion of all odri_control_interface header needed to use Odri.

Use of the namespace ros2_control_odri

Define of a structure used for IMU : GyroAccLineEulerQuater (must be optimized (x,y,z))


Functions :
   - Definition of init_robot: This function use ODRI methods to initialize the robot :
      - Call Ethernet output
      - Define main_board_ptr_ as theMasterboard with the Ethernet name
      - Define all the joints_ with properties
      - Define the IMU
      - Finally define the robot_ with those 3 elements

   - calibration() : use to calibrate the robot. Currently called in start function. **Maybe a bug to fix line 532 :  robot_->RunCalibration(calib_ctrl); {namespace ?}

   - start() : Start the robot, set some default values to 0, do the calibration, read sensors data

   - stop() : stop the Masterboard, so stop the robot

   - read() : sed all the sensors data to the computer

   - write() : allow the user to send commands to all actuators


----------------------

### Tests :

The compilation with ```colcon build --packages-select ros2_hardware_interface_odri``` is ok, but no way to test the good behaviour of those 5 new functions.

What has been done :

```
colcon build --packages-select ros2_hardware_interface_odri
source ./install/setup.bash
ros2 launch ros2_control_odri_bringup system_odri.launch.py
```

Bugs did not allow us to do test.
