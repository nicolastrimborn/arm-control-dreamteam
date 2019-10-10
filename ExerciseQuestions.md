## Explain how Elfin controller computed_torque_controller.cpp implements joint space inverse dynamics controller
Lines 266-279 correspond to lecture slide 30 in RobotMotionControl

## CONTROLLER INTERFACE USED
http://wiki.ros.org/ros_control#Hardware_Interfaces
controller_interface::Controller<hardware_interface::EffortJointInterface>
- bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n)
- void commandCB(const std_msgs::Float64MultiArrayConstPtr &msg)
    -   Publish commands as Float64MultiArray to /elfin/<controller>/command
- void starting(const ros::Time &time)
- void update(const ros::Time &time, const ros::Duration &period)
- void stopping(const ros::Time &time)

## TRANSMISSIONS
https://wiki.ros.org/urdf/XML/Transmission
*The transmission element is an extension to the URDF robot description model that is used to describe the relationship between an actuator and a joint.

### hardware_interface::JointStateInterface to expose the joint state to controllers. 

#### joint (one or more occurrences)
A joint the transmission is connected to. The joint is specified by its name attribute, and the following sub-elements:
##### hardwareInterface (one or more occurrences)
Specifies a supported joint-space hardware interface. Note that the value of this tag should be EffortJointInterface when this transmission is loaded in Gazebo and hardware_interface/EffortJointInterface when this transmission is loaded in RobotHW

## KDL package
http://docs.ros.org/indigo/api/orocos_kdl/html/classKDL_1_1JntArray.html
Contains all the classes for inverse dynamic/kinematic solving 
- ChainDynParam: 
Implementation of a method to calculate the matrices H (inertia),C(coriolis) and G(gravitation) for the calculation torques out of the pose and derivatives. (inverse dynamics)
- Tree:
This class encapsulates a tree kinematic interconnection structure. It is build out of segments. 
- Chain:
This class encapsulates a serial kinematic interconnection structure. It is build out of segments.

## URDF package
http://wiki.ros.org/urdf
Contains a C++ parser for the Unified Robot Description Format (URDF), which is an XML format for representing a robot model.

## To Run
Launch the controller
$ roslaunch elfin_launch gravityPD_controller.launch
Define desired joints in data
$ rostopic pub -r 1000 /elfin/gravityPD_controller/command std_msgs/Float64MultiArray """data: [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]"""

## Reza Notes
The Jacobian Calculation is not in the correct place. Correct it when moving to task space control
