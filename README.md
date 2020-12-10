# ROSRagnarEDU
ROS interface for Ragnar EDU

## Calculation of IK and FK

Describe how the inverse and forward kinemativ of Delta Robot is computed [Doku](https://github.com/SiHaoShen/ROSDeltaRobot/blob/main//Kinematic_Calculation_IK_FK_DeltaRobot.pdf)
  
## Prepare the Environment

  1. Install Git [Doku](https://github.com/SiHaoShen/ROSDeltaRobot/blob/main//HowToSetupDeveloperPC_18-04.pdf)
  2. Install ROS Melodic [Doku](https://github.com/SiHaoShen/ROSDeltaRobot/blob/main//HowToSetupDeveloperPC_18-04.pdf)
  3. Install Eclipse [Doku](https://github.com/SiHaoShen/ROSDeltaRobot/blob/main//HowToSetupDeveloperPC_18-04.pdf)
  
## Dependencies

ROS Ragnar depends on the following core packages:

  1. [industrial_core](https://github.com/ros-industrial/industrial_core)
  2. rviz
  3. Simple Message
  4. Industrial msgs 
  5. Industrial utils 
  6. Industrial trajectory filters
  7. Industrial robot simulator

**Please note that all terminals need to be sourced in your current workspace**

**Please note that you will need the most recent versions of the debians for 'robot_state_publisher' as of 24 Nov, 2015. Please 'apt-get update' if you have not**

**Please note that you will need an RViz version > 1.11.8**

## Simulation
If you want to simulate the robot, you need to open 5 command line windows. Each of the window commits:
```
roscore
source ~[Catkin_Workspace]/devel/setup.bash
roslaunch ragnar_support simulate.launch
```

This simulation listens to the same `joint_path_command` topic that the driver does
and can be commanded to move in the same way.

## General Bringup
If you want to bringup the robot state listener, the command interface, and visualization:
```
roslaunch ragnar_support bringup.launch [robot_ip:=xxx.xxx.xxx.xxx] [state_port:=xxxx]
```

The ip defaults to `192.168.1.240` and the state port defaults to `11002`.

## Feedback and Visualization
To launch a node that will listen to the robot feedback:
```
roslaunch ragnar_drivers ragnar_state_listener.launch [robot_ip:=xxx.xxx.xxx.xxx]
```

Note that the ip for this node defaults to `192.168.1.240`

## Command Interface
To launch a node that enables commanding the robot to follow trajectories:
```
roslaunch ragnar_drivers ragnar_streaming_interface.launch robot_ip:=xxx.xxx.xxx.xxx
```

Note that the robot listens on the ```joint_path_command``` topic with type ```trajectory_msgs::JointTrajectory```

## Demonstrations
After these other components have been launched, you may run a test script with:
```
rosrun ragnar_kinematics ragnar_demo_motions
```

## Outstanding Issues

This package is still in development, and requires a few components to be workable:

 1. The robot needs a ROS action server interface so that client programs can be aware of the state of
    robot motion
 2. The robot needs safety checks. There may be mechanical limits that dissallow certain joint configurations. If these are not caught by the kinematics routines or by a simple bounding box check on the work volume, they may damage the robot.
 3. The robot now plays nicely with other geometry in URDF files, but there are still some outstanding issues with JointState messages that come from other scene geometry. Ragnar state publisher assumes the first four joints that come in are meant for it. To fix this, and a few other issues, we need to identify standard names.

