


# MovoInterface

## Requirements
- Ubuntu 18.04 or higher
- ROS catkin or higher with the following ROS packages provided by Kinova: 	
	- movo_msgs
- OPPT v0.5 or higher    

## Installation:

Clone and build the MovoInterface package:

    git clone https://github.com/hoergems/MovoInterface.git
    cd MovoInterface && mkdir build && cd build
    cmake -DCMAKE_INSTALL_PREFIX=<install folder> ..
    make && make install
    
Next, navigate to your catkin workspace and simlink the ROS packages inside the ``<MovoInterface>/ROS/`` to your catkin workspace:

	cd <catkin_worspace>/src
	ln -s <MovoInterface>/ROS/movo_robotiq_gripper .
	ln -s <MovoInterface>/ROS/movo_ros .
	
Finally, build your catkin workspace:

	cd <catkin_workspace>
	catkin_make 

## Hardware setup
First you need to make sure that you're connected to the robot (via Ethernet) and that you can properly communicate with the arm. To do this, connect your laptop via Ethernet. In the network settings of your laptop, use the default settings (i.e. the IPv4 Method should be set to "Automatic (DHCP)"). To test whether you can communicate with the arms, open a terminal and type
    
    ping 10.66.171.16

for the left arm and

    ping 10.66.171.15

Next, figure out your local ip by opening a terminal and type

    ifconfig

Note the local ip of your network adapter and open the 

    <MovoInterface>/cfg/MovoInterface.cfg
file. Under the [movoOptions] sections, the the localIP parameter to the local ip of your network adapter.

## Preparing and running the MovoInterface
On the MOVO2 computer, open a terminal and run

    movostop

Then run

    roslaunch movo_bringup_simple main_mobile_base.launch

This will launch the neccessary movo packages on Movo2 (excluding the MoveIt stack).

On your computer, open a terminal and source your catkin workspace:

	source <catkin_workspace>/devel/setup.bash

In the same terminal, navigate to the ``<MovoInterface>/script`` folder and execute the ``launchRobotiqGripper.sh`` script:

	cd <MovoInterface>/script
	./launchRobotiqGripper

This launches the ROS stack to control the RobotiQ gripper.
Next, open a new terminal on your computer and execute

    source <install folder>/share/oppt/setup.sh

or add this line to your .bashrc file. \<install folder\> is the installation folder specified in the cmake command above. In the same terminal you can then run the example problem with the provided config file, e.g.

    cd <oppt folder>/bin
    ./abt --cfg <MovoInterface>/cfg/MovoInterface.cfg
## Components

### MovoRobotInterface
The MovoInterface in ```<MovoInterface>/plugins/shared/MovoRobotInterface/MovoRobotInterface.hpp``` is a lightweight wrapper around the Kinova API to control the arm directly without going through Kinova's MoveIt setup. An example of how the MovoInterface is being used is in the 
    
    <MovoInterface>/plugins/transitionPlugins/MovoTransitionPlugin.cpp

file. The MovoRobotInterface provides simple methods to control the joint angles and joint velocities of the arm via the

    sendTargetJointAngles

and 

    applyJointVelocities

methods. The

    moveToInitialState(const VectorFloat &initialJointAngles)
method computes a trajectory of the arm to move from its current configuration to the joint angles provided by ```initialJointAngles```. It does so by solving a motion planning problem via RRTConnect that takes self-collisions into account. Note that ```initialJointAngles``` must be a 7D-vector.

Furthermore, the 

    openGripper()

and 

    closeGripper()

methods only work when using the KG3 gripper provided by Kinova. The RobotiQ gripper is controlled via the RobotiQInterface below. 

## MovoMobileBase
The MovoMobileBase in ```<MovoInterface>/plugins/shared/MovoRobotInterface/MovoMobileBase.hpp``` is a small interface to control the mobile base. The 

    moveToPose(const oppt::geometric::Pose &pose)

method can be used to move the mobile base to the given pose. The ```pose``` parameter herby specifies the target pose relative to the initial pose of the mobile base. After starting the robot, this initial pose is set to ```(0, 0, 0, 0, 0, 0)```. Note that as of now, only the ```xy```- components of the pose are considered (i.e. the mobile base moves along the xy-plane), whereas the orientation component of the pose is ignored. 

## RobotiqInterface
The RobotiqInterface in ```<MovoInterface>/plugins/shared/MovoRobotInterface/MovoRobotInterface.hpp```
controls the RobotiQ gripper mounted on the left arm. Currently this interface provides two methods
    
    openGripper()

and 

    closeGripper()

that fully open and close the gripper respectively, and a 

    isMoving()

method to determine wheter the fingers are currently moving.

