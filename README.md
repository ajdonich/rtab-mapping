## Project: SphereBot 

This project implements a small robot: Spherebot, designed to run in a Gazebo simulator using the Robot Operating System (ROS). It is an extension of the Udacity nanodegree go-chase-it project. It employs a catkin package structure and incorporates several Gazebo plugins including a skid steer drive controller, ROS joint control, and both camera and lidar controllers. Spherebot is currently programmed to search his environment for a white ball, approach/pursue it until it is within close range, then attempt to pick up the ball using his robot arm. He then finds a box/bucket marked with a red circular sign, approaches it while carrying the ball, and finally drops the ball into the bucket.

![Spherebot](https://github.com/ajdonich/spherebot/blob/main/spherebot.jpg)

___

### Installation and Build:

Note: the project requires both [ROS](http://wiki.ros.org/ROS/Installation) and [Gazebo](http://gazebosim.org/) to be appropriately installed (including PATH access to respective binaries) on the client machine. The project was developed on a Ubuntu system with Gazebo v7.16.0 and a ROS Kinetic Kame distro (neither of which are the most recent, but they're what Udacity currently provides). To install and build:

```
$ git clone https://github.com/ajdonich/spherebot.git
$ cd spherebot/catkin_ws
$ catkin_make
```

This build will generate two directories in *catkin_ws*: *build* and *devel*. Before executing ROS commands to launch the simulation from a terminal, you will need to source the ros/catkin setup file (in each terminal window you use, and you will need at least two):
```
$ source devel/setup.bash
```
**Note:** execution of commands described below from a terminal that does not have this environment sourced will likely produce cryptic errors, and because it is required in every new terminal, you may prefer to automate this step by adding something like the following to your *.bash_profile* (or similar):

``` bash
ROS_SETUP=/home/workspace/spherebot/catkin_ws/devel/setup.bash
if test -f "$ROS_SETUP"; then
    echo "Catkin workspace found, sourcing setup file:"
    echo "${ROS_SETUP}"
    cd /home/workspace/spherebot/catkin_ws
    source devel/setup.bash
    echo ""
fi
```

___

### Execution:

To begin the simulation, start a terminal window and launch the Spherebot Node. This command execution should take about 20sec to complete and finish with the Gazebo client GUI displaying the Spherebot within his world.

```
$ roslaunch spherebot spherebot_spawn.launch
```
 **Trouble Shooting:** Occasionally the Gazebo client GUI crashes on launch (I suspect it may relate to insufficient memory as several launching joint controllers compete with the GUI for memory on start-up), you will simply need to start another terminal and launch the Gazebo GUI by hand with:
```
$ gzclient
```
Finally, in a second terminal, launch the Ball Chaser Node with the following. This will kick off the articulation of Spherebot through his simulation sequence, which takes about 7-8min to complete successfully using the default test scenario. Note, however, that it plays out somewhat non-deterministically depending on subtleties of the Gazebo physics simulation. 

```
$ roslaunch ball_chaser ball_chaser.launch
```

This command also launches an RQT GUI displaying the images produced from Spherebot's camera (i.e. his POV). Additionally, a variety of robot state information is logged to the terminal, including camera and lidar values read from sensors, joint commands published, and ascertained joint states.

 **Trouble Shooting:** (1) Ball pickup is the biggest challenge for Spherebot; sometimes he tosses the ball rather than pulling it in. In this case, it is likely most efficient to start over and try again from scratch because he is not currently programmed to handle (or even detect) this failure intelligently. Nevertheless, if you give him another shot, he will likely succeed (in final tests he went 8 for 10). (2) Occasionally the RQT GUI crashes on launch. This does not actually disrupt the simulation for the Spherebot, but will prevent you from seeing his POV. To correct this, you can simply start another terminal and launch the RQT GUI by hand with:
 ```
$ rosrun rqt_image_view rqt_image_view
```  

___  
  
### Implementation Notes:

Spherebot's design specification can be found in [spherebot.xacro](https://github.com/ajdonich/spherebot/blob/main/catkin_ws/src/spherebot/urdf/spherebot.xacro), [robotarm.xacro](https://github.com/ajdonich/spherebot/blob/main/catkin_ws/src/spherebot/urdf/robotarm.xacro), and [spherebot.gazebo](https://github.com/ajdonich/spherebot/blob/main/catkin_ws/src/spherebot/urdf/spherebot.gazebo). A best effort was made to generate realistic moments of inertia, which were calculated with the assumption of real material densities (primarily aluminum  and steel) and accurate volumes. Lighter weight material had to be abandoned due to major robot instability produced by small inertia values.  

Additional joint control configuration (particularly PID values) can be found in [joint_controllers.yaml](https://github.com/ajdonich/spherebot/blob/main/catkin_ws/src/spherebot/config/joint_controllers.yaml). Another significant challenge was trying to tune joint controller parameters relative to moments of inertia to manage both robot stability and movement control of joints through delicate maneuvers such as ball pickup. Some of these parameters include friction, velocity, effort limits, PID, and kp/kd values.

The majority of Spherebot's articulation logic can be found in [spherebot_cns.cpp](https://github.com/ajdonich/spherebot/blob/main/catkin_ws/src/ball_chaser/src/spherebot_cns.cpp). In general, this process runs two threads: one to receive incoming state information from camera, lidar, and joint state publishers, and a second thread to manage execution of a robot state machine and send out various articulation commands relative to that state. Adding this additional thread significantly improved Spherebot's ability to operate accurately and efficiently interleave efferent motor commands with afferent sensory feedback.




