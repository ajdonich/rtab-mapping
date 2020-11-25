## Project: Adaptive Monte Carlo Localization 

This project applies the ROS [AMCL](http://wiki.ros.org/amcl) package to a stripped down version of [Spherebot](https://github.com/ajdonich/spherebot) to perform robot localization in a similated Gazebo world. It does not contain any custom coded nodes, but rather employs a number of existing ROS packages to enable basic visualization and testing, in particular [RViz](http://wiki.ros.org/rviz) is used to visualize Spherebot, an AMCL particle cloud, and a 2D environment map, and it facilitates navigation commands to be issued through the [move_base](http://wiki.ros.org/move_base) package.

![localization](https://github.com/ajdonich/amc-localization/blob/main/localization.jpg)

___

### Installation and Build:

Note: the project requires both [ROS](http://wiki.ros.org/ROS/Installation) and [Gazebo](http://gazebosim.org/) to be appropriately installed (including PATH access to respective binaries) on the client machine. The project was developed on a Ubuntu system with Gazebo v7.16.0 and a ROS Kinetic Kame distro (neither of which are the most recent, but they're what Udacity currently provides). To install and build:

```
$ git clone https://github.com/ajdonich/amc-localization.git
$ cd amc-localization/catkin_ws
$ catkin_make
```

This build will generate two directories in *catkin_ws*: *build* and *devel*. Before executing ROS commands to launch the simulation from a terminal, you will need to source the ros/catkin setup file (in each terminal window you use, and you will need at least two):
```
$ source devel/setup.bash
```
**Note:** execution of commands described below from a terminal that does not have this environment sourced will likely produce cryptic errors, and because ROS setup is required in every new terminal, you may prefer to automate this step by adding something like the following to your *.bash_profile* (or similar):

``` bash
ROS_SETUP=/home/workspace/amc-localization/catkin_ws/devel/setup.bash
if test -f "$ROS_SETUP"; then
    echo "Catkin workspace found, sourcing setup file:"
    echo "${ROS_SETUP}"
    cd /home/workspace/amc-localization/catkin_ws
    source devel/setup.bash
    echo ""
fi
```

___

### Execution:

To run the simulation, start two terminal windows and execute the following two commands in them respectively. The first launches Gazebo, ROS Core and Spherebot and should complete with the Gazebo client GUI displaying Spherebot within his world. The second launches the Map Server, AMCL, Move Base, RViz and RQT Nodes and should complete with the RViz client GUI displaying Spherebot surrounded by a particle cloud on the environment map. 

```
$ roslaunch spherebot spherebot_spawn.launch
```
```
$ roslaunch spherebot amcl.launch
```

Next, in the RViz GUI select the **2D Nav Goal** button in the top menu bar, then click anywhere within the map to place a navigation goal marker. Spherebot should respond by driving to the marker, approximately in alignment with a green trajectory path displayed in RViz. The localization particle cloud should condense more tightly around Spherebot when he is in motion. Feel free to repeat this marker placement as you like (you must reselect the **2D Nav Goal** button each time).  

Finally, you are free to explore a variety of parameter tuning. Though I circled around to using mostly AMCL defaults, the [RQT Rconfigure](http://wiki.ros.org/rqt_reconfigure) tool was very helpful to tune configurations at runtime, which you can run with simply:
```
$ rosrun rqt_reconfigure rqt_reconfigure
```

 **Trouble Shooting:** Collisions are easy to induce that either displace obstacles from their originally mapped locations, or topple Spherebot entirely. Generally, restarting the whole simulation is the only way to fully recover (but see (3) below). A few specific scenarios:

 1. Though the global path planning seems reasonably intelligent about discerning a global trajectory, the exection of drive commands coming from Move Base cannot always direct Spherebot with high precision through sharp turns, and object/wall collisions may result. I believe this is due, at least in part, to lack of precise responsiveness from the [Skid Steer Drive Controller](http://wiki.ros.org/steer_drive_controller) to simultaneous linear and angular velecity commands.
 
 2. For tricky marker placements (e.g. hidden around corners), despite the display of global path trajectories that might work, the drive commands divert Spherebot from that smarter global trajectory toward a more direct route to the marker (that is typically blocked by a wall and results in collision). 
 
 3. You can use the RViz **2D Pose Estimate** button, then click anywhere on the map, which will move/replace Spherebot in Rviz, to confuse (or explore the limits of) the localization algorithm, and AMCL can recover and recalibrate the actual robot position relative to the map if the Pose Estimate is not too divergent. However, if you significantly diverge the estimate from the actual, the particle cloud tends to go crazy and localization is totally lost.  


___  
  
### Implementation Notes:

Spherebot's stripped down design specification can be found in [spherebot.xacro](https://github.com/ajdonich/amc-localization/blob/main/catkin_ws/src/spherebot/urdf/spherebot.xacro) and [spherebot.gazebo](https://github.com/ajdonich/amc-localization/blob/main/catkin_ws/src/spherebot/urdf/spherebot.gazebo). For his more complete implementation, see the [Spherebot](https://github.com/ajdonich/spherebot) repo. The [localize_map.pgm](https://github.com/ajdonich/amc-localization/blob/main/catkin_ws/src/spherebot/maps/localize_map.pgm) environement map was generated directly from the Gazebo [localize_room.world](https://github.com/ajdonich/amc-localization/blob/main/catkin_ws/src/spherebot/worlds/localize_room.world) file using the [pgm_map_creator](https://github.com/udacity/pgm_map_creator) plugin (not included in this repo).





