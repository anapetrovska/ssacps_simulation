# Smart Self-Adaptive Cyber-Physical Systems
## ROS-Based Simulated Multi-Robot Systems
This repository contains resources for the simulation of ROS-Based Multi-Robot Systems. The implementations are based on ROS and Gazebo for simulation. Aside from simply running the use case of the multi-robot cleaning reference problem, they can be modified to any extent for individual purposes.

The use case was run on Ubuntu 18.04 with ROS Melodic, Gazebo(9.0.0) respectively. For simulation two Turtlebot3 Burger are used. Environment and robots can be replaced by other models - some ROS nodes' parameters may have to be adapted accordingly. Switching to another ROS version like Kinetic should work, but was not tested.

If you are familiar with ROS and Gazebo, you may want to skip to the [installation instructions for the required ROS packages](#installing-the-required-packages) or to [the section on how to run the simulated system](#running-the-simulated-system).

# Prerequisites
Before setting up and running the use cases make sure that:  
- __ROS__ is installed and running properly. For the installation of ROS stick to the [official installation instructions](http://wiki.ros.org/melodic/Installation/Ubuntu). Pick the desktop-full or desktop installation. The desktop-full installation should come with Gazebo.
- __Gazebo__ and the required ROS dependencies work.  
   If you installed the desktop-full version of ROS, Gazebo is probably installed as well. If this is not the case, follow the [official documentation](http://gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros).  
   __Important__: We will need some ROS plugins in the Gazebo models, which is why we need the [corresponding ROS package](http://wiki.ros.org/gazebo_plugins): `sudo apt install ros-melodic-gazebo-plugins`.

Now ROS and the simulators should be ready. Before continuing make sure that these components are properly installed (Can you start ROS and the simulator? Does the simulator publish ROS messages for a ROS-based simulation? Are there any warnings or errors?).

# Setting up the system

The simulated system depend on several ROS packages:  

- goal\_provider  
- map\_server  
- turtlebot3  
- ROS navigation stack (Kinetic release)  

The complete list of packages can be found on the following link: https://github.com/tum-i22/ssacps_packages.

Some of the packages are part of meta-packages. The following section guides through the process of creating the workspace for the packages and installing them.

## Installing the required packages

1. If the environment hasn't been sourced yet: `source /opt/ros/melodic/setup.bash`
2. Create a catkin workspace e.g.: `~/catkin_ws` with a src directory `~/catkin_ws/src` (or just run : `mkdir -p ~/catkin_ws/src`)
3. In `~/catkin_ws` run `catkin_make`
4. Run `source devel/setup.bash`
5. Get the necessary packages. Switch to the corresponding directory `cd ~/catkin_ws/src` and get the source code for the packages:
     - Get the Kinetic release of the navigation stack; you can either clone the repository and checkout the Kinetic release with `git clone https://github.com/ros-planning/navigation.git && git checkout kinetic-devel && git checkout tags/1.14.4 -b most-recent-kinetic-release` or you just [download the zipped files](https://github.com/ros-planning/navigation/archive/1.14.4.zip) and unpack them. In both cases we have to make a little change to `amcl/src/amcl_node.cpp` to build for Melodic, the adaption is explained [here](https://github.com/moriarty/navigation/commit/ae060c92a423783c45ef35005ec443e5736c6689).
     - Get the Turtlebot resources: `git clone https://github.com/ROBOTIS-GIT/turtlebot3.git`
     - Clone the the ssacps_packages repository
6. Now we have all the required packages; go back to the root of your catkin workspace: `cd ~/catkin_ws`
7. Get all packages' dependencies: `rosdep install --from-paths src --ignore-src --rosdistro melodic -r -y`
8. Build all our packages: `catkin_make -DCMAKE_BUILD_TYPE=Release`. This might take a few minutes the first time and depending on the system.
9. Et voilà: All packages are available. Keep in mind that you have to source them: `source ~/catkin_ws/devel/setup.bash`. You can list all available ROS packages with the command `rospack list-names`.

Now we can finally proceed to launch the simulations.


# Running the simulated system
We can differ between two steps:  

1. Launching the simulation  
   Gazebo-based simulations can be started from a launch file: `roslaunch simulation_multi_robot.launch`  
2. Launching  the robots' ROS nodes  
   Start everything from the corresponding launch file: `roslaunch multi-robot.launch`

In this repository, go to the directory of the system that you want to launch.

Starting the simulation with the following three steps:  
1. Specify the Turtlebot 3 model once in both terminals: `export TURTLEBOT3_MODEL=burger`  
2. `roslaunch simulation_multi_robot.launch`  
3. `roslaunch multi_robot_launch.launch`  

For more detailed description of how to start the simulation, please refer to the following [link](cleaning_gazebo). 


__IMPORTANT__: You need to source the ROS and catkin setup files (in each terminal window). If you followed the standard ROS configuration, you already added the command (`source /opt/ros/melodic/setup.bash`) to your .bashrc and only have to `source ~/catkin_ws/devel/setup.bash`. 


# System architecture
The following figure wraps up the high-level architecture for the system (including the ROS nodes and topics):

![Overall Architecture](documentation_resources/high_level_architecture.PNG)  

As there are two robots, topics and nodes have to be associated to the namespace of the corresponding robot. Same applies to the transforms. The ROS nodes running for both robots are identical, since they perform the same tasks. The subsequent transform tree and ROS computation graph for the respective use cases depict the detailed structure of the implementations. 
<details><summary>ROS computation graph:</summary>  
 
![ROS Computation Graph](documentation_resources/material_rosgraph.png)  

</details>

As already pointed out, this use case relies on the Kinetic version of the ROS navigation stack, as some issues arise when using a realistic laser frequency (5 Hz) for the Turtlebot in combination with the Melodic version of the navigation stack. With the Kinetic version this issue could not be observed. For more details check out this [question on ROS answers](https://answers.ros.org/question/308814/costmap2d-observation-layers-laser-scan-callback-is-never-called-with-a-low-publication-rate/) and this [issue on Github](https://github.com/ROBOTIS-GIT/turtlebot3/issues/349).
</details>
  
Parameters can be changed in the YAML files and/or directly in the launch files. Furthermore, simulation environment, robots, etc. can be easily changed as well in the corresponding simulation files.

# Troubleshooting
There are various sources for problems or issues that can cause a ROS-based system to run not as intended or even not at all. With numerous parameters, many nodes and interconnections between them it can be quite hard to find the actual soure of the problem. Before debugging there are some basic steps which can be taken to find or at least narrow down the origin of the issue. The following list is a (non-exhaustive) compilation of steps and aspects to consider which may be helpful. Depending on the type of failure they are more or less relevant.

- Are the relevant topics advertised? See `rostopic list` for all topics. Get infos on a particular node with `rosnode info node_name`.
- With [rqt](http://wiki.ros.org/rqt)  (`rqt_graph`) you can visualize the ROS computation graph. Are all nodes up and publishing/subscribed to the correct topics?
- Are the messages actually published to a topic? Just run `rostopic echo /topic_name` to print the actual messages.
- Are the publication frequency for a message and the delays as expected? With `rostopic hz /topic_name` you can get the publication frequency and with `rostopic delay /topic_name` the delay of the messages published to the specified topic.
- By setting `rosparam set enable_statistics true` additional statistics are published - and visualized in the rqt's computation graph.
- If you use a simulation time or suspect that timing is an issue, you can check out the timestamps of the published messages.
- Are the parameter values provided to the ROS parameter server as expected? All available parameters are displayed when running `rosparam list` - also pay attention to typos. The value of a parameter is returned by running `rosparam get /parameter_name`.
- If the transforms seem not to work properly, first check the corresponding [transform tree](http://wiki.ros.org/rqt_tf_tree): `rosrun rqt_tf_tree rqt_tf_tree` (or select the tf_tree plugin in the rqt GUI).
- By running `roswtf` the running ROS system is diagnosed for errors. The [tool](http://wiki.ros.org/roswtf) returns warnings and suspected errors.
- If you need more detailed information on the transforms, the [tf debugging tools](http://wiki.ros.org/tf/Debugging%20tools) can be helpful.
- The visualization of the messages in [rviz](http://wiki.ros.org/rviz) can be an easy way to spot problems concerning the messages' content. For instance, if a map contains falsely marked obstacle or does not allign to the sensor readings (do the sensor readings match the environment? - increase the decay time for a better overall picture). But keep in mind that rviz causes an additional load.
- Not sourcing the (correct) environment can be easily forgot.
- If you record all (or a relevant subset of) messages, you can inspect them in detail with rqt_bag: `rqt_bag recorded_bag.bag`. The messages can be published or played from the recorded bag too: `rosbag play recorded_bag.bag`.
- If you suspect that something is wrong with the simulation, only launch the simulation without the other nodes and inspect if the expected topics are advertised/subscribed and if the content of the messages is correct. Also check if the physical properties of the used models are correct.
- Does your machine present a bottleneck? If your system can barely keep up with the resource demand of the simulator and the ROS nodes, chances are high that you encounter some issues.



# Misc

- All ROS packages that are available as debian package can be simply installed via apt. The naming scheme follows the following pattern:`sudo apt-get install ros-<distro>-<package_name>`, so for Melodic: `sudo apt-get install ros-melodic-<package_name>`
- The [status page of the ROS packages](http://repositories.ros.org/status_page/ros_melodic_default.html) can be very handy to checkout the build status of a particular package or the exact version number.
- Just a single package can be built with `catkin_make -DCMAKE_BUILD_TYPE=Release --pkg [pkg_name]`
- If you start the ROS master manually, don't forget to shut it down after a run; otherwise, parameters from the previous run might interfere with the new one.
- [__rviz__](http://wiki.ros.org/rviz) is a great tool for visualization. You can visualize various message types like sensor readings, (cost)maps, transforms, poses, paths, etc. or publish navigation goals (by right-clicking the button in the GUI you can modify the topic to which the goal is published - for instance, for a particular robot in a multi-robot setup). You can launch rviz with an existing configuration by running `rviz -d some_rviz_file.rviz`.
- If you want to use simulation time (or any time that is provided by some node), you have to set `rosparam set use_sim_time true` and the time published to `/clock` is used instead of wall time.
- For refining the configuration of the navigation stack, the [tutorial on ROS.org](http://wiki.ros.org/navigation/Tutorials/RobotSetup) can be helpful - particularly when switching to a real robot. Some configuration steps are are more relevant for real systems.
- You can save a published map (for instance the one generated by a SLAM algorithm) with the map_saver of the [map_server](http://wiki.ros.org/map_server): `rosrun map_server map_saver map:=/map_topic`. Maps can be easily saved in a fixed interval: `for i in {1..12}; do rosrun map_server map_saver map:=/map_topic -f "map_name_$i" && sleep 5.0; done`.
- Launching a map_server that provides a map (read from a pgm and corresponding yaml file), can be achieved with the following command: `rosrun map_server map_server /COMPLETE_PATH/demo_map.yaml map:=/reference_map/map`
- You can manually publish messages to any topic. For instance, instead of publishing a goal via the rviz GUI you can publish a goal to the goal topic with `rostopic pub --once p3dx0/move_base_simple/goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "p3dx0_tf/map"}, pose: {position: {x: 2.0, y: -20.0, z: 0.0}, orientation: {w: 1.0}}}'` (adapt the topics and frames according to your setup).
- Sometimes Gazebo crashes on startup. This can be caused by a stuck simulation server. In this case running `killall gzserver` helps.


# Testing and benchmark

`rosbag_data` is added for testing and benchmark purposes
