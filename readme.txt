COMPSYS726: Assignment 1 Solution
==================================

To run the solution
1. extract the assignment1 folder into [catkin_ws]/src/, where [catkin_ws] is the directory of a preexisting catkin workspace. If you do not have a catkin workspace then make one by making directory with a src/ folder
2. In the root of the catkin workspace, [catkin_ws]/, run the command 
	$ catkin_make
3. Set up the source by running 
	$ source [catkin_ws]/devel/setup.bash
4. Launch the turtlebot gazebo world by running
	$ roslaunch turtlebot_gazebo turtlebot_world.launch 
you can add world_file:=$PWD/worlds/[filename] at the end to launch where [filename] is a .world file to launch a specific world in the working directory
5. Launch the assignment1 launch file by running 
	$ roslaunch assignment1 assignment.launch
to run the solution
