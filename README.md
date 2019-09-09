# ROS Exercises
The following is a collection of small exercises written in C++ within the ROS middleware for a course in Distributed Robotic Systems.

## Building
The catkin software was used to build the programs. Copy the exercises in your catkin source directory and run
```
catkin_make 
```
## Running
To run the exercises, follow the instructions:

### Exercise 1
Exercise 1 uses _RViz_ to visualize the robot pose.
In a terminal, run:
``` 
roslaunch exercise1 exercise1.launch
```
### Exercise 2
Exercise 2 uses _rqt_plot_ and needs parameters for linear and angular speed saturations. 
The launch file contains the linear and angular velocities that can be modified and sent to the custom /control topic:
```
 <node name="topic_pub" pkg="rostopic" type="rostopic" args="pub -r 10 /control exercise2/control &quot; lin_vel: <value> &#13; ang_vel: <value> &quot;" output="screen"/>
```
In a terminal, run:
```
roslaunch exercise2 exercise2.launch vel_lin_min:=<value> vel_lin_max:=<value> vel_ang_min:=<value> vel_ang_max:=<value>
```
### Exercise 3
In a terminal, run:
``` 
roslaunch exercise3 exercise3.launch
```
### Exercise 4
Use the launch file in the folder to modify the initial position of the robots in the turtlesim.
In a terminal, run:
```
roslaunch exercise4 exercise4.launch
```
### Exercise 5
Exercise 5 uses 3 launch files to demonstrate the characteristics of the usage of potentials to get to a goal.
The first two cases show the robot successfully reaching the goal:
```
roslaunch exercise5 exercise5_1.launch
roslaunch exercise5 exercise5_2.launch
```
The third case shows that the robot gets stuck in a local minima:
```
roslaunch exercise5 exercise5_3.launch
```
