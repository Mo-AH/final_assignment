
# Third and Final Assignment of Research Track 1 - Robotics Engineering - UniGE

In this assignment, a robot moves in an environment unknown to it. A laser scan is mounted on the robot in order to move in the environment: when it explored all the maped, it has even a complete reconstruction of it:

The user can interact with the robot in 3 ways:
*  setting a coordinate [X,Y] of a goal that will autonomously reached by the robot
*  drive manually the robot via keyboard
*  drive manually the robot via keyboard with an assistance to avoid obstacles


-----------------------
## Installing and running

To run this simulator, [__ROS Noetic__](http://wiki.ros.org/noetic/Installation) is required and it should be installed in the machine, following the instructions of the linked page, and a ROS workspace should be created.

The project make use of three additional elements:

* slam gmapping package
* ros navigation stack
* xterm

They can be installed, in order, by following those istructions:

```console
$ git clone https://github.com/CarmineD8/slam_gmapping.git
```

```console
$ sudo apt-get install ros-<your_ros_distro>-navigation
```

```console
$ sudo apt install xterm
```

Done that, you can simply follow these steps:
1. clone this repository in the `src` folder of your ROS workspace.
2. launch `$catkin_make` command from the main directory of the workspace to build the package
3. launch `$roslaunch final_assignment run.launch` command to run all the nodes required


-----------------------
## Environment and mapping
The robot initial position and the environment are the following (Gazebo view):




As said, it doesn't know the map at the begging, but thanks to the laser scan it can recognize it.
This is his initial view and knowledge of the map (RViz view):





-----------------------
## Program structure






-----------------------
### ui_node

That's his pseudocode:

```
SWITCH CHOICE: 
    [1] make the user input a coordinate [x , y]  and send a goal in the topic `move_base/goal` of `move_base` package
        get feedback of the action
              if the timeout, set to 2 minutes, has been reached, cancel the goal and return to the main menu
              check status:
                  if the goal has been reached, cancel goal and return to the main menu
                  if the goal cannot be reached, cancel goal and return to the main menu 

    [2] set `collisionAssistant` to false and run the `manual_node` 
    [3] set `collisionAssistant` to true and run the `manual_node`
    
    [0] if there is a goal running, cancels the goal and stops the robot
        else do nothing
        
    [Q] shutdown the program
```

-----------------------
### manual_node

This is the node that allows the user to control the robot with the keyboard, with the `teleop_twist_keyboard` console.

In the standard manual mode, it simply publishes velocities as received, so the user can even go straight to an obstacle.
In the manual mode with collision assistant, it receives velocities and laser scan ranges, which are checked to see if the user is going in a direction where there are obstacles more near than a certain threshold (0.8).
In this case, it set the velocity (linear/angular) pointing to the obstacle to 0.

That's his pseudocode, where 2 parts run simoultanously:

```
get parameter collisionAssistant

== PART 1
if user enter a command, switch off the node and go back to the `ui_node`


== PART 2
If collisionAssistant
  check obstacles (via laserScan) and velocities as described
else
  publish velocities as received




```

## Possible improvements

The program could be improved in several ways:
* Get more fluid movements of the robot, maybe simulating a real trajectory of a F1 Machine
* Do more controls when changing speed, for example to never go under 0 for both linear and angular speed, to avoid unrealistic behaviour
* Adding a button to change the sense of the lap from counter-clockwise to clockwise
* Adding an int counting the number of laps and a time variable to get the time for every lap
