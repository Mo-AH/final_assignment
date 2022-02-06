
# Third and Final Assignment of Research Track 1 - Robotics Engineering - UniGE

In this assignment, a robot moves in an environment unknown to it. A laser scan is mounted on the robot in order to move in the environment: when it explored all the maped, it has even a complete reconstruction of it.

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

![image](https://user-images.githubusercontent.com/91679281/152662701-0632bd93-18f9-4366-ac78-e96088d28c25.png)


As said, it doesn't know the map at the beginning, but thanks to the laser scan it can recognize it.
This is his initial view and knowledge of the map (RViz view):


![image](https://user-images.githubusercontent.com/91679281/152662754-78015187-4c33-40ca-b9c6-f1aba4ad9ff0.png)



-----------------------
## Program structure

To do the assignment, I created 2 nodes:
* `ui_node`: provides the main menu to user and implements the mechanism to reach autonomously a goal.
* `manual_node`: enables manual driving and avoid obstacles with the assistant, if the parameter is enabled.

The user can manual drive the robot via `teleop_twist_keyboard` console, which normally publishes the velocities on `/cmd_vel` topic.
This topic is remapped in the launch file to `/check_vel` topic, which is received by `manual_node` (if active).

I also set a boolean parameter `collisionAssistant`, which is also checked by the `manual_node` to know if it has to assist the user to avoid obstacles.


-----------------------
### ui_node

This is the main node of the program. It provides a main menu to the user, who can choose the modality among the three mentioned. It is executed by the launch file and it is always active.

It also implements the first modality, to reach the goal autonomously, by simply taking the goal coordinate from the user and publishing it in `move_base/goal` topic.
It receives feedback and it checks the status from the action.

![image](https://user-images.githubusercontent.com/91679281/152662919-dddf3ef8-0887-48bb-aaa2-bb66e3be3e82.png)


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

This is the node that allows the user to control the robot with the keyboard, via `teleop_twist_keyboard`, runned in another console.

In the standard manual mode, it simply publishes velocities as received, so the user can even go straight to an obstacle.

In the manual mode with collision assistant, it receives velocities and laser scan ranges, which are checked to see if the user is going in a direction where there are obstacles more near than a certain threshold (0.8).
In this case, it set the velocity (linear/angular) pointing to the obstacle to 0.

That's his pseudocode, where 2 parts run simoultanously:

```
get parameter collisionAssistant

== PART 1
if user enters a command, switch off the node and go back to the `ui_node`


== PART 2
If collisionAssistant
  receive velocity and laser scan ranges
  check obstacles in the ranges and modify velocity if required
  publish velocity
else
  publish velocities as received

```

## Possible improvements

The program could be improved in the following ways:
* It would have been better to do a separate node even for the first mode (automatic), in order to mantain a modular program
* When the robot has a complete knowledge of the map, it should be able to know if a certain coordinate is reacheable before trying
