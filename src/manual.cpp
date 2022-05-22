/**
* \file manual.cpp
* \brief Manual controller of the robot
* \author Mohammad Al Horany
* \version 1.0
* \date 10/04/2022

* \param [in] assistant Defines if the collision avoidance is on.
*
* \details
*
* Subscribes to: <BR>
*  /check_vel It contains the velocity to be sent to the robot
*  /scan It contains the laser scan ranges
*
* Publishes to: <BR>
*  /cmd_vel It sends the velocity to the robot
*
* Description :
* This is the node that allows the user to control the robot with the keyboard, via 'teleop_twist_keyboard', runned in another console.
* If the collision assistant is off, it simply publishes velocities as received, so the user can even go straight to an obstacle.
* If the collision assistant is on, it receives velocities and laser scan ranges, and checks if the user is going in a direction where there are
* obstacles more near than a certain threshold (0.8).
* In this case, it sets the velocity (linear/angular) pointing to the obstacle to 0.
*
*/

#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

// Threshold distance from the wall to not be exceeded when in collision avoidance mode
#define WALL_TH 0.8 ///< Threshold distance from the wall to not be exceeded when the collision assistan is on.    

// PUBLISHER DECLARING
ros::Publisher pub_vel; ///< Global publisher to send velocities.

//Variables for assistant mode
bool assistant = false; ///< Global boolean defining if the collision assistant is on/off. 
geometry_msgs::Twist myVel; ///< Global velocity message.

//FUNCTIONS FOR MANUAL DRIVING
void manualDrive();
void checkVel(const geometry_msgs::Twist::ConstPtr& msg);
void drivingAssistant(const sensor_msgs::LaserScan::ConstPtr &laserMsg);

/**
* \brief Main function of the manual node
* 
* This is the main function of the manual mode. It initializes the node, subscribes to "/check_vel" and "/scan" topics and defines a publisher on "/cmd_vel".
* After that, it gets the parameter "collisionAssistant" from the parameter server and puts it in a global variable.
* It calls the "manualDrive" function which returns if the user presses a key, so the main function can shutdown the node.
*/
int main(int argc, char **argv) {
    
    // NODE INITIALIZATION
    ros::init(argc, argv, "manual_node");
    ros::NodeHandle nh;
    
    // PUBLISHERS DEFINING
    pub_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    
    // SUBSCRIBERS DEFINING
    ros::Subscriber sub_velocity = nh.subscribe("/check_vel", 1000, checkVel); 
    ros::Subscriber sub_laser = nh.subscribe("/scan", 1000, drivingAssistant);

    //GET THE PARAMETER ACTIVATING COLLISION ASSISTANT
    ros::param::get("collisionAssistant", assistant);

    // MULTI-THREADING
    ros::AsyncSpinner spinner(3);
    spinner.start();

    manualDrive();
    
    spinner.stop();
    ros::shutdown();
    ros::waitForShutdown();

    return 0;
}

/**
* \brief Manual drive function
* 
* This function is called when the user choose the manual driving mode.
* It prints an informational output and waits for an input to exit the manual mode and shutdown the node.
*/
void manualDrive()
{
    
    char input;
    system("clear");
    std::cout <<  "\n\n\t ===         MANUAL DRIVING MODE       ===\n\n";
    if(assistant)
        std::cout << "\t    +++++   Driving assistance enabled +++++    \n";
    else
        std::cout << "\t    -----   Driving assistance disabled -----\n";

    std::cout << "\n\n\n \t  ### --- ENTER ANY KEY TO EXIT MANUAL MODE --- ###\n\n";

    std::cin >> input;
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    //STOPPING THE ROBOT
    myVel.linear.x = 0;
    myVel.angular.z = 0;
    pub_vel.publish(myVel); 
}

/**
* \brief Callback function of /check_vel topic
* \param msg Robot's velocity
*
* This function is called whenever the teleop twist keyboard node tries to send a new velocity:
* - COLLISION ASSISTANT OFF --> it publishes the velocity as received in /cmd_vel topic
* - COLLISION ASSISTANT ON --> it saves the velocity in a global variable
*/
void checkVel(const geometry_msgs::Twist::ConstPtr& msg) {

    if(!assistant)
    {
    	pub_vel.publish(msg);
    	return;
    }

    //take note of velocities that will be checked/modified by drivingAssistant
    myVel.linear.x = msg->linear.x;
    myVel.angular.z = msg->angular.z;
}

/**
* \brief Callback function of /scan topic (LaserScan)
* \param laserMsg Robot's laser scan
*
* This function is called at every message on the /scan topic:
* - COLLISION ASSISTANT OFF --> it ignores the message and returns 
* - COLLISION ASSISTANT ON --> it stops the robot motion along a certain direction if there is a wall near in that direction and publishes the velocity in /cmd_vel topic 
*/
void drivingAssistant(const sensor_msgs::LaserScan::ConstPtr &laserMsg)
{
    //IF Assistant ISN'T ENABLED, IGNORES THE MESSAGE
	if(!assistant) 
		return;
    
    //CHECK NEAREST WALL IN EACH DIRECTION
    int size = laserMsg->ranges.size();
    int sectionSize = size/ 3;
    float laserScan[size];
    for (int i=0; i <size; i++){
        laserScan[i] = laserMsg->ranges[i];
    }

    float nearestWall_Right = 100;
    float nearestWall_Front = 100;
    float nearestWall_Left = 100;

	for(int i = 0; i < sectionSize; i++){
        if (laserScan[i] < nearestWall_Right)
            nearestWall_Right = laserScan[i];
        if (laserScan[i + sectionSize] < nearestWall_Front)
            nearestWall_Front = laserScan[i + sectionSize];
        if (laserScan[i + 2*sectionSize] < nearestWall_Left)
            nearestWall_Left = laserScan[i + 2*sectionSize];
    }

	//BLOCKS THE MOTION IN A CERTAIN DIRECTION (MODIFYING MYVEL) IF THERE IS AN OBSTACLE NEAR IN THAT DIRECTION
	
    //RIGHT
    if(nearestWall_Right < WALL_TH && myVel.angular.z < 0){
        myVel.angular.z = 0;
        std::cout << "===     CAUTION : Obstacle near at right side       ===" << '\r';
	}
    //FRONT
	if(nearestWall_Front < WALL_TH && myVel.angular.z == 0){
        myVel.linear.x = 0;
        std::cout << "===     CAUTION : Obstacle near at front side       ===" << '\r';
	}

    //LEFT
	if(nearestWall_Left < WALL_TH && myVel.angular.z > 0){
    	myVel.angular.z = 0;
        std::cout << "===     CAUTION : Obstacle near at left side       ===" << '\r';
	}

    //PUBLISH THE VELOCITY MODIFIED
    pub_vel.publish(myVel);
}