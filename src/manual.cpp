#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

// Threshold distance from the wall to not be exceeded when in collision avoidance mode
#define WALL_TH 0.8           

// PUBLISHER DECLARING
ros::Publisher pub_vel;

//Variables for assistant mode
bool assistant = false;
geometry_msgs::Twist myVel;

//FUNCTIONS FOR MANUAL DRIVING
void manualDrive();
void checkVel(const geometry_msgs::Twist::ConstPtr& msg);
void drivingAssistant(const sensor_msgs::LaserScan::ConstPtr &laserMsg);

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

/*
This function is called when the user choose the manual driving.
It waits for an input to exit and shutdown the node
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

/*
Callback function of /check_vel topic

This function is called whenever the teleop twist keyboard node tries to to send a new velocity.

- MANUAL MODE -> send velocity as received

- MANUAL MODE WITH COLLISION ASSISTANT -> save velocity in global variable

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

/*
Callback function of /scan topic (LaserScan)

If the driving assistence is enabled:
	- stops the robot motion along a certain direction if there is a wall near
        in that direction
Else
    - ignores 
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