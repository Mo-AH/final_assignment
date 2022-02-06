#include <iostream>
#include <chrono>
#include "ros/ros.h"
#include "actionlib_msgs/GoalID.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "move_base_msgs/MoveBaseActionFeedback.h"

// Timeout = 2minutes (goal cancelled after timeout)
#define TIMEOUT 120               

// PUBLISHERS DECLARING
ros::Publisher pub_goal;
ros::Publisher pub_cancel;
ros::Publisher pub_manual;

// MESSAGE TO CANCEL THE GOL
actionlib_msgs::GoalID canc_goal;

bool goal = false;
bool shutdown = false;
        
// GOAL COORDINATES AND ID
float x_goal;                   
float y_goal;                   
std::string id_goal;

// TIME VARIABLES TO MEASURE DURATION OF DRIVING AUTONOMOUSLY
std::chrono::high_resolution_clock::time_point start;  
std::chrono::high_resolution_clock::time_point end;
long int duration;

//MAIN MENU
void inputCommand();

//FUNCTIONS TO REACH GOAL AUTONOMOUSLY
void inputGoal();
void checkGoalStatus(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& feedbackMsg);
void cancelGoal();


int main(int argc, char **argv) {
    
    // NODE INITIALIZATION
    ros::init(argc, argv, "ui_node");
    ros::NodeHandle nh;
    
    // PUBLISHERS DEFINING
    pub_goal = nh.advertise<move_base_msgs::MoveBaseActionGoal>("move_base/goal", 1000);
    pub_cancel = nh.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1000); 

    // SUBSCRIBERS DEFINING
    ros::Subscriber sub_feedback = nh.subscribe("/move_base/feedback", 1000, checkGoalStatus);

    // MULTI-THREADING
    ros::AsyncSpinner spinner(2);
    spinner.start();

    while (!shutdown) 
        inputCommand();

    spinner.stop();
    ros::shutdown();
    ros::waitForShutdown();

    return 0;
}

/*
Display menu and take command from user to switch modalities
*/
void inputCommand()
{
    // Take command from user
    char input;
    std::cout << "\n\n\n MAIN MENU:\n\n";
    std::cout << "\t[1] Set a new goal to be reached autonomosly\n\n";
    std::cout << "\t[2] Drive  robot with the keyboard \n\n";
    std::cout << "\t[3] Drive the robot with keyboard + Collision Avoidance Assistant\n\n";
    std::cout << "----------------------------------------------------------------------\n";
    std::cout << "\t\t\t\t[0] CANCEL GOAL\t [Q] QUIT\n\n";
    std::cin >> input;

    //ignore all characters after first
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    switch(input) 
	    	{

                // INPUT GOAL
                case '1':
	    			inputGoal(); 
	    			break;

                // MANUAL DRIVE MODE
	    		case '2':
                    ros::param::set("collisionAssistant", false);
                    system("rosrun final_assignment manual_node");
                    system("clear");
	    			break;

                // MANUAL DRIVE MODE with CollisionAssistant
	    		case '3':
                    ros::param::set("collisionAssistant", true);
                    system("rosrun final_assignment manual_node");
                    system("clear");
                    break;

                // CANCEL GOAL
	    		case '0':
                    cancelGoal();
                    break;

                //QUIT
                case 'q':
                case 'Q':
                    shutdown = true;

	    		default:
                    system("clear");
	    			std::cout << "\n\n\t ===   COMMAND NOT VALID   ===";
	    			break;
	    	
	    	}
    	}
        
/*
Take goal coordinates from user and publish them and takes the starting time
*/
void inputGoal()
{
	move_base_msgs::MoveBaseActionGoal goal_msg;

	// Take new goal from the user
    std::cout << "\n\n \t NEW GOAL COORDINATES \n\n  Enter X: ";
	std::cin >> x_goal;
    std::cout << "\n  Enter Y: ";
	std::cin >> y_goal;
	
	// Set new coordinates to reach
	goal_msg.goal.target_pose.header.frame_id = "map";
	goal_msg.goal.target_pose.pose.orientation.w = 1;   
	goal_msg.goal.target_pose.pose.position.x = x_goal;
	goal_msg.goal.target_pose.pose.position.y = y_goal;

	// Publish new goal and change state of the robot
	pub_goal.publish(goal_msg);
	goal = true;
    start = std::chrono::high_resolution_clock::now();

}

/*
Callback function of move_base/feedback topic

Update the goal id , check if the goal is been reached and cancel the goal if TIMEOUT has been exceeded
*/
void checkGoalStatus(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& feedbackMsg) {

    // UPDATE GOAL ID
    if (id_goal != feedbackMsg->status.goal_id.id) {
        id_goal = feedbackMsg->status.goal_id.id;
    }

    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::seconds>(end - start).count();
    if(duration > TIMEOUT)
    {
        std::cout << "\n TIMEOUT EXCEEDED : The goal can't be reached! \n\n Enter a new command: \n";
        cancelGoal(); // Cancel the goal if it can't be reached
    } else {
        std::cout << "\t Elapsed time = " << duration << " seconds" << "\r";
    }

    int status = feedbackMsg->status.status;

    if (status != 3 && status != 4)
        return;

    system("clear");

    if (status == 3) //SUCCEDED
        std::cout << "\t\t===   GOAL REACHED    ===";

    else            //ABORTED
        std::cout << "\t\t===   THE GOAL CANNOT BE REACHED   ===";

    cancelGoal();
}

/*
Cancel the goal by publishing in the move_base/cancel topic
*/
void cancelGoal()
{
	if(goal) {
        canc_goal.id = id_goal;
        pub_cancel.publish(canc_goal);
        goal = false;
        std::cout << "\n\n\t===   GOAL CANCELLED   ===\n";
        return;
    }
    std::cout << "\n\n\t===   NO GOAL TO CANCEL   ===\n";
}
