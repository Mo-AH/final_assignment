/**
* \file ui.cpp
* \brief User interface controller of the robot
* \author Mohammad Al Horany
* \version 1.0
* \date 10/04/2022

* \param [out] assistant Defines if the collision avoidance is on.
*
* \details
*
* Subscribes to: <BR>
* -  /move_base/feedback It contains the feedback of the move_base action
*
* Publishes to: <BR>
*  - /move_base/goal It permits to set a goal via move_base action
*  - /move_base/cancel It permits to cancel a goal via move_base action 
*
* Description : <BR>
*
* This is the main node of the program. It provides a main menu to the user, who can choose the modality among the three mentioned.
* It is executed by the launch file and it is always active.
* It also implements the automatic mode (reach the goal autonomously), by simply taking the goal coordinate from the user and publishing it in move_base/goal topic.
* It receives feedback and it checks the status from the action.
*
*/

#include <iostream>
#include <chrono>
#include "ros/ros.h"
#include "actionlib_msgs/GoalID.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "move_base_msgs/MoveBaseActionFeedback.h"

#define TIMEOUT 120 ///< Timeout = 2minutes (goal cancelled after timeout)            

// PUBLISHERS DECLARING
ros::Publisher pub_goal; ///< Publisher to send the goal.
ros::Publisher pub_cancel; ///< Publisher to cancel the goal.

// MESSAGE TO CANCEL THE GOL
actionlib_msgs::GoalID canc_goal; ///< Action message to cancel the goal.

bool goal = false; ///< Boolean defining if there is a goal active.
bool shutdown = false; ///< Boolean defining if user wants to exit.
        
// GOAL COORDINATES AND ID
float x_goal; ///< Coordinate X of the goal.        
float y_goal; ///< Coordinate Y of the goal.                
std::string id_goal; ///< Goal ID.

// TIME VARIABLES TO MEASURE DURATION OF DRIVING AUTONOMOUSLY
std::chrono::high_resolution_clock::time_point start; ///< Starting time of goal reaching. 
std::chrono::high_resolution_clock::time_point end; ///< Ending time of goal reaching.
long int duration; ///< Duration time of goal reaching.

//MAIN MENU
void inputCommand();

//FUNCTIONS TO REACH GOAL AUTONOMOUSLY
void inputGoal();
void checkGoalStatus(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& feedbackMsg);
void cancelGoal();

/**
* \brief Main function of the UI node
* 
* This is the main function of the user interface node. It initializes the node, defines publishers (to set and cancel a goal) and a subscriber (to get the action feedback). 
* After that, it repeatdly calls the 'inputCommand' function until user wants to exit, so that it shutdown the node.
*/
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


/**
* \brief Main menu to input commands
* 
* This function displays the main menu and takes commands from the user to choose the modality or to quit the program.
* When in autonomously driving mode, it provides a command to cancel the goal.
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
        

/**
* \brief Function to input the goal coordinates
* 
* This function is called when the user choose autonomously driving mode. It takes the goal coordinates from him and starts the action,
* changing the state of the robot and starting the timer.
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


/**
* \brief Callback function of /move_base/feedback
* \param feedbackMsg Action move_base feedback.
*
* This function updates the goal ID and checks if the goal has been reached or it cannot be reached, displaying a message in those cases.
* If the timeout has been exceeded, it cancels the goal.
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

/**
* \brief Function to cancel the goal
*
* This function is called to cancel a goal, by publishing a message with the goal ID in '/move_base/cancel'.
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
