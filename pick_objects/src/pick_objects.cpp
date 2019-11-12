#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/UInt8.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

double pick_up[3] = {4.0, 6.0, 1.0};
double dropoff[3] = {-2.0, -1.0, 1.0};


int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");
  ros::NodeHandle n;

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
  ros::Publisher pos_pub = n.advertise<std_msgs::UInt8>("/robot_position", 1);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = pick_up[0];
  goal.target_pose.pose.position.y = pick_up[1];
  goal.target_pose.pose.orientation.w = pick_up[2];

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
	ROS_INFO("Robot has reached the pick up location");
	ros::Duration(5.0).sleep();

	// Define a position and orientation for the robot to reach goal
  	goal.target_pose.pose.position.x = dropoff[0];
  	goal.target_pose.pose.position.y = dropoff[1];
  	goal.target_pose.pose.orientation.w = dropoff[2];
	
	// Send the goal position and orientation for the robot to reach goal
	ROS_INFO("Sending the goal location");
	ac.sendGoal(goal);

	//Wait an Infinte time for the results
	ac.waitForResult();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
     		{ROS_INFO("Hooray, Robot reached DROP OFF......");
		ros::Duration(5.0).sleep();}
  	else
     		{ROS_INFO("Robot failed to reach Drop off location for some reason");}
    	}	

  else
    ROS_INFO("Robot has failed to move towards Pick up location");

  return 0;
}
