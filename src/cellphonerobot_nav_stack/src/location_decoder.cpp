#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <vector>
#include <string>
#include <std_msgs/String.h>
#include <iostream>
/*
This script is for registering a action client and send a goal message
to move_base action server when receving a goal location.
*/
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void send_goal(const std_msgs::String::ConstPtr& msg){
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);	
  ROS_INFO("MoveBaseClient set up.");
  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
	
  std::string location_point = msg->data.c_str();
  std::vector<float> location;
  std::string param_name = "move_base/loc/" + location_point;
  if (ros::param::get(param_name, location)) {
	ROS_INFO("There ya go.");
	std::cout << param_name << std::endl;
	move_base_msgs::MoveBaseGoal goal;
	ROS_INFO("MoveBaseGoal created.");

	////we'll send a goal to the robot to move 1 meter forward
	goal.target_pose.header.frame_id = "/map";
	goal.target_pose.header.stamp = ros::Time::now();

	goal.target_pose.pose.position.x = location[0];
	goal.target_pose.pose.position.y = location[1];
	goal.target_pose.pose.position.z = location[2];

	goal.target_pose.pose.orientation.x = location[3];
	goal.target_pose.pose.orientation.y = location[4];
	goal.target_pose.pose.orientation.z = location[5];
	goal.target_pose.pose.orientation.w = location[6];

	ROS_INFO("Sending goal");
	ac.sendGoal(goal); 

	ac.waitForResult();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	  ROS_INFO("Hooray, the base moved 1 meter forward");
	else
      ROS_INFO("The base failed to move forward 1 meter for some reason");
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("location_goal", 100, send_goal);
  ros::spin();
  return 0;
}
