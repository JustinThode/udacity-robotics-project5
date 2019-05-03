#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

constexpr float pickX = 6.0F;
constexpr float pickY = -8.0F;
constexpr float dropX = 6.0F;
constexpr float dropY = -4.0F;

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  ROS_INFO("pickX %f", pickX);
  ROS_INFO("pickY %f", pickY);
  ROS_INFO("dropX %f", dropX);
  ROS_INFO("dropY %f", dropY);

  std::vector<move_base_msgs::MoveBaseGoal> goals;

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach

  // add first goal
  goal.target_pose.pose.position.x = pickX;
  goal.target_pose.pose.position.y = pickY;
  goal.target_pose.pose.orientation.w = 1.0;
  goals.push_back(goal);
  
  // add second goal
  goal.target_pose.pose.position.x = dropX;
  goal.target_pose.pose.position.y = dropY;
  goal.target_pose.pose.orientation.w = 1.0;
  goals.push_back(goal);

  // send goals
  
  // Send the goal position and orientation for the robot to reach
  int counter = 1;
  for (const auto& g : goals)
  {  
    ROS_INFO("Sending goal %d", counter);
    ac.sendGoal(g);

    // Wait an infinite time for the results
    ac.waitForResult();

    // Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Successfully reached goal %d", counter);
      ROS_INFO("Waiting 5 seconds to interact with target");
      ros::Duration(5.0).sleep();
    }
    else
    {
      ROS_INFO("Failed to reach goal");
    }
    counter++;
  }

  ros::spin();

  return 0;
}
