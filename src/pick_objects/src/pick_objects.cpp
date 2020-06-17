#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef move_base_msgs::MoveBaseGoal MoveGoal;



class ObjectHandler
{
  public:
  ObjectHandler()
  {
    object_loaded_ = 0;
  }
		
  MoveGoal define_goal(float x, float y);
  void handle_goal(MoveGoal goal, MoveBaseClient& move_base_client);
	
  bool object_loaded_ = 0;
		
};

MoveGoal ObjectHandler::define_goal(float x, float y)
{
  MoveGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation.w = 1.0;
	
  return goal;	
}

void ObjectHandler::handle_goal(MoveGoal goal, MoveBaseClient& move_base_client)
{
   

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  move_base_client.sendGoal(goal);

  // Wait an infinite time for the results
  move_base_client.waitForResult();
    
  // Check if the robot reached its goal and (un)load object
  if(move_base_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Hooray, goal reached");
    object_loaded_ = !object_loaded_;
    ros::Duration(5.0).sleep();
	//"Payload state: " + object_loaded_
    ROS_INFO("Payload state: ");
  }
  else
    ROS_INFO("The base failed to reach the goal");  
    
}	


int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "simple_navigation_goals");
  MoveBaseClient move_base_client("move_base", true);
  // Wait 5 sec for move_base action server to come up
  while(!move_base_client.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  
  ObjectHandler object_handler;    

  // define movement goals
  MoveGoal pickup = object_handler.define_goal(-1., -1.0);
  MoveGoal dropoff = object_handler.define_goal(-3.0, 2.0);
  
  // pickup object
  object_handler.handle_goal(pickup, move_base_client);
  
  // drop off object
  if(object_handler.object_loaded_)
	  object_handler.handle_goal(dropoff, move_base_client);	
  
  return 0;
}
