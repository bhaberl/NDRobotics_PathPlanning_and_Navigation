#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <string>


typedef visualization_msgs::Marker Marker;

const float kEpsilon = 0.5;

const float pickup_x_ = -1.5;
const float pickup_y_ = 0.0;
const float dropoff_x_ = -4.0;
const float dropoff_y_ = 3.0;

bool pickup_state_ = false;
bool dropoff_state_ = false;


class MarkerManager
{
  public:
  MarkerManager(){}

  //void OdometryTrackingCallback(const nav_msgs::Odometry& odom);
  void init_marker(uint32_t shape);
  void update_marker();    						;

  Marker marker_;
};


void MarkerManager::update_marker()
{
    //ROS_INFO("***UPDATE MARKER***");
    if (!pickup_state_ && !dropoff_state_)
    {
      marker_.action = Marker::ADD;
      //ROS_INFO("ADD MARKER");
    }
    if (pickup_state_ && !dropoff_state_)
    {
      marker_.action = Marker::DELETE;    
      ROS_INFO("DELETE MARKER");
    }
    if (pickup_state_ && dropoff_state_)
    {
      marker_.pose.position.x = dropoff_x_;
      marker_.pose.position.y = dropoff_y_;
      marker_.action = Marker::ADD; 
      ROS_INFO("ELSE ADD MARKER");
    }

}

void MarkerManager::init_marker(uint32_t shape)
{
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker_.header.frame_id = "map";
    marker_.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker_.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker_.ns = "basic_shapes";
    marker_.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker_.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker_.action = Marker::ADD;

    // Set the pose of the marker_.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker_.pose.position.x = pickup_x_;
    marker_.pose.position.y = pickup_y_;
    marker_.pose.position.z = 0;
    marker_.pose.orientation.x = 0.0;
    marker_.pose.orientation.y = 0.0;
    marker_.pose.orientation.z = 0.0;
    marker_.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker_.scale.x = 1.0;
    marker_.scale.y = 1.0;
    marker_.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker_.color.r = 0.0f;
    marker_.color.g = 1.0f;
    marker_.color.b = 0.0f;
    marker_.color.a = 1.0;

    marker_.lifetime = ros::Duration();
    ROS_INFO("MARKER INITIALIZED");
	
}

void OdometryTrackingCallback(const nav_msgs::Odometry& odom)
{
  float odom_x = odom.pose.pose.position.x;
  float odom_y = odom.pose.pose.position.y;
		 
  float pickup_distance = sqrt(pow(pickup_x_ - odom_x, 2) + pow(pickup_y_ - odom_y, 2));
  float dropoff_distance = sqrt(pow(dropoff_x_ - odom_x, 2) + pow(dropoff_y_ - odom_y, 2));
  ROS_INFO("**********************PICKUP_DISTANCE:");
  std::string msg1 = std::to_string(pickup_distance);
  ROS_INFO_STREAM(msg1);
  ROS_INFO("**********************DROPOFF_DISTANCE");
  std::string msg2 = std::to_string(dropoff_distance);
  ROS_INFO_STREAM(msg2);
  
  if (pickup_distance < kEpsilon) 
  {
    pickup_state_ = true;
    ROS_INFO("************************PICKUP STATE TRUE");
  }
  else if (dropoff_distance < kEpsilon)
  {
    dropoff_state_ = true;
    ROS_INFO("************************DROPOFF STATE TRUE");
  }
}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle node;
  ros::Rate r(1);
  
  MarkerManager marker_manager;
  marker_manager.init_marker(Marker::CUBE);  

  ros::Publisher marker_pub_ = node.advertise<Marker>("visualization_marker", 1);
  ros::Subscriber odometry_subscriber_ = node.subscribe("/odom", 10, OdometryTrackingCallback);
  
  while (ros::ok())
  {
    marker_manager.update_marker();
    marker_pub_.publish(marker_manager.marker_);
	
    // Handle ROS communication events
    ros::spinOnce();
      
  }
}
