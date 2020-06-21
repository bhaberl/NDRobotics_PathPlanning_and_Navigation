#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <math.h>


typedef visualization_msgs::Marker Marker;

const float kEpsilon = 0.01;

const float pickup_x_ = -1.0;
const float pickup_y_ = -1.0;
const float dropoff_x_ = -3.0;
const float dropoff_y_ = 2.0;

bool pickup_state_;
bool dropoff_state_;


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
    if (!pickup_state_ && !dropoff_state_)
      marker_.action = visualization_msgs::Marker::ADD;
    if (pickup_state_ && !dropoff_state_)
      marker_.action = visualization_msgs::Marker::DELETE;    
    if (pickup_state_ && dropoff_state_)
    {
      marker_.pose.position.x = dropoff_x_;
      marker_.pose.position.y = dropoff_y_;
      marker_.action = Marker::ADD;    
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
	
    pickup_state_ = 0;
    dropoff_state_ = 0;
}

void OdometryTrackingCallback(const nav_msgs::Odometry& odom)
{
  float odom_x = odom.pose.pose.position.x;
  float odom_y = odom.pose.pose.position.y;
		 
  // TODO: calculate goal states
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
		// Publish the marker
    /*while (marker_pub_.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }*/
    marker_manager.update_marker();
    marker_pub_.publish(marker_manager.marker_);
	
	// Handle ROS communication events
    ros::spinOnce();
      
  }
}
