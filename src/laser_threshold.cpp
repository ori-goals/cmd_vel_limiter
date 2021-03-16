#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

// Globals which are set by rosparam, do not edit here

// any range value less than this will trigger speed reduction
double distance_threshold;
//value to cap x vel to when distance_threshold is violated
double max_x_vel;
// whether the distrance threshold has been violated
bool apply_limit = false;
// whether the distrance threshold has been violated
bool limit_in_reverse = false;


// topic for publishing thresholded commands
ros::Publisher cmd_vel_pub;

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  for(auto i = msg->ranges.begin(); i != msg->ranges.end(); ++i) {
    if(*i < distance_threshold) {
      ROS_DEBUG_STREAM("Value under threshold: " << *i);
      apply_limit = true;
      return;
    }
  }
  apply_limit = false;
}

void cmdCallback(const geometry_msgs::Twist::Ptr& msg)
{

  if(apply_limit) {
    // apply limit
    if(msg->linear.x > max_x_vel) {
      ROS_DEBUG_STREAM("Applying limit forwards");
      msg->linear.x = max_x_vel;
    }
    else if(limit_in_reverse && msg->linear.x < -max_x_vel) {
      ROS_DEBUG_STREAM("Applying limit backwards");
      msg->linear.x = -max_x_vel;
    }
  }
  cmd_vel_pub.publish(msg);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_threshold");
  ros::NodeHandle priv("~"); 
  priv.param("distance_threshold", distance_threshold, 1.0);
  priv.param("max_x_vel", max_x_vel, 0.1);
  priv.param("limit_in_reverse", limit_in_reverse, false);

  ROS_INFO_STREAM("Throttling under distance: " << distance_threshold << "m");
  ROS_INFO_STREAM("Throttling to max x vel: " << max_x_vel << "m/s");
  ROS_INFO_STREAM("Limiting in reverse: " << limit_in_reverse);

  ros::NodeHandle n; 
  ros::Subscriber scan_sub = n.subscribe("scan", 10, laserCallback);
  ros::Subscriber cmd_sub = n.subscribe("cmd_vel", 10, cmdCallback);
  cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel_limited", 10);
  ros::spin();

  return 0;
}