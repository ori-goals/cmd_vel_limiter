#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

// any range value less than this will trigger speed reduction
double throttle_distance;
//value to cap x vel to when throttle_distance is violated
double max_x_vel;
// whether the distrance threshold has been violated
bool apply_throttle = false;

// topic for publishing thresholded commands
ros::Publisher cmd_vel_pub;

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  for(auto i = msg->ranges.begin(); i != msg->ranges.end(); ++i) {
    if(*i < throttle_distance) {
      ROS_DEBUG_STREAM("Threshold under: " << *i);
      apply_throttle = true;
      return;
    }
  }
  apply_throttle = false;
}

void cmdCallback(const geometry_msgs::Twist::Ptr& msg)
{

  if(apply_throttle) {
    // apply limit
    if(msg->linear.x > max_x_vel) {
      ROS_DEBUG_STREAM("Applying limit forwards");
      msg->linear.x = max_x_vel;
    }
    else if(msg->linear.x < -max_x_vel) {
      ROS_DEBUG_STREAM("Applying limit backwards");
      msg->linear.x = -max_x_vel;
    }
  }
  cmd_vel_pub.publish(msg);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_threshold");
  ros::NodeHandle n;
  n.param("throttle_distance", throttle_distance, 1.0);
  n.param("max_x_vel", max_x_vel, 0.1);

  ROS_INFO_STREAM("Throttling under distance: " << throttle_distance << "m");
  ROS_INFO_STREAM("Throttling to max x vel: " << max_x_vel << "m/s");

  ros::Subscriber scan_sub = n.subscribe("scan", 10, laserCallback);
  ros::Subscriber cmd_sub = n.subscribe("cmd_vel", 10, cmdCallback);
  cmd_vel_pub = n.advertise<geometry_msgs::Twist>("max_x_vel", 10);
  ros::spin();

  return 0;
}