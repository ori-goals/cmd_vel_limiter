#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

double throttle_distance;

serCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  for(auto i = msg->ranges.begin(); i != msg->ranges.end(); ++i) {
    if(*i <= throttle_distance) {
      ROS_INFO_STREAM("Threshold under: " << *i);
      break;
    }
  }


}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_threshold");
  ros::NodeHandle n;
  n.param("throttle_distance", throttle_distance, 1.0);
  ros::Subscriber sub = n.subscribe("scan", 10, laserCallback);
  ros::spin();

  return 0;
}