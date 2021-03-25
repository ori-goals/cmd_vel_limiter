#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include <math.h>

// Globals which are set by rosparam, do not edit here

// any range value less than this will trigger speed reduction
double distance_threshold;

// values in this range of the threshold will have scaling applied
double scale_range = 0;

//value to cap x vel to when distance_threshold is violated
double max_x_vel;
// how much of the limit should be applied
double scale_factor = 1.0;

// angular ranges in in degreess
double lower_bound_angle;
double upper_bound_angle;


// whether the distrance threshold has been violated
bool apply_limit = true;
// whether the distrance threshold has been violated
bool limit_in_reverse = false;


// topic for publishing thresholded commands
ros::Publisher cmd_vel_pub;

bool angle_indexes_set = false;
int lower_idx = -1;
int upper_idx = -1;

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  //compute indexes to relect angle
  if(!angle_indexes_set) {


    // if the lower_bound_angle is within the range of the scan use it
    if(lower_bound_angle > msg->angle_min) {
      // how many radians we need to increment up from min angle
      auto diff_rads = lower_bound_angle - msg->angle_min; 
      // convert this into number of readings
      lower_idx = floor(diff_rads / msg->angle_increment);
    }
    else {
      lower_idx = 0;
    }

    // if the upper_bound_angle is within the range of the scan use it
    if(upper_bound_angle < msg->angle_max) {
      // how many radians we need to increment down from max angle
      auto diff_rads = msg->angle_max - upper_bound_angle;
      // convert into number of readings 
      upper_idx = msg->ranges.size() - ceil(diff_rads / msg->angle_increment);
    }
    else {
      upper_idx = msg->ranges.size() - 1;
    }

    angle_indexes_set = true;
    ROS_INFO_STREAM("Indexes limits: " << 0 << " to " << msg->ranges.size() - 1);
    ROS_INFO_STREAM("Indexes bounds: " << lower_idx << " to " << upper_idx);
  }


  // we were seeing values less than range_min on the robot.
  auto min_possible_value = msg->range_min;
  // track smallest value
  double  min_reading = msg->range_max;

  apply_limit = false;
  for(int i = lower_idx; i <= upper_idx; ++i) {
    auto reading = msg->ranges.at(i);
    if(reading > min_possible_value) {
      
      if(reading < (distance_threshold + scale_range)) {
        
        apply_limit = true;

        if(reading < min_reading) {
          min_reading = reading;
        }
      }
    }
  }
    ROS_DEBUG_STREAM("Min reading: " << min_reading);

  if(apply_limit) {
    scale_factor = 0;
    if(min_reading > distance_threshold) {
      scale_factor = (min_reading - distance_threshold) / scale_range;
    }
    ROS_DEBUG_STREAM("Value under threshold: " << min_reading << " scale factor " << scale_factor);

  }

}

void cmdCallback(const geometry_msgs::Twist::Ptr& msg)
{

  if(apply_limit) {
    // apply limit
    if(msg->linear.x > max_x_vel) {
      ROS_DEBUG_STREAM("Applying limit forwards");
      msg->linear.x = max_x_vel + ((msg->linear.x - max_x_vel) * scale_factor);
    }
    else if(limit_in_reverse && msg->linear.x < -max_x_vel) {
      ROS_DEBUG_STREAM("Applying limit backwards");
      msg->linear.x = -max_x_vel - ((msg->linear.x + max_x_vel) * scale_factor);
    }
  }
  cmd_vel_pub.publish(msg);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_threshold");
  ros::NodeHandle priv("~"); 
  priv.param("distance_threshold", distance_threshold, 1.0);
  priv.param("scale_range", scale_range, 0.0);
  priv.param("max_x_vel", max_x_vel, 0.1);
  priv.param("limit_in_reverse", limit_in_reverse, false);
  priv.param("lower_bound_angle", lower_bound_angle, -M_PI/2);
  priv.param("upper_bound_angle", upper_bound_angle, M_PI/2);


  ROS_INFO_STREAM("Throttling under distance: " << distance_threshold << "m");
  ROS_INFO_STREAM("Scaling range above threshold: " << scale_range << "m");
  ROS_INFO_STREAM("Throttling to max x vel: " << max_x_vel << "m/s");
  ROS_INFO_STREAM("Limiting in reverse: " << limit_in_reverse);
  ROS_INFO_STREAM("Angle bounds: " << lower_bound_angle << " to " << upper_bound_angle);

  ros::NodeHandle n; 
  ros::Subscriber scan_sub = n.subscribe("scan", 5, laserCallback);
  ros::Subscriber cmd_sub = n.subscribe("cmd_vel", 5, cmdCallback);
  cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel_limited", 5);
  ros::spin();

  return 0;
}