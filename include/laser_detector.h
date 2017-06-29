#ifndef LASER_DETECTOR_H
#define LASER_DETECTOR_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <limits>
#include <math.h>
#include <algorithm>
#include "baby_follower/return_all_humans.h"
#include "baby_follower/laser_detector_is_enabled.h"
#include "baby_follower/human.h"
#include "baby_follower/human_array.h"

namespace laserdetector{

//Human struct
struct Human{
public:
  int rising_edge_count = 0;
  int trailing_edge_count = 0;
  int begin = 0;
  int end = 0;
  double human_position = 0.0;
  double human_width = 0.0;
  int obstacle_count = 0;
  int inf_count = 0;
  double human_distance = 0.0;
  double human_confidence = 1.0;
  bool is_valid = true;
  Human(){}
  
  // Sort by descending
  bool operator < (const Human& Another) const{
    return (human_confidence > Another.human_confidence);
  }
};
  
//Laser detect human class
class LaserDetector{
  ros::NodeHandle node_handle_;
  
  //Deal with the laserscan input
  double laser_angle_increment_ = 0.0;
  double laser_angle_increment_radians_ = 0.0;
  int laser_angle_size_;
  std::vector<float> laser_ranges_;
  std::string laser_topic_name_;
  std::string laser_frame_name_;
  ros::Subscriber scan_subscriber_;
  
  //Laserscan Mask
  std::string laser_config_file_name_;
  bool mask_not_initialized = true;
  int* laser_mask_;
  
  //Laserscan process
  double k_distance_diff_limitation_ = 0.15;
  
  //Human detect
  // Identify human or not: a decision-tree like model
  // Human distance 0.1~0.8 reward else, penalty, inf penalty to 0 
  // Human position: 90~270 penalty else reward
  // Human width: 3~50 reward
  // Human trailing_edge_count = rising_edge_count = 1 reward  
  const double reward_coefficient_ = 1.80, penalty_coefficient_ = 0.60;
  const double human_distance_max_ = 0.8, human_distance_min_ = 0.1;
  const double human_position_max_ = 60.0, human_position_min_ = 300.0;
  const double human_width_max_ = 50.0, human_width_min_ = 3.0;
  
  // Lidar coordinate transformation
  
  
  //Laser scan callback
  void LaserScanCallback(const sensor_msgs::LaserScan& scan_value);
  //Mask Initialization
  void MaskInitialization();
  //Human position cauculation 
  std::vector<Human> HumansPositionCauculation(double k_distance_diff_limitation);
  //Count the positive mod  
  int PositiveMod(int divident, int divisor);
  //Compute limitation from diff
  int InLimitation(double num);
  //Select the best one from humans
  Human SelectBestHuman(std::vector<Human> result_in);

public:  
  LaserDetector(ros::NodeHandle& node_handle);
  //The service response functions
  bool GetHumanPosition(baby_follower::laser_detector_is_enabled::Request &request,
			baby_follower::laser_detector_is_enabled::Response &response);
  bool GetHumanPositions(baby_follower::return_all_humans::Request& request, 
			 baby_follower::return_all_humans::Response& response);
};


}
#endif //LASER_DETECTOR_H