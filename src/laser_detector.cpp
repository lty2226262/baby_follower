#include <laser_detector.h>

#define DEBUG_LASE_R

namespace laserdetector {

LaserDetector::LaserDetector(ros::NodeHandle& node_handle): node_handle_(node_handle){
  node_handle_.param("laser_topic_name", laser_topic_name_, std::string("/scan"));
  node_handle_.param("lidar_frame", laser_frame_name_, std::string("/laser"));
  node_handle_.param("laser_config_file_name", laser_config_file_name_, std::string("/home/tt/.ros/LS01.yml")); 
  
  
  scan_subscriber_ = node_handle_.subscribe(laser_topic_name_, 1, &LaserDetector::LaserScanCallback, this);
}


bool LaserDetector::GetHumanPosition(baby_follower::laser_detector_is_enabled::Request& request, baby_follower::laser_detector_is_enabled::Response& response){
  if (request.data == true){
    Human result;
    result = SelectBestHuman(HumansPositionCauculation(0.15));
    if (result.is_valid == true){
      // print the info about best human
      std::cout << " Best Human:" << std::endl;
      std::cout << "range : " << result.begin << "~" << result.end << std::endl;
      std::cout << "width : " << result.human_width << std::endl;
      std::cout << "position : " << result.human_position << std::endl;
      std::cout << "distance : " << result.human_distance << std::endl;
      std::cout << "rising_edge_count :" << result.rising_edge_count << std::endl;
      std::cout << "trailing_edge_count :" << result.trailing_edge_count  << std::endl;
      std::cout << "inf_count :" << result.inf_count  << std::endl;
      std::cout << "obstacle_count :" << result.obstacle_count  << std::endl;
      std::cout << "human_confidence :" << result.human_confidence  << std::endl;
      
      response.msg.header.frame_id = laser_frame_name_;
      response.msg.header.stamp = ros::Time::now();
      response.msg.point.x = result.human_distance * cos(result.human_position * laser_angle_increment_radians_);
      response.msg.point.y = result.human_distance * sin(result.human_position * laser_angle_increment_radians_);
      response.msg.point.z = 0.0;
      return true;
    }else{
      return false;
    }
  }
}

bool LaserDetector::GetHumanPositions(baby_follower::return_all_humans::Request& request, baby_follower::return_all_humans::Response& response){
  std::vector<Human> inputs = HumansPositionCauculation(0.05);
  baby_follower::human output;
  for (auto it = inputs.begin(); it != inputs.end(); it++){
    output.begin = (double)(it -> begin)/ 180 * 3.141592653589793;
    output.distance = it -> human_distance;
    output.end = (double)(it -> end)/ 180 * 3.141592653589793;
    output.num = (int)(it - inputs.begin());
    output.position = it -> human_position / 180 * 3.141592653589793;
    output.width = it -> human_width/ 180 * 3.141592653589793;
    response.results.data.push_back(output);
  }
  return true;
}

void LaserDetector::LaserScanCallback(const sensor_msgs::LaserScan& scan_value){
  
  laser_ranges_ = scan_value.ranges;
  //Mask initialization
  if (mask_not_initialized) {   
    laser_angle_size_ = laser_ranges_.size();
    laser_angle_increment_ = 360 / laser_angle_size_; //degree
    laser_angle_increment_radians_ = 2 * 3.141592653589793 / laser_angle_size_; //radians
    MaskInitialization();
    mask_not_initialized = false;
  }
}

void LaserDetector::MaskInitialization(){
  cv::Mat outlier_index_mat;
  cv::FileStorage file_storage;
  while(ros::ok()){
    file_storage.open(laser_config_file_name_, cv::FileStorage::READ);
    if (file_storage.isOpened()){
      file_storage["outlier_index"] >> outlier_index_mat;
      break;
    } else {
      ROS_ERROR_STREAM("Laser config file cannot find at: " << laser_config_file_name_);
    }
  }
  
  //Convert mat to vector
  std::vector<int> outlier_index(outlier_index_mat.cols);
  memcpy(outlier_index.data(), outlier_index_mat.data, (outlier_index_mat.cols) * sizeof(int));
  laser_mask_ = new int[laser_angle_size_]();
  for (std::vector<int>::iterator it = outlier_index.begin(); it != outlier_index.end(); it++){
    laser_mask_[*it] = 1;
// #ifdef DEBUG_LASER
//     std::cout << *it << ", ";
// #endif
  }
  for (int i = 0; i < laser_angle_size_; i++){
    if (laser_mask_[i] == 1){
      laser_ranges_[i] = laser_ranges_[PositiveMod(i-1, laser_angle_size_)];
    } else ;
  }
}

std::vector<Human> LaserDetector::HumansPositionCauculation(double k_distance_diff_limitation){
  k_distance_diff_limitation_ = k_distance_diff_limitation;
  //Initialize for humans
  std::vector<Human> humans;
  humans.push_back(Human());
  
  //Compute distance differential coefficient
  double* distance_diff = new double[laser_angle_size_];
// #ifdef DEBUG_LASER
//   for (int i = 0; i < laser_angle_size_;i++) std::cout <<" " << i << ":" << laser_ranges_[i] << ",";
// #endif
  for (int i = 0; i < laser_angle_size_;i++)  distance_diff[i] = laser_ranges_[PositiveMod(i - 1, laser_angle_size_)] - laser_ranges_[i];
  for (int i = 0; i < laser_angle_size_; i++){
    int result_of_distance_diff = InLimitation(distance_diff[i]);
    
    if(result_of_distance_diff != 0){
      humans.back().end = PositiveMod(i - 1, laser_angle_size_);
      if (result_of_distance_diff == 1) humans.back().rising_edge_count++;
      else if (result_of_distance_diff == -1) humans.back().trailing_edge_count++;
      else ;
      humans.push_back(Human());
      humans.back().begin = i;
      if (result_of_distance_diff == 1) humans.back().rising_edge_count++;
      else if (result_of_distance_diff == -1) humans.back().trailing_edge_count++;
      else;
    } else; // no diff changes
    
    if (laser_mask_[i] == 1)
      humans.back().obstacle_count++;
    else;
    if (std::isinf(laser_ranges_[i]))
      humans.back().inf_count++;
    else;
  }
  
// #ifdef DEBUG_LASER
//   laser_ranges_.front() = laser_ranges_.back();
// #endif
// #ifdef DEBUG_LASER
//   laser_ranges_.front() = 1.0/0.0;
// #endif
  
  // Deal with loop and first element 
  if (InLimitation(distance_diff[0]) == 0){
    humans.front().begin = humans.back().begin;
    humans.front().rising_edge_count += humans.back().rising_edge_count;
    humans.front().trailing_edge_count += humans.back().trailing_edge_count;
    humans.front().obstacle_count += humans.back().obstacle_count;
    humans.front().inf_count += humans.back().inf_count;
    humans.front().human_width = (laser_angle_size_ + 1 - humans.front().begin + humans.front().end) * laser_angle_increment_;
    humans.front().human_position = PositiveMod((laser_angle_size_ + humans.front().end + humans.front().begin) / 2, laser_angle_size_) * laser_angle_increment_;
    humans.front().human_distance = (laser_ranges_[humans.front().begin] + laser_ranges_[humans.front().end]) / 2.0;
    humans.pop_back();
  } else {
    humans.back().end = laser_angle_size_ - 1;
    humans.back().rising_edge_count += humans.front().rising_edge_count;
    humans.back().trailing_edge_count += humans.front().trailing_edge_count;
    humans.erase(humans.begin());
// #ifdef DEBUG_LASER
//     std::cout << "*********************"<< std::endl;
//     std::cout << "begin: " << humans[0].begin << ", " << "end: " << humans[0].end << std::endl;
//     std::cout << "*********************"<< std::endl;
// #endif
    humans.front().human_width = (1 - humans.front().begin + humans.front().end) * laser_angle_increment_;
    humans.front().human_position =  ((double)(humans.front().end + humans.front().begin)) / 2.0 * laser_angle_increment_;
    humans.front().human_distance = (laser_ranges_[humans.front().begin] + laser_ranges_[humans.front().end]) / 2.0;
  }
  
  // Deal with other elements
  for (int i = 1; i < humans.size(); i++){
    humans[i].human_width = (1 - humans[i].begin + humans[i].end) * laser_angle_increment_;
    humans[i].human_position =  ((double)(humans[i].end + humans[i].begin)) / 2.0 * laser_angle_increment_;
    humans[i].human_distance = (laser_ranges_[humans[i].begin] + laser_ranges_[humans[i].end]) / 2.0;
  }

// #ifdef DEBUG_LASER
//   std::cout << std::endl;
//   int i = 0;
//   for (std::vector<Human>::iterator it = humans.begin(); it != humans.end(); ++it ){
//     std::cout << "Human" << i++ << " : " << std::endl;
//     std::cout << "range : " << it->begin << "~" << it->end << std::endl;
//     std::cout << "width : " << it->human_width << std::endl;
//     std::cout << "position : " << it->human_position << std::endl;
//     std::cout << "distance : " << it->human_distance << std::endl;
//     std::cout << "rising_edge_count :" << it->rising_edge_count << std::endl;
//     std::cout << "trailing_edge_count :" << it->trailing_edge_count  << std::endl;
//     std::cout << "inf_count :" << it->inf_count  << std::endl;
//     std::cout << "obstacle_count :" << it->obstacle_count  << std::endl;
//   }
// #endif
  
  // Find and process outlier
  bool changed = true;
#ifdef DEBUG_LASER
  int count = 0;
#endif
  while (changed){
    changed = false;
    for (int i = 0; i < humans.size();){
      int temp_humans_size = humans.size();
      int i_minus_one = PositiveMod(i - 1, temp_humans_size);
      int i_plus_one = PositiveMod(i + 1, temp_humans_size);
      
      if (((humans[i].human_width < 2) || (humans[i].human_width - humans[i].obstacle_count < 2)) && 
	( (fabs(humans[i_minus_one].human_distance - humans[i_plus_one].human_distance) < k_distance_diff_limitation_) ||  std::isnan(humans[i_minus_one].human_distance - humans[i_plus_one].human_distance) )  ){
// #ifdef DEBUG_LASER
// 	std::cout << "going to modify element:" << i << "," << i_minus_one << "," << i_plus_one << std::endl;
// #endif
	humans[i].rising_edge_count = -humans[i].rising_edge_count + humans[i_minus_one].rising_edge_count + humans[i_plus_one].rising_edge_count;
	humans[i].trailing_edge_count = -humans[i].trailing_edge_count + humans[i_minus_one].trailing_edge_count + humans[i_plus_one].trailing_edge_count;
	humans[i].begin = humans[i_minus_one].begin;
	humans[i].end = humans[i_plus_one].end;
	humans[i].human_position = (humans[i].begin < humans[i].end) ? ((humans[i].end + humans[i].begin) / 2.0) : (PositiveMod((humans[i].end + laser_angle_size_ + humans[i].begin), laser_angle_size_) / 2.0);
	humans[i].human_width = (humans[i].begin < humans[i].end) ? (humans[i].end - humans[i].begin + 1) : (humans[i].end +laser_angle_size_ - humans[i].begin + 1);
	humans[i].obstacle_count = humans[i_minus_one].obstacle_count + humans[i].obstacle_count + humans[i_plus_one].obstacle_count;
	humans[i].inf_count = humans[i_minus_one].inf_count + humans[i].inf_count + humans[i_plus_one].inf_count;
	humans[i].human_distance = (humans[i_minus_one].human_distance + humans[i_plus_one].human_distance) / 2.0;
// #ifdef DEBUG_LASER
// 	int i_bak = i;
// 	i = i_minus_one;
// 	std::cout << "fromer_Human" << i << " : " << std::endl;
// 	std::cout << "range : " << humans[i].begin << "~" << humans[i].end << std::endl;
// 	std::cout << "width : " << humans[i].human_width << std::endl;
// 	std::cout << "position : " << humans[i].human_position << std::endl;
// 	std::cout << "distance : " << humans[i].human_distance << std::endl;
// 	std::cout << "rising_edge_count :" << humans[i].rising_edge_count << std::endl;
// 	std::cout << "trailing_edge_count :" << humans[i].trailing_edge_count  << std::endl;
// 	std::cout << "inf_count :" << humans[i].inf_count  << std::endl;
// 	std::cout << "obstacle_count :" << humans[i].obstacle_count  << std::endl;
// 	i = i_bak;
// 	std::cout << "this_Human" << i << " : " << std::endl;
// 	std::cout << "range : " << humans[i].begin << "~" << humans[i].end << std::endl;
// 	std::cout << "width : " << humans[i].human_width << std::endl;
// 	std::cout << "position : " << humans[i].human_position << std::endl;
// 	std::cout << "distance : " << humans[i].human_distance << std::endl;
// 	std::cout << "rising_edge_count :" << humans[i].rising_edge_count << std::endl;
// 	std::cout << "trailing_edge_count :" << humans[i].trailing_edge_count  << std::endl;
// 	std::cout << "inf_count :" << humans[i].inf_count  << std::endl;
// 	std::cout << "obstacle_count :" << humans[i].obstacle_count  << std::endl;
// 	i = i_plus_one;
// 	std::cout << "this_Human" << i << " : " << std::endl;
// 	std::cout << "range : " << humans[i].begin << "~" << humans[i].end << std::endl;
// 	std::cout << "width : " << humans[i].human_width << std::endl;
// 	std::cout << "position : " << humans[i].human_position << std::endl;
// 	std::cout << "distance : " << humans[i].human_distance << std::endl;
// 	std::cout << "rising_edge_count :" << humans[i].rising_edge_count << std::endl;
// 	std::cout << "trailing_edge_count :" << humans[i].trailing_edge_count  << std::endl;
// 	std::cout << "inf_count :" << humans[i].inf_count  << std::endl;
// 	std::cout << "obstacle_count :" << humans[i].obstacle_count  << std::endl;
// 	i = i_bak;
// #endif
	humans.erase(humans.begin() + (i_minus_one > i_plus_one ? i_minus_one: i_plus_one));
	try {
	  humans.erase(humans.begin() + (i_minus_one < i_plus_one ? i_minus_one: i_plus_one));
	} catch (const std::exception& e){
	  ROS_ERROR_STREAM("Only one element:" << e.what());
	}
// #ifdef DEBUG_LASER
// 	std::cout << "size:" << humans.size();
// #endif
	changed = true;
      } else {
	i++;
      }
    }
// #ifdef DEBUG_LASER
//     count++;
//     std::cout << "count!!" << count;
// #endif
  }
  
  for (std::vector<Human>::iterator it = humans.begin(); it!= humans.end(); ++it){
    it -> human_confidence *= ((it -> human_distance < human_distance_max_) && (it -> human_distance > human_distance_min_)) ? reward_coefficient_ : penalty_coefficient_;
    it -> human_confidence *= ((it -> human_position > human_position_max_) && (it -> human_position < human_position_min_)) ? penalty_coefficient_: reward_coefficient_;
    it -> human_confidence *= ((it -> human_width < human_width_max_) && (it -> human_width > human_width_min_)) ? reward_coefficient_: 1.0;
    it -> human_confidence *= ((it -> rising_edge_count == 1) && (it -> trailing_edge_count == 1)) ? reward_coefficient_ : 1.0;
  }
  
  std::sort(humans.begin(), humans.end());
  
  
  
// #ifdef DEBUG_LASER
//   std::cout << std::endl;
//   int i = 0;
//   for (std::vector<Human>::iterator it = humans.begin(); it != humans.end(); ++it ){
//     std::cout << "Human" << i++ << " : " << std::endl;
//     std::cout << "range : " << it->begin << "~" << it->end << std::endl;
//     std::cout << "width : " << it->human_width << std::endl;
//     std::cout << "position : " << it->human_position << std::endl;
//     std::cout << "distance : " << it->human_distance << std::endl;
//     std::cout << "rising_edge_count :" << it->rising_edge_count << std::endl;
//     std::cout << "trailing_edge_count :" << it->trailing_edge_count  << std::endl;
//     std::cout << "inf_count :" << it->inf_count  << std::endl;
//     std::cout << "obstacle_count :" << it->obstacle_count  << std::endl;
//     std::cout << "human_confidence :" << it->human_confidence  << std::endl;
//   }
// #endif
  return humans;
}

Human LaserDetector::SelectBestHuman(std::vector<Human> result_in){
  if (result_in.size() > 1){
    std::vector<Human> max_results;
    double max_confidence_value = result_in[0].human_confidence;
    for (int i = 0; (i < result_in.size()) && (result_in[i].human_confidence == max_confidence_value) ; i++) max_results.push_back(result_in[i]);
    if (max_results.size() == 1) return max_results[0];
    else {
      ROS_WARN_STREAM("TOO MANY PEOPLE STAND IN FRONT OF ME, ONLY ONE IS ENOUGH");
      double nearest_distance_value = 1.0/0.0;
      int nearest_distance_index = -1;
      for (auto it = max_results.begin(); it != max_results.end(); it++){
	if (it -> human_distance < nearest_distance_value) {
	  nearest_distance_value = it -> human_distance;
	  nearest_distance_index = it - max_results.begin();
	}
	std::cout << "Human" << it - max_results.begin() << " : " << std::endl;
	std::cout << "range : " << it->begin << "~" << it->end << std::endl;
	std::cout << "width : " << it->human_width << std::endl;
	std::cout << "position : " << it->human_position << std::endl;
	std::cout << "distance : " << it->human_distance << std::endl;
	std::cout << "rising_edge_count :" << it->rising_edge_count << std::endl;
	std::cout << "trailing_edge_count :" << it->trailing_edge_count  << std::endl;
	std::cout << "inf_count :" << it->inf_count  << std::endl;
	std::cout << "obstacle_count :" << it->obstacle_count  << std::endl;
	std::cout << "human_confidence :" << it->human_confidence  << std::endl;
      }
      ROS_WARN_STREAM("NOW I RETURN THE NEAREST ONE FROM ME, HUMAN" << nearest_distance_index);
      return max_results[nearest_distance_index];
    }
  }else{
    ROS_ERROR_STREAM("ONLY ONE DIFF RESULT, IMPOSSIBLE!");
    Human invalid_human = Human();
    invalid_human.is_valid = false;
    return invalid_human;
  }
}

int LaserDetector::PositiveMod(int divident, int divisor){
  return ((divident % divisor) + divisor) % divisor;
}

int LaserDetector::InLimitation(double num){

  if (std::isnan(num)){
    return 0;
  } else if (num > k_distance_diff_limitation_){
    return 1;
  } else if (num < -k_distance_diff_limitation_){
    return -1;
  } else {
    return 0;
  }
}
}

int main (int argc, char** argv){
  // Init the ROS node
  ros::init(argc, argv, "laser_detector");
  ros::NodeHandle node_handle;
  
  laserdetector::LaserDetector laser_detector(node_handle);
  ROS_INFO_STREAM("Laser config file load accomplish"); 
  ros::ServiceServer service = node_handle.advertiseService("detector_is_enabled", &laserdetector::LaserDetector::GetHumanPosition, &laser_detector);
  ros::ServiceServer service_all = node_handle.advertiseService("return_all_humans", &laserdetector::LaserDetector::GetHumanPositions, &laser_detector);
  ros::spin();
}
