#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/LaserScan.h>



void LidarOutlierPickerCallback(const sensor_msgs::LaserScan &input){
  //Configure parameters
  std::string filename = "LS01.yml";
  double range_limit = 1.0;
  
  std::vector<int> outlier_index_vector;
  std::vector<double> outlier_value_vector;
  cv::FileStorage file_storage(filename, cv::FileStorage::WRITE);
  int outlier_size;
  
  int size = input.ranges.size();
  
  for (int i = 0; i<size; i++){
    if (input.ranges[i] < range_limit){
      outlier_index_vector.push_back(i);
      outlier_value_vector.push_back(input.ranges[i]);
    } else;
  }
  
  //Convert vector to cvMat
  outlier_size = outlier_index_vector.size();
  cv::Mat_<int> outlier_index(1,outlier_size);
  cv::Mat_<double> outlier_value(1,outlier_size);
  memcpy(outlier_index.data, outlier_index_vector.data(), outlier_size*sizeof(int));
  memcpy(outlier_value.data, outlier_value_vector.data(), outlier_size*sizeof(double));
  
  //Write yaml
  file_storage << "range_limit" << range_limit;
  file_storage << "outlier_index" << outlier_index;
  file_storage << "outlier_value" << outlier_value;
  
  file_storage.release();
  
  std::cout << "Write done at: " << filename << std::endl;
  ros::shutdown();
  
}
int main ( int argc, char** argv){
  ros::init (argc, argv, "lidar_outlier_picker");
  ros::NodeHandle nh;
  ros::Rate loop_rate(1);
  while (ros::ok()){
    ros::Subscriber lidar_sub = nh.subscribe("/scan", 1, LidarOutlierPickerCallback);
    loop_rate.sleep();
    ros::spinOnce();
  }
  
}