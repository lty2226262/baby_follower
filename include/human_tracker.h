#ifndef HUMAN_TRACKER_H
#define HUMAN_TRACKER_H

#include <ros/ros.h>
#include "baby_follower/laser_detector_is_enabled.h"
#include "baby_follower/return_all_humans.h"
#include "baby_follower/human_array.h"
#include "baby_follower/human.h"
#include <thread>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <bfl/filter/extendedkalmanfilter.h>
#include <bfl/wrappers/matrix/matrix_wrapper.h>
#include <bfl/model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <bfl/model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <bfl/pdf/analyticconditionalgaussian.h>
#include <bfl/pdf/linearanalyticconditionalgaussian.h>
#include "nonlinearanalyticconditionalgaussianodo.h"


namespace humantracker{

class HumanEstimation{
  BFL::AnalyticSystemModelGaussianUncertainty* system_model_;
  BFL::NonLinearAnalyticConditionalGaussianOdo* system_probability_density_function_;
  BFL::LinearAnalyticMeasurementModelGaussianUncertainty* measure_model_;
  BFL::LinearAnalyticConditionalGaussian* measure_probability_density_function_;
  BFL::Gaussian* prior_;
  BFL::ExtendedKalmanFilter* filter_;
  tf::Transformer transformer_;
  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster tf_broadcaster_;
  baby_follower::return_all_humans request_all_human_srv_;
  ros::ServiceClient request_all_human_; 
  
  MatrixWrapper::ColumnVector filter_estimate_old_vec_, velocity_desi_;
  MatrixWrapper::SymmetricMatrix measure_noise_covariance_;
  std::string goal_frame_name_, odom_frame_name_;
  ros::Time filter_time_old_, filter_update_time_old_;
  bool filter_initialized_, distance_initialized_;
  double lost_tolerant_time_, distance_tolerance_, move_tolerance_;
  
  ros::NodeHandle node_handle_;
  tf::Transform filter_estimate_old_;
  ros::Publisher velocity_converter_is_enabled_pub_;
  std_msgs::Bool msg_;
  double last_width_;
  
  //human related parameters
  double min_leg_, max_leg_;
  
  // Initialize the filter
  void Initialize(const geometry_msgs::PointStamped observed_goal);
  // If the filter is initialized
  bool IsInitialized(){return filter_initialized_;};

  
public:
  HumanEstimation(const std::string goal_frame_name, const std::string odom_frame_name, ros::NodeHandle& node_handle);
  ~HumanEstimation();
    // state update
  bool update(const geometry_msgs::PointStamped observed_goal, ros::Time filter_time, bool measure_availiable);
  // measure the object state
  void measure(geometry_msgs::PointStamped& observed_goal, ros::Time& filter_time, bool& measure_availiable, std::string laser_frame_name, std::string odom_frame_name);
};
  
class HumanTracker{
  // ros node and service
  ros::NodeHandle node_handle_;
  ros::ServiceClient request_best_human_; 
  tf::TransformListener tf_listener_;
  baby_follower::laser_detector_is_enabled request_best_human_srv_;
  
  // Detect points
  geometry_msgs::PointStamped observed_point_in_global_frame_;
  
  // Multiple thread
  std::string input_string_;
  std::condition_variable cv_;
  
  // Frame names
  std::string laser_frame_name_;
  std::string odom_frame_name_;
  std::string goal_frame_name_;
  
  // Publish results
  double loop_rate_num_;
  
  // EKF 
  HumanEstimation* human_estimation_;
  
  // Wait cin for five seconds
  void WaitInputForFiveSeconds();
  // Set goal global frame after detection
  void SetDetectGlobalFrame(geometry_msgs::PointStamped input_point, geometry_msgs::PointStamped & output_point);
  
public:
  HumanTracker(ros::NodeHandle& node_handle);
  void InitializationUserInterface();
};
}

#endif
