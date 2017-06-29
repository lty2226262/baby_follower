#include "human_tracker.h"

#define HUMAN_DEBUG




namespace humantracker {

HumanTracker::HumanTracker(ros::NodeHandle& node_handle): node_handle_(node_handle){
  node_handle_.param("lidar_frame", laser_frame_name_, std::string("/laser"));
  node_handle_.param("odom_frame", odom_frame_name_, std::string("/odom"));
  node_handle_.param("goal_frame", goal_frame_name_, std::string("/test/goal"));
  node_handle_.param("loop_rate", loop_rate_num_, 5.0);
  
  request_best_human_ = node_handle_.serviceClient<baby_follower::laser_detector_is_enabled>("detector_is_enabled");
  
  human_estimation_ = new HumanEstimation(goal_frame_name_, odom_frame_name_, node_handle_);
}

void HumanTracker::WaitInputForFiveSeconds()
{
  try{
    std::cin >> input_string_;
    cv_.notify_one();
  } catch (std::exception const&){}
}

void HumanTracker::SetDetectGlobalFrame(geometry_msgs::PointStamped input_point, geometry_msgs::PointStamped & output_point){
  input_point.header.stamp = ros::Time();
  while(ros::ok()){
    try{
      tf_listener_.transformPoint(odom_frame_name_, ros::Time(0), input_point, odom_frame_name_, output_point);
      break;
    } catch (std::exception const& ex){
      ROS_WARN_STREAM("unable to get tfs, have you launched lidar AND base description file?");
      ROS_ERROR_STREAM(ex.what());
      ros::Duration(2.0).sleep();
    }
  }
}

void HumanTracker::InitializationUserInterface()
{ 
  ROS_INFO_STREAM("HELLO, I AM THE LEGS OF XIAOBAO");
  ROS_INFO_STREAM("LET ME SEE IF THERE IS ANYONE STANDS IN FRONT OF ME");
  request_best_human_srv_.request.data = true;
  bool not_finished = true;
  
  // get the correct object loop
  while (not_finished){
    
    //service call loop
    while( !request_best_human_.call(request_best_human_srv_)){
      ROS_WARN_STREAM("sorry, fail to initialize, I'll try it again after 2 second");
      ros::Duration(2.0).sleep();
    }
    ROS_INFO_STREAM("Is this correct? (y/n)");
    
    // deal with timeout
    std::thread th(&HumanTracker::WaitInputForFiveSeconds, this);
    std::mutex mtx;
    std::unique_lock<std::mutex> lck(mtx);
    if (cv_.wait_for(lck, std::chrono::seconds(10)) == std::cv_status::timeout){
      ROS_INFO_STREAM("TIME OUT!!!!!!");
      not_finished = false;
      th.detach();
    } else{
      if(input_string_.compare("y") == 0){
	not_finished = false;
      } else if (input_string_.compare("n") == 0){
	not_finished = true;
      } else {
	not_finished = true;
	ROS_INFO_STREAM("Input y or n please");
      }
      th.join();  
    }
  }
  
  // Set goal_global_frame
  ros::Rate loop_rate(loop_rate_num_);
  geometry_msgs::PointStamped observed_goal_global_point;
  SetDetectGlobalFrame(request_best_human_srv_.response.msg, observed_goal_global_point);
  ros::Time filter_time = ros::Time::now();
  bool measure_availiable = true;

  // Have got the observed goal frame
  while (human_estimation_->update(observed_goal_global_point, filter_time, measure_availiable)){
    human_estimation_->measure(observed_goal_global_point, filter_time, measure_availiable, laser_frame_name_, odom_frame_name_);
    loop_rate.sleep();
  }
  
  
// #ifdef HUMAN_DEBUG
//   while(true){
//     ROS_INFO_STREAM("The target frame x is: " << observed_goal_global_point.point.x);
//     ROS_INFO_STREAM("The target frame y is: " << observed_goal_global_point.point.y);
//     ros::Duration(5.0).sleep();
//   }
// #endif
#ifdef HUMAN_DEBUG
  ROS_ERROR_STREAM("I've got it!");
#endif
 
}

HumanEstimation::HumanEstimation(const std::string goal_frame_name, const std::string odom_frame_name, ros::NodeHandle& node_handle): goal_frame_name_(goal_frame_name), odom_frame_name_(odom_frame_name), node_handle_(node_handle)
{
  node_handle_.param("min_leg", min_leg_, 0.07);
  node_handle_.param("max_leg", max_leg_, 0.29);
  //set system model
  MatrixWrapper::ColumnVector system_noise_mu(6);
  system_noise_mu = 0;
  MatrixWrapper::SymmetricMatrix system_noise_covariance(6);
  system_noise_covariance = 0;
  for (unsigned int i = 1; i <= 6; i++)
    system_noise_covariance(i,i) = pow(1000.0, 2);
  BFL::Gaussian system_uncertainty(system_noise_mu, system_noise_covariance);
  system_probability_density_function_ = new BFL::NonLinearAnalyticConditionalGaussianOdo(system_uncertainty);
  system_model_ = new BFL::AnalyticSystemModelGaussianUncertainty(system_probability_density_function_);
  //set measurement model
  MatrixWrapper::ColumnVector measure_noise_mu(2);
  measure_noise_mu = 0;
  MatrixWrapper::SymmetricMatrix measure_noise_covariance(2);
  measure_noise_covariance = 0;
  for (unsigned int i = 1; i <= 2; i++)
    measure_noise_covariance = 1;
  BFL::Gaussian measurement_uncertainty(measure_noise_mu, measure_noise_covariance);
  MatrixWrapper::Matrix H(2,6);
  H = 0;
  H(1,1) = 1;
  H(2,2) = 1;
  measure_probability_density_function_ = new BFL::LinearAnalyticConditionalGaussian(H, measurement_uncertainty);
  measure_model_ = new BFL::LinearAnalyticMeasurementModelGaussianUncertainty(measure_probability_density_function_);
  //set ros topic 
  velocity_converter_is_enabled_pub_ = node_handle_.advertise<std_msgs::Bool>("/velocity_converter_is_enabled",1);
  request_all_human_ = node_handle_.serviceClient<baby_follower::return_all_humans>("return_all_humans");
  request_all_human_srv_.request.data = true;
  msg_.data = true;
  //set varibles
  velocity_desi_ = MatrixWrapper::ColumnVector(2);
  velocity_desi_ = 0;
  measure_noise_covariance_ = MatrixWrapper::SymmetricMatrix(2);
  measure_noise_covariance_ = 0;
  distance_tolerance_ = 0.02;
  move_tolerance_ = 0.15;
  lost_tolerant_time_ = 3;
  distance_initialized_ = false;
}

HumanEstimation::~HumanEstimation(){
  if(filter_) delete filter_;
  if(prior_) delete prior_;
  delete system_model_;
  delete system_probability_density_function_;
  delete measure_model_;
  delete measure_probability_density_function_;
}

void HumanEstimation::Initialize(const geometry_msgs::PointStamped observed_goal){
  // set filter prior
  MatrixWrapper::ColumnVector prior_mu(6);
  prior_mu(1) = observed_goal.point.x;
  prior_mu(2) = observed_goal.point.y;
  MatrixWrapper::SymmetricMatrix prior_covariance(6);
  prior_covariance = 0;
  prior_covariance(1,1) = pow(0.001,2);
  prior_covariance(2,2) = pow(0.001,2);
  prior_covariance(6,6) = pow(3.1415926/4,2);
  prior_ = new BFL::Gaussian(prior_mu, prior_covariance);
  filter_ = new BFL::ExtendedKalmanFilter(prior_);
  
  // set the goal frame
  tf::Transform goal_transform;
  ros::Time goal_publish_time;
  goal_transform.setOrigin(tf::Vector3(observed_goal.point.x, observed_goal.point.y, 0.0));
  goal_transform.setRotation(tf::Quaternion(0,0,0,1));
  tf::StampedTransform goal_stamped_transform(goal_transform, goal_publish_time = ros::Time::now(), odom_frame_name_, goal_frame_name_);
// #ifdef HUMAN_DEBUG
//   ROS_INFO_STREAM("goal_transform:");
//   ROS_INFO_STREAM("             x:" << goal_transform.getOrigin().getX());
//   ROS_INFO_STREAM("             y:" << goal_transform.getOrigin().getY());
//   ROS_INFO_STREAM("           yaw:" << goal_transform.getRotation().getAngle());
// #endif
  tf_broadcaster_.sendTransform(goal_stamped_transform);
  velocity_converter_is_enabled_pub_.publish(msg_);
  
  // remember prior
  filter_estimate_old_vec_ = prior_mu;
  filter_estimate_old_ = goal_transform;
  filter_update_time_old_ = goal_publish_time;
  filter_time_old_ = goal_publish_time;
  
  // filter initialized
  filter_initialized_ = true;
  
  // measurement initialized
  measure_noise_covariance_(1,1) = pow(1, 2);
  measure_noise_covariance_(2,2) = pow(1, 2);
}

void HumanEstimation::measure(geometry_msgs::PointStamped& observed_goal, ros::Time& filter_time, bool& measure_availiable, std::string laser_frame_name, std::string odom_frame_name){
  if (!filter_initialized_){
    filter_time = ros::Time::now();
    measure_availiable = true;
    ROS_WARN_STREAM("Initialized complete by measure, maybe something wrong");
    return;
  } else{
    geometry_msgs::PointStamped local_goal;
    geometry_msgs::PointStamped last_global_goal;
    last_global_goal.header = observed_goal.header;
    last_global_goal.point.x = filter_estimate_old_.getOrigin().getX();
    last_global_goal.point.y = filter_estimate_old_.getOrigin().getY();
    last_global_goal.point.z = filter_estimate_old_.getOrigin().getZ();
    
    tf_listener_.transformPoint(laser_frame_name, ros::Time(0), last_global_goal, odom_frame_name_, local_goal);
    double local_theta = atan2(local_goal.point.y,local_goal.point.x);
    double local_rho = sqrt(pow(local_goal.point.x, 2) + pow(local_goal.point.y, 2));
    request_all_human_.call(request_all_human_srv_);
    std::vector<baby_follower::human> all_human = request_all_human_srv_.response.results.data;
    std::vector<double> distance_vector;
    double width;
    for (auto it = all_human.begin(); it!= all_human.end(); it++){
      width = tan(it->width/2) * it -> distance * 2;
      if ((min_leg_ >= width) || (max_leg_ <= width)){
	distance_vector.push_back(1.0/0.0);
      }
      else {
	distance_vector.push_back(sqrt(pow(it->distance,2) + pow(local_rho, 2) - 2 * local_rho * it->distance * cos(it -> position - local_theta)));
      }
    }
    
    std::vector<double>::iterator min_index_point = std::min_element(std::begin(distance_vector), std::end(distance_vector));
    int min_index = std::distance(std::begin(distance_vector), min_index_point);
    
    if (distance_vector[min_index] < distance_tolerance_){
      measure_availiable = true;
      if (!(distance_initialized_)){
// 	last_width_ = tan(all_human[min_index].width/2) * all_human[min_index].distance * 2;
	distance_initialized_ = true;
      }
// #ifdef HUMAN_DEBUG
//       ROS_INFO_STREAM("Do not move" << distance_vector[min_index]);
// #endif
    } else if (distance_vector[min_index] < move_tolerance_){
      measure_availiable = true;
      local_goal.point.x = all_human[min_index].distance * cos(all_human[min_index].position);
      local_goal.point.y = all_human[min_index].distance * sin(all_human[min_index].position);
      tf_listener_.transformPoint(odom_frame_name, ros::Time(0), local_goal, odom_frame_name_, observed_goal);
// #ifdef HUMAN_DEBUG
//       ROS_INFO_STREAM("move the distance of " << distance_vector[min_index]);
// #endif
    } else {
// #ifdef HUMAN_DEBUG
//       ROS_INFO_STREAM("remember width:" << last_width_ << ", this width : " << distance_vector[min_index]);
//       ROS_INFO_STREAM("begin: " << all_human[min_index].begin << ", end: " << all_human[min_index].end << ", distance: " << all_human[min_index].distance << ", width: " << all_human[min_index].width);
//       ROS_INFO_STREAM("min distance is:" << distance_vector[min_index]);
//       ROS_INFO_STREAM("local_rho" << local_rho<<"local_theta"<<local_theta);
//       ROS_INFO_STREAM("human_rho" << all_human[min_index].distance << "human_theta" << all_human[min_index].position);
// #endif
      measure_availiable = false;
    }
    
    
    filter_time = ros::Time::now();
  }
}

bool HumanEstimation::update(const geometry_msgs::PointStamped observed_goal, ros::Time filter_time, bool measure_availiable){
  if (!filter_initialized_){
    Initialize(observed_goal);
    ROS_INFO_STREAM("Initialized complete by update");
    return true;
  } else {
    double dt = ((ros::Duration)(filter_time - filter_time_old_)).toSec();
    if (dt == 0) return false;
    else if (dt < 0){
      ROS_INFO_STREAM("Could not update filter in the past");
      return false;
    } else{};
    
    // system update filter
    filter_->Update(system_model_, velocity_desi_);
    
    if (measure_availiable){
// #ifdef HUMAN_DEBUG
//       ROS_INFO_STREAM("dt is: " << dt);
// #endif
      measure_probability_density_function_->AdditiveNoiseSigmaSet(measure_noise_covariance_ * pow(dt, 2));
      MatrixWrapper::ColumnVector measure_vector(2);
      measure_vector(1) = observed_goal.point.x;
      measure_vector(2) = observed_goal.point.y;
      filter_->Update(measure_model_, measure_vector);
      filter_update_time_old_ = filter_time;
    } else{
      if (((ros::Duration)(filter_time - filter_update_time_old_)).toSec() > lost_tolerant_time_) {
	ROS_ERROR_STREAM("I AM LOST");
	distance_initialized_ = false;
	return false;
      }
    }
    
    // remember last estimate
    filter_estimate_old_vec_ = filter_->PostGet()->ExpectedValueGet();
    tf::Quaternion q;
    q.setRPY(filter_estimate_old_vec_(4), filter_estimate_old_vec_(5), filter_estimate_old_vec_(6));
    filter_estimate_old_ = tf::Transform(q, tf::Vector3(filter_estimate_old_vec_(1), filter_estimate_old_vec_(2), filter_estimate_old_vec_(3)));
    filter_time_old_ = filter_time;
    tf::StampedTransform goal_stamped_transform(filter_estimate_old_, filter_time, odom_frame_name_, goal_frame_name_);
    tf_broadcaster_.sendTransform(goal_stamped_transform);
#ifdef HUMAN_DEBUG
    tf::Transform observe_for_test(q, tf::Vector3(observed_goal.point.x, observed_goal.point.y, observed_goal.point.z));
    tf::StampedTransform obs_stamped_transform(observe_for_test, filter_time, odom_frame_name_, "obs_frame");
    tf_broadcaster_.sendTransform(obs_stamped_transform);
#endif
    velocity_converter_is_enabled_pub_.publish(msg_);
    
    return true;
  }
}
  
}

int main (int argc, char** argv){
  // Init the ROS node
  ros::init(argc, argv, "human_tracker");
  ros::NodeHandle node_handle;
  humantracker::HumanTracker human_tracker(node_handle);
  human_tracker.InitializationUserInterface();
}
