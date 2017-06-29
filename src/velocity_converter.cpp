
#include "velocity_converter.h"

#define VELO_DEBUG

namespace velocityconverter{
VelocityConverter::VelocityConverter(ros::NodeHandle& node_handle) : node_handle_(node_handle){ 
  // Get parameters
  node_handle_.param("base_frame", base_frame_, std::string("/base_link"));
  node_handle_.param("goal_frame",goal_frame_,std::string("/test/goal"));
  node_handle_.param("linear_tolerance", linear_tolerance_, 0.0);
  node_handle_.param("angular_tolerance", angular_tolerance_, 0.2618);
  node_handle_.param("follow_distance", follow_distance_, 0.5);
  node_handle_.param("linear_k_p", linear_pid.k_p , 0.1);
  node_handle_.param("linear_k_i", linear_pid.k_i , 0.0);
  node_handle_.param("linear_k_d", linear_pid.k_d , 0.0);
  node_handle_.param("angular_k_p", angular_pid.k_p , 2.10);
  node_handle_.param("angular_k_i", angular_pid.k_i , 1.381578);
  node_handle_.param("angular_k_d", angular_pid.k_d , 1.798);
  node_handle_.param("linear_speed_limit", linear_speed_limit_, 0.0);
  node_handle_.param("angular_speed_limit", angular_speed_limit_, 1.5);
  
  // Setup topics
  velocity_converter_is_enabled_ = node_handle_.subscribe("/velocity_converter_is_enabled", 1, &VelocityConverter::TrigCallback, this);
  cmd_vel_pub_ = node_handle_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  
}

void VelocityConverter::TrigCallback(const std_msgs::Bool& msg){
  if (msg.data == true){
    try{
      listener_.lookupTransform(base_frame_, goal_frame_, ros::Time(0), from_sensor_to_goal_);
    } catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(0.5).sleep();
    }
    current_linear_distance_ = from_sensor_to_goal_.getOrigin().length() - follow_distance_;
    current_angular_distance_y_ = from_sensor_to_goal_.getOrigin().getY();
    current_angular_distance_x_ = from_sensor_to_goal_.getOrigin().getX();
    current_angular_distance_ = tfNormalizeAngle(tfAtan2Fast(current_angular_distance_y_, current_angular_distance_x_));
    if (tfFabs(current_linear_distance_) <  linear_tolerance_) current_linear_distance_ = 0.0;
    if (tfFabs(current_angular_distance_) < angular_tolerance_) current_angular_distance_ = 0.0;
//#ifdef VELO_DEBUG
//    ROS_INFO_STREAM("linear distance" << current_linear_distance_);
//    ROS_INFO_STREAM("angular distance" << current_angular_distance_);
//#endif
    VelocityPIDCalculate(linear_pid, current_linear_distance_);
    VelocityPIDCalculate(angular_pid, current_angular_distance_);
#ifdef VELO_DEBUG
    ROS_INFO_STREAM("angular speed is(p):" << angular_pid.p_out);
    ROS_INFO_STREAM("angular speed is(i):" << angular_pid.i_out);
    ROS_INFO_STREAM("angular speed is(d):" << angular_pid.d_out);
    ROS_INFO_STREAM("angular speed is:" << angular_pid.total_out);
    ROS_INFO_STREAM("angular speed is(time):" << angular_pid.current_time - angular_pid.last_time);
    ROS_INFO_STREAM("angular error is:" << current_angular_distance_);
#endif
    VelocityUpdate(linear_pid.total_out, angular_pid.total_out);
  }
  else;
}


void VelocityConverter::VelocityPIDCalculate(VelocityPID& velocity_pid, tfScalar error){
  
  velocity_pid.current_time = ros::Time::now().toSec();
  
  // Proportional term
  velocity_pid.p_out = velocity_pid.k_p * (double)error;
  
  // Initialize check
  if (velocity_pid.last_time == -1){
    velocity_pid.last_time = velocity_pid.current_time;
    velocity_pid.pre_error = (double)error;
  } else {

    if (velocity_pid.current_time - velocity_pid.last_time < 0.1) return;
    
    // Integral term
    velocity_pid.integral += (double)error * (velocity_pid.current_time - velocity_pid.last_time);
    velocity_pid.i_out = velocity_pid.k_i * velocity_pid.integral;
    
    // Derivative term
    velocity_pid.derivative = ((double)error - velocity_pid.pre_error) / (velocity_pid.current_time - velocity_pid.last_time);
    velocity_pid.d_out = velocity_pid.k_d * velocity_pid.derivative;
#ifdef VELO_DEBUG
//    ROS_INFO_STREAM("current time" << velocity_pid.current_time);
//    ROS_INFO_STREAM("last time" << velocity_pid.last_time);
    ROS_INFO_STREAM("Duration: " << velocity_pid.current_time - velocity_pid.last_time);
//    ROS_INFO_STREAM("Error: " << error);
//    ROS_INFO_STREAM("Last_error: " << velocity_pid.pre_error);
#endif
     
    velocity_pid.last_time = velocity_pid.current_time;
    velocity_pid.pre_error = (double)error;
//     if (std::isnan(velocity_pid.d_out)) velocity_pid.d_out = 0.0;
    // Calculate the total output
    velocity_pid.total_out = velocity_pid.p_out + velocity_pid.i_out + velocity_pid.d_out;
  }
}

void VelocityConverter::VelocityUpdate(double linear_speed, double angular_speed) {
  geometry_msgs::Twist base_command;
//#ifdef VELO_DEBUG
//  ROS_INFO_STREAM("linear speed is(bef):" << linear_speed);
//#endif
  VelocityPositiveAndNegativeMaxCheck(linear_speed, linear_speed_limit_);
//#ifdef VELO_DEBUG
//    ROS_INFO_STREAM("linear speed is(aft):" << linear_speed);
//#endif
  base_command.linear.x = linear_speed;
  

  VelocityPositiveAndNegativeMaxCheck(angular_speed, angular_speed_limit_);
  base_command.angular.z = angular_speed;
  cmd_vel_pub_.publish(base_command);
}

void VelocityConverter::VelocityPositiveAndNegativeMaxCheck(double& speed, const double limit) {
  if (speed > limit) speed = limit;
  else if (speed < -limit) speed = -limit;
  else;
}

}

int main (int argc, char** argv){
  // Init the ROS node
  ros::init(argc, argv, "velocity_converter_for_follower");
  ros::NodeHandle nh;
  velocityconverter::VelocityConverter vel_cvt(nh);
  ros::spin();
}
