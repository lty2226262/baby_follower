#ifndef VELOCITY_CONVERTER_H
#define VELOCITY_CONVERTER_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Bool.h>

namespace velocityconverter{

//Struct for pid control
struct VelocityPID{
  double k_p, k_i, k_d;
  double error, pre_error;
  double integral = 0;
  double p_out, i_out, d_out;
  double derivative;
  double current_time, last_time = -1;
  double total_out;
};  

//from goal to velocity
class VelocityConverter{
  ros::NodeHandle node_handle_;
  
  //Parameters fetch from parameter server
  std::string base_frame_, goal_frame_;
  double linear_tolerance_, angular_tolerance_, follow_distance_;
  double linear_speed_limit_, angular_speed_limit_;
  
  
  //Listeners, subscribers and publishers
  tf::TransformListener listener_;
  ros::Subscriber velocity_converter_is_enabled_;
  ros::Publisher cmd_vel_pub_;
  
  //Variables for distance calculation
  tf::StampedTransform from_sensor_to_goal_;
  tfScalar current_linear_distance_, current_angular_distance_, current_angular_distance_y_, current_angular_distance_x_;
  
  //Variables for pid control
  VelocityPID linear_pid, angular_pid;
  
  //Trig callback of velocity converter
  void TrigCallback(const std_msgs::Bool& msg);
  //Compute using PID 
  void VelocityPIDCalculate(VelocityPID& velocity_pid, tfScalar error);
  //Compute velocity using PID results 
  void VelocityUpdate(double linear_speed, double angular_speed);
  //A subfunction of velocity update
  void VelocityPositiveAndNegativeMaxCheck(double& speed, const double limit);

public:
  VelocityConverter(ros::NodeHandle& node_handle);
};


}

#endif //VELOCITY_CONVERTER_H