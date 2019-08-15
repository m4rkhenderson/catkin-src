/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Haikal Pribadi <haikal.pribadi@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of the Haikal Pribadi nor the names of other
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "eddie_controller.h"

EddieController::EddieController() :
  left_power_(30), right_power_(30), rotation_power_(80),
  acceleration_power_(30), deceleration_power_(100), min_power_(10),
  left_speed_(36), right_speed_(36), rotation_speed_(36), acceleration_speed_(36),
  linear_scale_(1.0), angular_scale_(1.0)
{

  velocity_keyboard_sub_ = node_handle_.subscribe("/eddie/command_velocity", 1, &EddieController::velocityCallbackKeyboard, this);
  velocity_sub_ = node_handle_.subscribe("cmd_vel", 1, &EddieController::velocityCallback, this);

  ping_distances_sub_ = node_handle_.subscribe("/eddie/ping_distances", 1, &EddieController::distanceCallback, this);
  ir_distances_sub_ = node_handle_.subscribe("/eddie/ir_voltages", 1, &EddieController::irCallback, this);

  encoder_sub_ = node_handle_.subscribe("/eddie/distance_wheel",1,&EddieController::encoderCallback, this);

  eddie_status_srv_ = node_handle_.advertiseService("emergency_status", &EddieController::getStatus, this);

  eddie_drive_power_ = node_handle_.serviceClient<morelab_robot_platform::DriveWithPower > ("drive_with_power");
  eddie_drive_speed_ = node_handle_.serviceClient<morelab_robot_platform::DriveWithSpeed > ("drive_with_speed");
  eddie_acceleration_rate_ = node_handle_.serviceClient<morelab_robot_platform::Accelerate > ("acceleration_rate");
  eddie_turn_ = node_handle_.serviceClient<morelab_robot_platform::Rotate > ("rotate");
  eddie_stop_ = node_handle_.serviceClient<morelab_robot_platform::StopAtDistance > ("stop_at_distance");
  eddie_heading_ = node_handle_.serviceClient<morelab_robot_platform::GetHeading > ("get_heading");
  eddie_reset_ = node_handle_.serviceClient<morelab_robot_platform::ResetEncoder > ("reset_encoder");

  node_handle_.param("left_power", left_power_, left_power_);
  node_handle_.param("right_power", right_power_, right_power_);
  node_handle_.param("rotation_power", rotation_power_, rotation_power_);
  node_handle_.param("acceleration_power", acceleration_power_, acceleration_power_);
  node_handle_.param("deceleration_power", deceleration_power_, deceleration_power_);
  node_handle_.param("min_power", min_power_, min_power_);
  node_handle_.param("left_speed", left_speed_, left_speed_);
  node_handle_.param("right_speed", right_speed_, right_speed_);
  node_handle_.param("rotation_speed", rotation_speed_, rotation_speed_);
  node_handle_.param("acceleration_speed", acceleration_speed_, acceleration_speed_);

  node_handle_.param("angular_scale", angular_scale_, angular_scale_);
  node_handle_.param("linear_scale", linear_scale_, linear_scale_);

  sem_init(&mutex_execute_, 0, 1);
  sem_init(&mutex_interrupt_, 0, 1);
  sem_init(&mutex_state_, 0, 1);
  sem_init(&mutex_ping_, 0, 1);

  sem_wait(&mutex_state_);
  current_power_ = 16;
  current_speed_ = 144;
  ping_distance_data_[0] = 4096;
  ping_distance_data_[1] = 4096;
  ir_distance_data_[0]= 0.0;
  ir_distance_data_[1]= 0.0;
  ir_distance_data_[2]= 0.0;
  ir_distance_data_[3]= 0.0;
  ir_distance_data_[4]= 0.0;

  number_of_rotate_left = 0;
  number_of_rotate_right = 0;

  cur_left_encoder_ = 0.0;
  cur_right_encoder_ = 0.0;
  old_left_encoder_ = 0.0;
  old_right_encoder_ =0.0;

  left_ = 0;
  right_ = 0;
  angular_ = 0;
  rotate_ = false;
  process_ = false;
  last_cmd_time_ = ros::Time::now();
  sem_post(&mutex_state_);
  sem_wait(&mutex_interrupt_);
  interrupt_ = false;
  sem_post(&mutex_interrupt_);

  setAccelerationRate(acceleration_speed_);
}
//
void EddieController::encoderCallback(const morelab_robot_platform::DistanceWheel::ConstPtr &message)
{
    if (message->status.substr(0, 5) == "ERROR") // ERROR messages may be longer than 5 if in VERBOSE mode
    {
        ROS_ERROR("ERROR: Unable to read distance wheel data from encoders");
        return;
    }

    cur_left_encoder_ = 0 - message->value[1];
    cur_right_encoder_ = 0 - message->value[0];
    //ROS_INFO("cur_left_encoder = %f, cur_right_encoder = %f", cur_left_encoder_, cur_right_encoder_);
}
/*
 * The information from navigation package
*/
void EddieController::velocityCallback(const geometry_msgs::TwistConstPtr & message)
{

    float linear = 1.0*(message->linear.x);
    float angular = 1.0*(message->angular.z);

    ROS_INFO("Linear = %f, Angular = %f", linear, angular);
    //moveLinearAngular(linear, angular);
    /* */
    if (linear == 0.0 && angular == 0.0)
    {
      stop();
      ROS_INFO("Stop Motor");
    }
    else
    {
      moveLinearAngular(linear, angular);
      ROS_INFO("Move Linear Angular");
    }

}
/*
 * The information from "eddie_teleop.cpp" or eddiebot_joy.cpp
*/
void EddieController::velocityCallbackKeyboard(const morelab_robot_platform::Velocity::ConstPtr& message)
{
  float linear = message->linear;
  float angular = message->angular;
  //ROS_INFO("Linear = %f, Angular = %f", linear, angular);

  if (linear == 0.0 && angular == 0.0)
  {
    stop();
    ROS_INFO("Stop Motor");
  }
  else
  {
    moveLinearAngular(linear, angular);
    ROS_INFO("Move Linear Angular");
  }
}
// From Ping sensors
void EddieController::distanceCallback(const morelab_robot_platform::Distances::ConstPtr& message)
{
  sem_wait(&mutex_ping_);
  bool okay = true;
  for (uint i = 0; i < message->value.size(); i++)
  {
      ping_distance_data_[i] = message->value[i];
      //ROS_INFO("PINGPING %d =  %d", i, message->value[i]);
      if (message->value[i] != 0 && message->value[i] < 300) // about mm
      {
          okay = false;
      }
  }
  ping_distances_okay_ = okay;
  if (!okay)
    stop();

  sem_post(&mutex_ping_);
}
// From IR sensors
void EddieController::irCallback(const morelab_robot_platform::Voltages::ConstPtr& message)
{
  sem_wait(&mutex_ir_);
  bool okay = true;

  for (uint i = 0; i < message->value.size(); i++)
  {
      ir_distance_data_[i] = message->value[i];
      //ROS_INFO("IR = %f", message->value[i]);
      if (message->value[i] != 0 && message->value[i] > 1.7) //1.7
      okay = false;
  }
  ir_distances_okay_ = okay;
  if (!okay)
    stop();
  sem_post(&mutex_ir_);
}
//
bool EddieController::getStatus(morelab_robot_platform::GetStatus::Request& req,
  morelab_robot_platform::GetStatus::Response& res)
{
  sem_wait(&mutex_ping_);
  sem_wait(&mutex_ir_);

  if (ping_distances_okay_ && ir_distances_okay_)
    res.okay = true;
  else
    res.okay = false;

  sem_post(&mutex_ping_);
  sem_post(&mutex_ir_);
  return true;
}
//
void EddieController::stop()
{
  morelab_robot_platform::StopAtDistance dist;
  dist.request.distance = 4; // stop after 4 encoder positions

  sem_wait(&mutex_interrupt_);
  interrupt_ = true;
  sem_post(&mutex_interrupt_);

  for (int i = 0; !eddie_stop_.call(dist) && i < 5; i++)
      ROS_ERROR("ERROR: at trying to stop Eddie - stop() function");

  left_ = 0;
  right_ = 0;
}
//
void EddieController::setAccelerationRate(int rate)
{
  morelab_robot_platform::Accelerate acc;
  acc.request.rate = acceleration_speed_;
  if (!eddie_acceleration_rate_.call(acc))
    ROS_ERROR("ERROR: Failed to set acceleration rate to %d", rate);
}
/// This program is used for Dynamic social zone paper
void EddieController::moveLinearAngular(float linear, float angular)
{
  int8_t left, right;
  float left_vel, right_vel;
  float width_robot = 0.4;

  if(linear == 0)
  {
      // turning
      right_vel = 10*angular * width_robot / 2.0;
      left_vel = (-1) * right_vel;
  }else if(angular == 0)
  {
      // forward / backward
      left_vel = right_vel = linear;
  }else
  {
      // moving doing arcs
      left_vel = linear - angular * width_robot / 2.0;
      right_vel = linear + angular * width_robot / 2.0;
  }
  left = clipPower(left_power_, left_vel);
  right = clipPower(right_power_, right_vel);

  sem_wait(&mutex_interrupt_);
  left_ = left;
  right_ = right;
  rotate_ = false;
  process_ = true;
  interrupt_ = true;
  sem_post(&mutex_interrupt_);
}
//
void EddieController::drive_with_power(int8_t left, int8_t right)
{
  sem_wait(&mutex_execute_);
  sem_wait(&mutex_interrupt_);
  interrupt_ = false;
  bool cancel = interrupt_;
  sem_post(&mutex_interrupt_);

  morelab_robot_platform::DriveWithPower power;
  ros::Time now;
  bool shift = true;

  while (ros::ok() && shift && !cancel)
  {
    now = ros::Time::now();
    if ((now.toSec() - last_cmd_time_.toSec()) >= 0.1)
    {
        power.request.left = left;
        power.request.right = right;  // ?
        if (eddie_drive_power_.call(power))
        {
            last_cmd_time_ = ros::Time::now();
            ROS_INFO("Current power move linear: left = %d, right = %d", power.request.left, power.request.right);
        }

        if (left != left_ && right != right_) // transfer the command when the values of left or right are different
          shift = true;
        else
          shift = false;
    }
    ros::spinOnce();
    usleep(1000);
    sem_wait(&mutex_interrupt_);
    cancel = interrupt_;
    sem_post(&mutex_interrupt_);
  }

  sem_post(&mutex_execute_);
}
// Use left_power_ and right_power_ for power_unit
int8_t EddieController::clipPower(int power_unit, float linear)
{
  int8_t power;
  if (power_unit * abs(linear) > 30)//127
  {
    if (linear > 0)
      power = 30;//127
    else
      power = -30;//-127
  }
  else
  {
    power = power_unit * linear;
  }
  // Chua check duoc cac truong hop cua phan nay. Neu chay khong ngon lam thi comment lai
  if((abs(power)<min_power_)&&(power!=0))
  {
      power =  (abs(power)/power)*min_power_;
      ROS_INFO("Threshold of the power  = %d", power);
  }

  return power;
}
//
void EddieController::execute()
{
  ros::Rate rate(1000);
  while (ros::ok())
  {
      //ROS_INFO("Eddie Controller While Loop Buc minh qua %d ", ros::ok());
    sem_wait(&mutex_interrupt_);
    bool ex = process_;
    sem_post(&mutex_interrupt_);
    if (ex)
    {
      //ROS_INFO("Eddie Controller if ex %d",ex);
      sem_wait(&mutex_interrupt_);
      process_ = false;
      int8_t l = left_;
      int8_t r = right_;
      sem_post(&mutex_interrupt_);
      drive_with_power(l, r);
      ROS_INFO("drive with power");
    }
    ros::spinOnce();
    rate.sleep();
  }
}
/*
 * 
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "eddie_controller");
  EddieController controller;
  controller.execute();

  return (EXIT_SUCCESS);
}

