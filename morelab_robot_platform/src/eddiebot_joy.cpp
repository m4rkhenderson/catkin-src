/* Copyright (c) 2010, Willow Garage Inc.
 * Copyright (c) 2012, Tang Tiong Yew
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <morelab_robot_platform/Velocity.h>
#include <sensor_msgs/Joy.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "ros/console.h"

class EddiebotTeleop
{
public:
  EddiebotTeleop();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void publish();

  ros::NodeHandle ph_, nh_;

  int linear_, angular_, deadman_axis_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

  morelab_robot_platform::Velocity last_published_;
  boost::mutex publish_mutex_;
  bool deadman_pressed_;
  ros::Timer timer_;

};

EddiebotTeleop::EddiebotTeleop():
  ph_("~"),
  linear_(1), // index
  angular_(0), // index
  deadman_axis_(5),
  l_scale_(1), //0.3
  a_scale_(1) //0.9
{
  ph_.param("axis_linear", linear_, linear_);
  ph_.param("axis_angular", angular_, angular_);
  ph_.param("axis_deadman", deadman_axis_, deadman_axis_);
  ph_.param("scale_angular", a_scale_, a_scale_);
  ph_.param("scale_linear", l_scale_, l_scale_);

  vel_pub_ = nh_.advertise<morelab_robot_platform::Velocity>("eddie/command_velocity", 1);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &EddiebotTeleop::joyCallback, this);

  timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&EddiebotTeleop::publish, this));
}

void EddiebotTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{ 
  morelab_robot_platform::Velocity vel;
  vel.angular = a_scale_*joy->axes[angular_]; // rotate
  vel.linear = l_scale_*joy->axes[linear_]; // linear
  last_published_ = vel;
  deadman_pressed_ = joy->buttons[deadman_axis_]; // enable of moving

  ROS_INFO("vel_angular = %f, vel_linear = %f, enable of moving = %d ",vel.angular, vel.linear, deadman_pressed_ );

}

void EddiebotTeleop::publish()
{
  boost::mutex::scoped_lock lock(publish_mutex_);

  //if (deadman_pressed_)
  //{
    vel_pub_.publish(last_published_);
  //}

}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "eddiebot_teleop_joy");
  EddiebotTeleop eddiebot_teleop;

  ros::spin();
}
