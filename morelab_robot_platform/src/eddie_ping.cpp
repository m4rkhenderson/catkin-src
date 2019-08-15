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

#include "eddie_ping.h"

EddiePing::EddiePing()
{
  ping_pub_ = node_handle_.advertise<morelab_robot_platform::Distances > ("/eddie/ping_distances", 1);
  stop_sliding_signal_ = node_handle_.advertise<morelab_robot_platform::StopSlidingSignal>("/sliding/stop_signal",1);
  ping_sub_ = node_handle_.subscribe("/eddie/ping_data", 1, &EddiePing::pingCallback, this);

}

void EddiePing::pingCallback(const morelab_robot_platform::Ping::ConstPtr& message)
{
  morelab_robot_platform::Distances distances;
  morelab_robot_platform::StopSlidingSignal stop_sliding;
  uint16_t d;
  if (message->status.substr(0, 5) == "ERROR") // ERROR messages may be longer than 5 if in VERBOSE mode
  {
    ROS_ERROR("ERROR: Unable to read Ping data from ping sensors");
    return;
  }
  for (uint i = 0; i < message->value.size(); i++)
  {
    //OTHER WAYS OF ENCODING THE DATA MAY BE DONE HERE.
    //DEFAULT DATA REPRESENTS DISTANCE IN MILLIMETERS
    d = message->value[i];
    ROS_INFO("PING % d", d);
    distances.value.push_back(d);
  }
  ping_pub_.publish(distances);
  // when sliding goes to the top
  if((message->value[2] < 50)&&(message->value[2] > 0))
  {
      stop_sliding.value = true;
  }
  else
  {
      stop_sliding.value = false;
  }
  stop_sliding_signal_.publish(stop_sliding);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "parallax_ping");
  EddiePing ping;
  ros::spin();

  return 0;
}
