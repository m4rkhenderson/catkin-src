#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <pedsim_msgs/AgentStates.h>

geometry_msgs::PoseArray people;
geometry_msgs::PoseArray agents;
geometry_msgs::Pose robot;

std::string nn;
int NP;

void agentCallback(const pedsim_msgs::AgentStates& data){
  for(int i=0; i< data.agent_states.size(); i++){
    agents.poses.push_back(data.agent_states[i].pose);
  }
}

void robotCallback(const nav_msgs::Odometry& data){
  robot = data.pose.pose;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pedsim_remap");
  ros::NodeHandle nh;
  ros::Rate r(20);

  ros::Publisher people_pub = nh.advertise<geometry_msgs::PoseArray>("people", 10);
  ros::Subscriber agent_sub = nh.subscribe("pedsim_simulator/simulated_agents", 10, agentCallback);
  ros::Subscriber robot_sub = nh.subscribe("odom", 10, robotCallback);
  geometry_msgs::Pose person;

  tf::Quaternion q1, q2, q3;
  double d, th, roll, pitch, yaw;
  q1.setRPY(0, 0, 0);
  q2.setRPY(0, 0, 0);
  q3.setRPY(0, 0, 0);

  while(nh.ok()){

    for(int i=0; i<agents.poses.size(); i++){
      q1 = {agents.poses[i].orientation.x,
            agents.poses[i].orientation.y,
            agents.poses[i].orientation.z,
            agents.poses[i].orientation.w};
      q2 = {robot.orientation.x,
            robot.orientation.y,
            robot.orientation.z,
            robot.orientation.w};
      q3 = q1 * q2.inverse(); // quaternion subtraction
      tf::Matrix3x3(q2).getRPY(roll, pitch, yaw);
      d = sqrt((robot.position.x - agents.poses[i].position.x)*
                      (robot.position.x - agents.poses[i].position.x)+
                      (robot.position.y - agents.poses[i].position.y)*
                      (robot.position.y - agents.poses[i].position.y));
      th = atan2((robot.position.y - agents.poses[i].position.y),(robot.position.x - agents.poses[i].position.x));
      person.position.x = -d*cos(th - yaw);
      person.position.y = -d*sin(th - yaw);
      person.orientation.z = q3.z();
      person.orientation.w = q3.w();
      //ROS_INFO("x:%f, y%f, z:%f, w:%f", person.position.x, person.position.y, person.orientation.z, person.orientation.w);

      people.poses.push_back(person);
    }

    people.header.frame_id = "/base_link";

    people_pub.publish(people);
    people.poses.clear();
    agents.poses.clear();
    ros::spinOnce();
    r.sleep();
  }
}
