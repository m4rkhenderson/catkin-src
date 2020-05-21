#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

geometry_msgs::PoseArray people;
geometry_msgs::PoseWithCovarianceStamped person;
geometry_msgs::Pose robot_cur;
geometry_msgs::Pose robot_pre;
double dx = 0.0;
double dy = 0.0;
double dz = 0.0;
double dw = 0.0;
bool flag = false;

std::string nn;
int NP;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "people_remap");
  ros::NodeHandle nh;
  ros::Rate r(10);

  ros::Publisher people_pub = nh.advertise<geometry_msgs::PoseArray>("people", 10);
  geometry_msgs::Pose person;

  nn = ros::this_node::getName();
  nh.param(nn + "/num_people", NP, 1);

  tf::TransformListener listener;
  tf::StampedTransform transform_l;


  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;

  transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  q.setRPY(0, 0, 0);
  transform.setRotation(q);

  while(nh.ok()){

    for(int i=1; i<=NP; i++){
      std::string p_id = std::to_string(i);
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/robot_0/odom", "/robot_"+p_id+"/odom"));
      ROS_INFO("p_id: %s", ("/robot_"+p_id+"/odom").c_str());
    }

    for(int i=1; i<=NP; i++){
      std::string p_id = std::to_string(i);
      try
      {
        listener.lookupTransform("/robot_0/base_link", "/robot_"+p_id+"/base_link", ros::Time(0), transform_l);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
      }
      person.position.x = transform_l.getOrigin().x();
      person.position.y = transform_l.getOrigin().y();
      person.orientation.z = transform_l.getRotation().z();
      person.orientation.w = transform_l.getRotation().w();
      ROS_INFO("x:%f, y%f, z:%f, w:%f", person.position.x, person.position.y, person.orientation.z, person.orientation.w);

      people.poses.push_back(person);
    }

    people.header.frame_id = "/robot_0/base_link";

    people_pub.publish(people);
    people.poses.clear();
    ros::spinOnce();
    r.sleep();
  }
}
