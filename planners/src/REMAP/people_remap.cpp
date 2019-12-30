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

//void peopleCallback(const nav_msgs::Odometry& data){
//  geometry_msgs::Pose person = data.pose.pose;
////  person.position.x += dx;
////  person.position.y += dy;
////  person.orientation.z += dz;
////  person.orientation.w += dw;
////  ROS_INFO("x:%f, y%f, z:%f, w:%f", person.position.x, person.position.y, person.orientation.z, person.orientation.w);
//  people.poses.push_back(person);
//  people.header.frame_id = "/robot_0/odom";
//}

//void robotCallback(const geometry_msgs::PoseWithCovarianceStamped& data){
//  person = data;
//  robot_cur = data.pose.pose;
//  if(robot_pre.position.x != robot_cur.position.x ||
//     robot_pre.position.y != robot_cur.position.y ||
//     robot_pre.orientation.z != robot_cur.orientation.z ||
//     robot_pre.orientation.w != robot_cur.orientation.w){
//    dx = robot_cur.position.x - robot_pre.position.x;
//    dy = robot_cur.position.y - robot_pre.position.y;
//    dz = robot_cur.orientation.z - robot_pre.orientation.z;
//    dw = robot_cur.orientation.w - robot_pre.orientation.w;
//    robot_pre = robot_cur;
//  }
//  flag = true;
//}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "people_remap");
  ros::NodeHandle nh;
  ros::Rate r(10);

  ros::Publisher people_pub = nh.advertise<geometry_msgs::PoseArray>("people", 10);
//  ros::Publisher initPose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/robot_1/initialpose", 10);
//  ros::Subscriber robot_sub = nh.subscribe("/robot_0/initialpose", 10, robotCallback);
//  ros::Subscriber person1_sub = nh.subscribe("/robot_1/odom", 10, peopleCallback);
//  ros::Subscriber person2_sub = nh.subscribe("/robot_2/odom", 10, peopleCallback);
//  ros::Subscriber person3_sub = nh.subscribe("/robot_3/odom", 10, peopleCallback);
//  ros::Subscriber person4_sub = nh.subscribe("/robot_4/odom", 10, peopleCallback);
//  ros::Subscriber person5_sub = nh.subscribe("/robot_5/odom", 10, peopleCallback);
//  ros::Subscriber person6_sub = nh.subscribe("/robot_6/odom", 10, peopleCallback);
  geometry_msgs::Pose person;

  tf::TransformListener listener;
  tf::StampedTransform transform_l_1;
  tf::StampedTransform transform_l_2;
  tf::StampedTransform transform_l_3;
  tf::StampedTransform transform_l_4;
  tf::StampedTransform transform_l_5;
  tf::StampedTransform transform_l_6;

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;

  transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  q.setRPY(0, 0, 0);
  transform.setRotation(q);

  while(nh.ok()){
//    if(flag == true){
//      initPose_pub.publish(person);
//    }

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/robot_0/odom", "/robot_1/odom"));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/robot_0/odom", "/robot_2/odom"));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/robot_0/odom", "/robot_3/odom"));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/robot_0/odom", "/robot_4/odom"));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/robot_0/odom", "/robot_5/odom"));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/robot_0/odom", "/robot_6/odom"));


    // get transform for robot_1
    try
    {
      listener.lookupTransform("/robot_0/base_link", "/robot_1/base_link", ros::Time(0), transform_l_1);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    person.position.x = transform_l_1.getOrigin().x();
    person.position.y = transform_l_1.getOrigin().y();
    person.orientation.z = transform_l_1.getRotation().z();
    person.orientation.w = transform_l_1.getRotation().w();
    ROS_INFO("x:%f, y%f, z:%f, w:%f", person.position.x, person.position.y, person.orientation.z, person.orientation.w);

    people.poses.push_back(person);


    // get transform for robot_2
    try
    {
      listener.lookupTransform("/robot_0/base_link", "/robot_2/base_link", ros::Time(0), transform_l_2);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    person.position.x = transform_l_2.getOrigin().x();
    person.position.y = transform_l_2.getOrigin().y();
    person.orientation.z = transform_l_2.getRotation().z();
    person.orientation.w = transform_l_2.getRotation().w();
    ROS_INFO("x:%f, y%f, z:%f, w:%f", person.position.x, person.position.y, person.orientation.z, person.orientation.w);

    people.poses.push_back(person);


    // get transform for robot_3
    try
    {
      listener.lookupTransform("/robot_0/base_link", "/robot_3/base_link", ros::Time(0), transform_l_3);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    person.position.x = transform_l_3.getOrigin().x();
    person.position.y = transform_l_3.getOrigin().y();
    person.orientation.z = transform_l_3.getRotation().z();
    person.orientation.w = transform_l_3.getRotation().w();
    ROS_INFO("x:%f, y%f, z:%f, w:%f", person.position.x, person.position.y, person.orientation.z, person.orientation.w);

    people.poses.push_back(person);


    // get transform for robot_4
    try
    {
      listener.lookupTransform("/robot_0/base_link", "/robot_4/base_link", ros::Time(0), transform_l_4);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    person.position.x = transform_l_4.getOrigin().x();
    person.position.y = transform_l_4.getOrigin().y();
    person.orientation.z = transform_l_4.getRotation().z();
    person.orientation.w = transform_l_4.getRotation().w();
    ROS_INFO("x:%f, y%f, z:%f, w:%f", person.position.x, person.position.y, person.orientation.z, person.orientation.w);

    people.poses.push_back(person);


    // get transform for robot_5
    try
    {
      listener.lookupTransform("/robot_0/base_link", "/robot_5/base_link", ros::Time(0), transform_l_5);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    person.position.x = transform_l_5.getOrigin().x();
    person.position.y = transform_l_5.getOrigin().y();
    person.orientation.z = transform_l_5.getRotation().z();
    person.orientation.w = transform_l_5.getRotation().w();
    ROS_INFO("x:%f, y%f, z:%f, w:%f", person.position.x, person.position.y, person.orientation.z, person.orientation.w);

    people.poses.push_back(person);


    // get transform for robot_6
    try
    {
      listener.lookupTransform("/robot_0/base_link", "/robot_6/base_link", ros::Time(0), transform_l_6);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    person.position.x = transform_l_6.getOrigin().x();
    person.position.y = transform_l_6.getOrigin().y();
    person.orientation.z = transform_l_6.getRotation().z();
    person.orientation.w = transform_l_6.getRotation().w();
    ROS_INFO("x:%f, y%f, z:%f, w:%f", person.position.x, person.position.y, person.orientation.z, person.orientation.w);


    people.poses.push_back(person);


    people.header.frame_id = "/robot_0/base_link";

    people_pub.publish(people);
    people.poses.clear();
    ros::spinOnce();
    r.sleep();
  }
}
