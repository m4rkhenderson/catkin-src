#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <string.h>

nav_msgs::OccupancyGrid map;

void mapCallback(const nav_msgs::OccupancyGrid& data){
  map = data;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_remap");
  ros::NodeHandle nh;
  ros::Rate r(10);

  ros::Publisher map1_pub = nh.advertise<nav_msgs::OccupancyGrid>("robot_1/map", 10);
  ros::Publisher map2_pub = nh.advertise<nav_msgs::OccupancyGrid>("robot_2/map", 10);
  ros::Publisher map3_pub = nh.advertise<nav_msgs::OccupancyGrid>("robot_3/map", 10);
  ros::Publisher map4_pub = nh.advertise<nav_msgs::OccupancyGrid>("robot_4/map", 10);
  ros::Publisher map5_pub = nh.advertise<nav_msgs::OccupancyGrid>("robot_5/map", 10);
  ros::Publisher map6_pub = nh.advertise<nav_msgs::OccupancyGrid>("robot_6/map", 10);
  ros::Publisher map7_pub = nh.advertise<nav_msgs::OccupancyGrid>("robot_7/map", 10);
  ros::Publisher map8_pub = nh.advertise<nav_msgs::OccupancyGrid>("robot_8/map", 10);
  ros::Publisher map9_pub = nh.advertise<nav_msgs::OccupancyGrid>("robot_9/map", 10);
  ros::Subscriber map_sub = nh.subscribe("robot_0/move_base/global_costmap/costmap", 10, mapCallback);

  while(nh.ok()){
    map1_pub.publish(map);
    map2_pub.publish(map);
    map3_pub.publish(map);
    map4_pub.publish(map);
    map5_pub.publish(map);
    map6_pub.publish(map);
    map7_pub.publish(map);
    map8_pub.publish(map);
    map9_pub.publish(map);
    ros::spinOnce();
    r.sleep();
  }
}
