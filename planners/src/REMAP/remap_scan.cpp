#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <string>

sensor_msgs::LaserScan scan;
std::string nn;
std::string ns;

void scanCallback(const sensor_msgs::LaserScan& data){
  scan = data;
  scan.header.frame_id = ns + "/base_laser_link";
}

int main(int argc, char** argv){
  ros::init(argc, argv, "remap_scan");
  ros::NodeHandle nh;

  ns = ros::this_node::getNamespace();
  ns.erase(ns.begin());
  //nn = ros::this_node::getName();

  ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("laser_scan", 10);
  ros::Subscriber scan_sub = nh.subscribe("base_scan", 10, scanCallback);

  ros::Rate r(10);

  while(nh.ok()){
    scan_pub.publish(scan);
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
