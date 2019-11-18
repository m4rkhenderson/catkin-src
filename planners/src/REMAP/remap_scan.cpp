#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

sensor_msgs::LaserScan scan;

void scanCallback(const sensor_msgs::LaserScan& data){
  scan = data;
  scan.header.frame_id = "robot_0/base_laser_link";
}

int main(int argc, char** argv){
  ros::init(argc, argv, "remap_scan");
  ros::NodeHandle nh;

  ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("base_scan", 10);
  ros::Subscriber scan_sub = nh.subscribe("/robot_0/base_scan", 10, scanCallback);

  ros::Rate r(10);

  while(nh.ok()){
    scan_pub.publish(scan);
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
