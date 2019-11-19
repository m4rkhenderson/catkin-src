#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

sensor_msgs::LaserScan scan;
std::string frame_id;
std::string nn;
double red_factor;
int num_scans;

void scanCallback(const sensor_msgs::LaserScan& data){
  scan = data;
  scan.header.frame_id = frame_id;
  num_scans = (scan.angle_max - scan.angle_min)/scan.angle_increment;
  for(int i=0; i<num_scans; i++){
    if(scan.ranges[i] - red_factor > scan.range_min){
      scan.ranges[i] = scan.ranges[i] - red_factor;
    }
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "clear_scan");
  ros::NodeHandle nh;

  ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("clear_scan", 10);
  ros::Subscriber scan_sub = nh.subscribe("base_scan", 10, scanCallback);

  nn = ros::this_node::getName();
  nh.param<std::string>(nn + "/laser_frame_id", frame_id, "laser");
  nh.param(nn + "/reduction_factor", red_factor, 0.5);

  ros::Rate r(10);

  while(nh.ok()){
    scan_pub.publish(scan);
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
