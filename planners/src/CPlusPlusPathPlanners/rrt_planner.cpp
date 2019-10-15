#include <ros/ros.h>
#include <stdlib.h>
#include <array>

std::array<int,3> global;

int main(int argc, char** argv){
  ros::init(argc, argv, "c_rrtnode");
  ros::NodeHandle n;
  ros::Rate r(20);

  while(n.ok()){

  }

  return 0;
}
