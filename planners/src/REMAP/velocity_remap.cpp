#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

geometry_msgs::Twist vel;

void velCallback(const geometry_msgs::Twist& data){
  vel = data;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "velocity_remap");
  ros::NodeHandle nh;
  ros::Rate r(20);

  ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("pedbot/control/cmd_vel", 10);
  ros::Subscriber vel_sub = nh.subscribe("robot_0/cmd_vel", 10, velCallback);

  while(nh.ok()){
    vel_pub.publish(vel);
    ros::spinOnce();
    r.sleep();
  }
}
