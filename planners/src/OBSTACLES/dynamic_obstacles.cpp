#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>

int counter = 0;
geometry_msgs::Twist cmd_vel;
double obstacle_velocity = 0.0;
int obstacle_steps = 10;
int obstacle_delay_1;
int obstacle_delay_2;
std::string ns;
std::string nn;

int main(int argc, char**argv){
  ros::init(argc, argv, "dynamic_obstacles");
  ros::NodeHandle nh;

  ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  ns = ros::this_node::getNamespace();
  nn = ros::this_node::getName();
  nh.param(nn + "/obstacle_velocity", obstacle_velocity, 1.0);
  nh.param(nn + "/obstacle_steps", obstacle_steps, 10);
  nh.param(nn + "/obstacle_delay_1", obstacle_delay_1, 0);
  nh.param(nn + "/obstacle_delay_2", obstacle_delay_2, 0);
  cmd_vel.linear.x = obstacle_velocity;

  ros::Rate r(10);

  for(int i=0; i<obstacle_delay_1; i++){
    for(int j=0; j<obstacle_delay_2; j++){
    }
  }

  while(nh.ok()){
    if(counter > obstacle_steps){
      cmd_vel.linear.x = -1*cmd_vel.linear.x;
      counter = 0;
    }
    counter++;
    cmd_pub.publish(cmd_vel);
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
