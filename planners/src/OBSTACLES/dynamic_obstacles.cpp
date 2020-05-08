#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
#include <string>
#include <tf/transform_listener.h>

bool start = false;
int counter = 0;
geometry_msgs::Twist cmd_vel;
geometry_msgs::Twist cmd_vel_c;
geometry_msgs::Twist zero;
double obstacle_velocity = 0.0;
int obstacle_steps = 10;
int obstacle_delay_1;
int obstacle_delay_2;
std::string ns;
std::string nn;

int num;

double per_m;

bool callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){
  counter = 0;
  start = !start;
  if(cmd_vel_c.linear.x < 0){
    cmd_vel_c.linear.x = -1*cmd_vel_c.linear.x;
  }
  ROS_INFO("movement activated");
  return true;
}

int main(int argc, char**argv){
  ros::init(argc, argv, "dynamic_obstacles");
  ros::NodeHandle nh;

  ros::ServiceServer server = nh.advertiseService("start_obstacle", callback);
  ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  ns = ros::this_node::getNamespace();
  //ROS_INFO("%s", ns.c_str());
  if(ns.size() < 9){ // figure out which robot this is
    num = std::atoi(&ns[7])-1; // -1 because it needs to be zero indexed
    //ROS_INFO("%d", num);
  }
  else{
    num = std::atoi(&ns[8])+9;
  }

  tf::TransformListener listener;
  tf::StampedTransform transform;

  nn = ros::this_node::getName();
  nh.param(nn + "/obstacle_velocity", obstacle_velocity, 1.0);
  nh.param(nn + "/obstacle_steps", obstacle_steps, 10);
  nh.param(nn + "/obstacle_delay_1", obstacle_delay_1, 0);
  nh.param(nn + "/obstacle_delay_2", obstacle_delay_2, 0);
  cmd_vel_c.linear.x = obstacle_velocity;
  zero.linear.x = 0.0;

  ros::Rate r(10);

  for(int i=0; i<obstacle_delay_1; i++){
    for(int j=0; j<obstacle_delay_2; j++){
    }
  }

  ROS_INFO("Recieved Position Data");

  while(nh.ok()){
    try {
      listener.lookupTransform("/robot_0/base_link", ns + "/base_link", ros::Time(0), transform);
    }
    catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
    }
    per_m = sqrt(transform.getOrigin().x()*transform.getOrigin().x()+
                 transform.getOrigin().y()*transform.getOrigin().y());

    cmd_vel.linear.x = cmd_vel_c.linear.x*1/(1+exp(-1*(per_m-5.0)/2.0)); // slow down robot when approaching robot_0

    if(start == true){
      if(counter > obstacle_steps){
        cmd_vel_c.linear.x = -1*cmd_vel_c.linear.x;
        counter = 0;
      }
      counter++;
      cmd_pub.publish(cmd_vel);
    }
    else{
      cmd_pub.publish(zero);
    }
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
