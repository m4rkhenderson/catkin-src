#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>
#include <string>

std::string ns;
std::string nn;
double x_pos;
double y_pos;
bool start = false;
int count = 0;
geometry_msgs::PoseStamped goal;

bool callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){
  start = !start;
  count = 0;
  ROS_INFO("movement activated");
  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "smart_obstacle_goal");
  ros::NodeHandle nh;

  ns = ros::this_node::getNamespace();
  nn = ros::this_node::getName();

  ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>(ns + "/move_base_simple/goal", 10);
  ros::ServiceServer server = nh.advertiseService("start_obstacle", callback);

  nh.param(nn + "/x_pos", x_pos, 0.0);
  nh.param(nn + "/y_pos", y_pos, 0.0);

  goal.pose.position.x = x_pos;
  goal.pose.position.y = y_pos;
  goal.pose.orientation.z = 0.0; /// arbitrary
  goal.pose.orientation.w = 1; /////////////

  goal.header.frame_id = "map";

  ros::Rate r(10);

  while(nh.ok()){
    if(start == true && count < 1){
      ROS_INFO("goal is sent...");
      goal_pub.publish(goal);
      count++;
    }
    ros::spinOnce();
    r.sleep();
  }
}
