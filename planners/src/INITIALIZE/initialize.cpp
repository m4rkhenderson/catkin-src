#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

geometry_msgs::PoseWithCovarianceStamped init;
std::vector<geometry_msgs::PoseStamped> goal;
geometry_msgs::PoseStamped temp;

double distance;
int k;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "initialize");
  ros::NodeHandle nh;

  ros::Publisher init_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("robot_0/initialpose", 10);
  ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("robot_0/move_base_simple/goal", 10);

  tf::TransformListener listener;
  tf::StampedTransform transform;

  ros::Rate r(10);

  init.header.frame_id = "map";
  temp.header.frame_id = "map";

  init.pose.pose.position.x = -16.0;
  init.pose.pose.position.y = -19.0;
  init.pose.pose.orientation.z = 1.0;
  init.pose.pose.orientation.w = 1.0;

  temp.pose.position.x = -16.0;
  temp.pose.position.y = -10.0;
  temp.pose.orientation.z = 4.71;
  temp.pose.orientation.w = 1.0;

  goal.push_back(temp);

  temp.pose.position.x = -2.0;
  temp.pose.position.y = -7.0;
  temp.pose.orientation.z = 4.71;
  temp.pose.orientation.w = 1.0;

  goal.push_back(temp);

  temp.pose.position.x = 2.0;
  temp.pose.position.y = 7.0;
  temp.pose.orientation.z = 4.71;
  temp.pose.orientation.w = 1.0;

  goal.push_back(temp);

  temp.pose.position.x = 16.0;
  temp.pose.position.y = 10.0;
  temp.pose.orientation.z = 4.71;
  temp.pose.orientation.w = 1.0;

  goal.push_back(temp);

  temp.pose.position.x = 16.5;
  temp.pose.position.y = 19.0;
  temp.pose.orientation.z = 4.71;
  temp.pose.orientation.w = 1.0;

  goal.push_back(temp);

  for(int i=0; i<1000; i++){
    for(int j=0; j<1000000; j++){
    }
  }
  init_pub.publish(init);
  for(int i=0; i<5000; i++){
    for(int j=0; j<1000000; j++){
    }
  }
  goal_pub.publish(goal[0]);

  ROS_INFO("Sent...");
  while(nh.ok()){
    listener.lookupTransform("/map", "robot_0/base_footprint", ros::Time(0), transform);
    distance = sqrt((transform.getOrigin().x() - goal[k].pose.position.x)*
                    (transform.getOrigin().x() - goal[k].pose.position.x)+
                    (transform.getOrigin().y() - goal[k].pose.position.y)*
                    (transform.getOrigin().y() - goal[k].pose.position.y));
    if(distance < 3.0){
      k++;
      if(k<5){
        goal_pub.publish(goal[k]);
      }
    }

    ros::spinOnce();
    r.sleep();
  }
}
