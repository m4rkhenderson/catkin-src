#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <std_msgs/Float64.h>


std::string nn;
std::string costmap_topic_;
std::string base_frame_;
std_msgs::Float64 obstacle_dist_;
double prev_o_dist_;
double x_;
double y_;

void costmapCallback(const nav_msgs::OccupancyGrid& data){
  double min_d = 6.0;
  double pos_d = 0;
  for(unsigned int i=0; i<data.info.width; i++){
    for(unsigned int j=0; j<data.info.height; j++){
      if(data.data[i + j] > 0){
        pos_d = sqrt((x_ - (i*data.info.resolution + data.info.origin.position.x))*
                     (x_ - (i*data.info.resolution + data.info.origin.position.x))+
                     (y_ - (j*data.info.resolution + data.info.origin.position.y))*
                     (y_ - (j*data.info.resolution + data.info.origin.position.y)));
        if(pos_d < min_d){
          min_d = pos_d;
        }
      }
    }
  }
  obstacle_dist_.data = min_d;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "obstacle_distance");
  ros::NodeHandle nh;

  ros::Rate r(10);

  nn = ros::this_node::getName();
  nh.param<std::string>(nn + "/costmap_topic", costmap_topic_, "move_base/local_costmap/costmap");
  nh.param<std::string>(nn + "/base_frame", base_frame_, "/base_footprint");

  ros::Publisher distance_pub = nh.advertise<std_msgs::Float64>("obstacle_dist", 10);
  ros::Subscriber costmap_sub = nh.subscribe(costmap_topic_, 10, costmapCallback);

  tf::TransformListener listener;
  tf::StampedTransform transform;

  while(nh.ok()){
    ros::Time now = ros::Time::now();
    listener.waitForTransform("/map", base_frame_, now, ros::Duration(1.0));
    listener.lookupTransform("/map", base_frame_, now, transform);
    x_ = transform.getOrigin().x();
    y_ = transform.getOrigin().y();
    if(obstacle_dist_.data != prev_o_dist_){
      distance_pub.publish(obstacle_dist_);
      ROS_INFO("Distance to Nearest Obstacle: %f", obstacle_dist_.data);
      prev_o_dist_ = obstacle_dist_.data;
    }
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
