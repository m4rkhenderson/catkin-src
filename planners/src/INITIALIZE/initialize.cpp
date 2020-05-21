#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <vector>

geometry_msgs::PoseWithCovarianceStamped init;
std::vector<geometry_msgs::PoseStamped> goal;
geometry_msgs::PoseStamped temp;
std::string nn;

double distance;
int k;
int NP;
std::vector<int> CP;
int NC;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "initialize");
  ros::NodeHandle nh;
  ros::Rate r(10);

  ros::Publisher init_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("robot_0/initialpose", 10);
  ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("robot_0/move_base_simple/goal", 10);

  init.header.frame_id = "map";
  temp.header.frame_id = "map";

  tf::TransformListener listener;
  tf::StampedTransform transform;

  nn = ros::this_node::getName();
  nh.param(nn + "/num_people", NP, 1);
  nh.param(nn + "/num_cases", NC, 1);
  for(int i=0; i<NC; i++){
    int CP_temp;
    std::string c_id = std::to_string(i);

    nh.param(nn + "/case_" + c_id + "_people", CP_temp, 0); // activate parameters for given case number
    CP.push_back(CP_temp);

    nh.param(nn + "/goal_" + c_id + "_x", temp.pose.position.x, 0.0); // activate parameters for case goal
    nh.param(nn + "/goal_" + c_id + "_y", temp.pose.position.y, 0.0);
    temp.pose.orientation.z = 4.71;
    temp.pose.orientation.w = 1.0;
    goal.push_back(temp); // add case goal to stored goals
  }
  nh.param(nn + "/init_x", init.pose.pose.position.x, 0.0); // get parameters for robot initial pose
  nh.param(nn + "/init_y", init.pose.pose.position.y, 0.0);
  nh.param(nn + "/init_z", init.pose.pose.orientation.z, 0.0);
  nh.param(nn + "/init_w", init.pose.pose.orientation.w, 0.0);

  ros::ServiceClient stage_client = nh.serviceClient<std_srvs::Empty>("reset_positions"); // for resetting the stage simulation
  ros::ServiceClient obstacle_client[NP];
  for(int i=1; i<=NP; i++){
    std::string p_id = std::to_string(i);
    obstacle_client[i-1] = nh.serviceClient<std_srvs::Empty>("robot_"+p_id+"/start_obstacle");
//    ROS_INFO("p_id: %s", ("/robot_"+p_id+"/start_obstacle").c_str());
  }
  std_srvs::Empty srv;

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

  for(int i=0; i<CP[0]; i++){ // start first obstacle set
    obstacle_client[i].call(srv);
  }

  ROS_INFO("Sent...");


  while(nh.ok()){
    listener.lookupTransform("/map", "robot_0/base_footprint", ros::Time(0), transform);
    distance = sqrt((transform.getOrigin().x() - goal[k].pose.position.x)*
                    (transform.getOrigin().x() - goal[k].pose.position.x)+
                    (transform.getOrigin().y() - goal[k].pose.position.y)*
                    (transform.getOrigin().y() - goal[k].pose.position.y));
    if(distance < 4.0){
      k++;
      int CP_t = 0;
      if(k<NC){
        goal_pub.publish(goal[k]); // publish k-case goal
        for(int i=0; i<k; i++){
          CP_t += CP[i];
        }
        for(int i=CP_t-CP[k]-CP[k-1]; i<CP_t-CP[k]; i++){ // stop previous obstacles
          obstacle_client[i].call(srv);
        }
        for(int i=CP_t-CP[k]; i<CP_t; i++){ // start new obstacles
          obstacle_client[i].call(srv);
        }
      }
      else{
        k=0;
        for(int i=0; i<NC; i++){
          CP_t += CP[i];
        }
        for(int i=CP_t-CP[NC-1]; i<CP_t; i++){ // note NC is not zero-indexed, stop last obstacle
          obstacle_client[i].call(srv);
        }
        stage_client.call(srv); // reset robot positions
        init_pub.publish(init); // re-initialize robot pose
        goal_pub.publish(goal[k]); // publish k-case goal
        for(int i=0; i<CP[k]; i++){ // start first obstacle set
          obstacle_client[i].call(srv);
        }
      }
    }

    ros::spinOnce();
    r.sleep();
  }
}
