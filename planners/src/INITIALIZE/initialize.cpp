#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>

geometry_msgs::PoseWithCovarianceStamped init;
std::vector<geometry_msgs::PoseStamped> goal;
geometry_msgs::PoseStamped temp;
std::string nn;

double distance;
int k;
int NP;
int NP_01;
int NP_02;
int NP_03;
int NP_04;
int NP_05;
int NP_06;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "initialize");
  ros::NodeHandle nh;

  ros::Publisher init_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("robot_0/initialpose", 10);
  ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("robot_0/move_base_simple/goal", 10);

  nn = ros::this_node::getName();
  nh.param(nn + "/num_people", NP, 1);
  nh.param(nn + "/case_01_people", NP_01, 1);
  nh.param(nn + "/case_02_people", NP_02, 0);
  nh.param(nn + "/case_03_people", NP_03, 0);
  nh.param(nn + "/case_04_people", NP_04, 0);
  nh.param(nn + "/case_05_people", NP_05, 0);
  nh.param(nn + "/case_06_people", NP_06, 0);

  tf::TransformListener listener;
  tf::StampedTransform transform;

  ros::ServiceClient stage_client = nh.serviceClient<std_srvs::Empty>("reset_positions"); // for resetting the stage simulation
  ros::ServiceClient obstacle_client[NP];
  for(int i=1; i<=NP; i++){
    std::string p_id = std::to_string(i);
    obstacle_client[i-1] = nh.serviceClient<std_srvs::Empty>("robot_"+p_id+"/start_obstacle");
//    ROS_INFO("p_id: %s", ("/robot_"+p_id+"/start_obstacle").c_str());
  }
  std_srvs::Empty srv;

  ros::Rate r(10);

  init.header.frame_id = "map";
  temp.header.frame_id = "map";

  init.pose.pose.position.x = 27.0;
  init.pose.pose.position.y = -11.0;
  init.pose.pose.orientation.z = 3.14;
  init.pose.pose.orientation.w = 0.0;

  temp.pose.position.x = -2.0; // goal 1
  temp.pose.position.y = -11.0;
  temp.pose.orientation.z = 4.71;
  temp.pose.orientation.w = 1.0;

  goal.push_back(temp);

  temp.pose.position.x = -27.0; // goal 2
  temp.pose.position.y = -8.0;
  temp.pose.orientation.z = 4.71;
  temp.pose.orientation.w = 1.0;

  goal.push_back(temp);

  temp.pose.position.x = -24.0; // goal 3
  temp.pose.position.y = 10.0;
  temp.pose.orientation.z = 4.71;
  temp.pose.orientation.w = 1.0;

  goal.push_back(temp);

  temp.pose.position.x = -5.0; // goal 4
  temp.pose.position.y = 9.0;
  temp.pose.orientation.z = 4.71;
  temp.pose.orientation.w = 1.0;

  goal.push_back(temp);

  temp.pose.position.x = -5.0; // goal 5
  temp.pose.position.y = -7.0;
  temp.pose.orientation.z = 4.71;
  temp.pose.orientation.w = 1.0;

  goal.push_back(temp);

  temp.pose.position.x = 27.0; // goal 6
  temp.pose.position.y = -11.0;
  temp.pose.orientation.z = 4.71;
  temp.pose.orientation.w = 1.0;

  goal.push_back(temp);

  temp.pose.position.x = 27.0; // goal 7
  temp.pose.position.y = 11.0;
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


  for(int i=0; i<NP_01; i++){
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
      switch(k){
        case 1:
          goal_pub.publish(goal[k]);
          for(int i=NP_01; i<NP_01+NP_02; i++){
            obstacle_client[i].call(srv);
          }
          for(int i=0; i<NP_01; i++){
            obstacle_client[i].call(srv);
          }
          break;

        case 2:
          goal_pub.publish(goal[k]);
          for(int i=NP_01+NP_02; i<NP_01+NP_02+NP_03; i++){
            obstacle_client[i].call(srv);
          }
          for(int i=NP_01; i<NP_01+NP_02; i++){
            obstacle_client[i].call(srv);
          }
          break;

        case 3:
          goal_pub.publish(goal[k]);
          for(int i=NP_01+NP_02+NP_03; i<NP_01+NP_02+NP_03+NP_04; i++){
            obstacle_client[i].call(srv);
          }
          for(int i=NP_01+NP_02; i<NP_01+NP_02+NP_03; i++){
            obstacle_client[i].call(srv);
          }
          break;

        case 4:
          goal_pub.publish(goal[k]);
          for(int i=NP_01+NP_02+NP_03+NP_04; i<NP_01+NP_02+NP_03+NP_04+NP_05; i++){
            obstacle_client[i].call(srv);
          }
          for(int i=NP_01+NP_02+NP_03; i<NP_01+NP_02+NP_03+NP_04; i++){
            obstacle_client[i].call(srv);
          }
          break;

        case 5:
          goal_pub.publish(goal[k]);
          for(int i=NP_01+NP_02+NP_03+NP_04; i<NP_01+NP_02+NP_03+NP_04+NP_05; i++){
            obstacle_client[i].call(srv);
          }
          break;

        case 6:
          goal_pub.publish(goal[k]);
          for(int i=NP_01+NP_02+NP_03+NP_04+NP_05; i<NP_01+NP_02+NP_03+NP_04+NP_05+NP_06; i++){
            obstacle_client[i].call(srv);
          }
          break;

        case 7:
          k = 0;
          for(int i=NP_01+NP_02+NP_03+NP_04+NP_05; i<NP_01+NP_02+NP_03+NP_04+NP_05+NP_06; i++){
            obstacle_client[i].call(srv);
          }
          stage_client.call(srv); // reset robot positions
          init_pub.publish(init);
          goal_pub.publish(goal[k]);
          for(int i=0; i<NP_01; i++){
            obstacle_client[i].call(srv);
          }
          break;
      }
    }

    ros::spinOnce();
    r.sleep();
  }
}
