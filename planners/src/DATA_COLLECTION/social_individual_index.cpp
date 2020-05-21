#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float64.h>

std_msgs::Float64 sii;
//double r_om = ; // no object interaction for now
double r_gk = 1.2; // some group interaction, could be variable but for now constant [0.45m, 1.2m] according to Hall's personal space criterion

void peopleCallback(const geometry_msgs::PoseArray& data){
  sii.data = 0.0;
  double sii_temp = 0.0;
  for(int i=0; i<data.poses.size(); i++){
    //ROS_INFO("x: %f, y: %f", data.poses[i].position.x, data.poses[i].position.y);
    sii_temp = exp(-((data.poses[i].position.x/(sqrt(2)*r_gk/2))*(data.poses[i].position.x/(sqrt(2)*r_gk/2))+
                     (data.poses[i].position.y/(sqrt(2)*r_gk/2))*(data.poses[i].position.y/(sqrt(2)*r_gk/2)))); // calculate the sii for this person
    //ROS_INFO("sii_temp: %f", sii_temp);
    if(sii_temp > sii.data){
      sii.data = sii_temp;
    }
  }
}

int main(int argc, char **argv){
  ros::init(argc,argv, "social_individual_index");
  ros::NodeHandle nh;
  ros::Rate r(10);

  ros::Publisher sii_pub = nh.advertise<std_msgs::Float64>("sii", 10);
  ros::Subscriber people_sub = nh.subscribe("people", 1, peopleCallback);

  while(nh.ok()){
    sii_pub.publish(sii);
    ros::spinOnce();
    r.sleep();
  }
}
