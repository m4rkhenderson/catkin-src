#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/console.h>
#include <std_srvs/Empty.h>
#include <string>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <geometry_msgs/Quaternion.h>

std::string ns;
std::string nn;

bool start = false;
geometry_msgs::Twist cmd_vel;
geometry_msgs::Twist zero;
sensor_msgs::LaserScan laser;
geometry_msgs::PoseStamped force;
double l_vel_max;
double a_vel_max;
double goal_x;
double goal_y;
double Fg_mag;
double Fp_mag;
double Fo_mag;
double Ft_mag;
double Fg_dir;
double Fp_dir;
double Fo_dir;
double Ft_dir;
double Ft_y;
double Ft_x;
std::vector<double> obs_mag;
double obs_mag_sum;
std::vector<double> obs_dir;
double obs_dir_sum;

double delta_t;
ros::WallTime time_c = ros::WallTime::now();
ros::WallTime time_p = ros::WallTime::now();
double d_m_c;
double d_m_p;
double d_r_c;
double d_r_p;
double d_o_avg_c;
double d_o_avg_p;
double ang_o_avg;
int cost;

double x_m;
double y_m;
double th_m;
double x_r;
double y_r;

int x_cells;
int y_cells;
int i_min;
int i_max;
int j_min;
int j_max;

double v_m;
double v_r;
double v_o;

bool startCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){
  start = !start;
//  ROS_INFO("movement toggled...");
  return true;
}

void laserCallback(const sensor_msgs::LaserScan& data){
  laser = data;
//  ROS_INFO("Got Laser Data");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "SFM");
  ros::NodeHandle nh;

  ns = ros::this_node::getNamespace();
  nn = ros::this_node::getName();

  ros::ServiceServer server = nh.advertiseService("start_obstacle", startCallback);
  ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::Publisher force_pub = nh.advertise<geometry_msgs::PoseStamped>("force", 1);
  ros::Subscriber laser_sub = nh.subscribe("base_scan", 1, laserCallback);

  nh.param(nn + "/goal_x", goal_x, 0.0);
  nh.param(nn + "/goal_y", goal_y, 0.0);
  nh.param(nn + "/linear_velocity_max", l_vel_max, 1.0);
  nh.param(nn + "/angular_velocity_max", a_vel_max, 1.0);

  tf::TransformListener listener;
  tf::StampedTransform transform_robot;
  tf::StampedTransform transform_map;

  zero.linear.x = 0.0;
  zero.linear.y = 0.0;
  zero.linear.z = 0.0;
  zero.angular.x = 0.0;
  zero.angular.y = 0.0;
  zero.angular.z = 0.0;

  ros::Rate r(10);
  while(nh.ok()){
    try {
      listener.lookupTransform("/robot_0/base_link", ns + "/base_link", ros::Time(0), transform_robot);
    }
    catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
    }
    try {
      listener.lookupTransform("/map", ns + "/base_link", ros::Time(0), transform_map);
    }
    catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
    }

    time_c = ros::WallTime::now();
    delta_t = time_c.toSec() - time_p.toSec();
    time_p = time_c;

    if(start == true){
      x_m = transform_map.getOrigin().x(); // get global origin
      y_m = transform_map.getOrigin().y();
      d_m_c = sqrt((x_m-goal_x)*(x_m-goal_x)+(y_m-goal_y)*(y_m-goal_y));
      th_m = tf::getYaw(transform_map.getRotation());

      v_m = (d_m_c-d_m_p)/delta_t;
      d_m_p = d_m_c;

      Fg_mag = (1/(1+exp(-1*v_m-2)))*(1/(1+exp(-1*d_m_c+5))); // pulls more when velocity becomes too positive, stops on approach to goal
      Fg_dir = atan2(y_m-goal_y,x_m-goal_x);
//      ROS_INFO("Magnitude: %f", Fg_mag);
//      ROS_INFO("Direction: %f", Fg_dir);

      x_r = transform_robot.getOrigin().x(); // get origin relative to robot
      y_r = transform_robot.getOrigin().y();
      d_r_c = sqrt(x_r*x_r+y_r*y_r);

      v_r = (d_r_c-d_r_p)/delta_t;
      d_r_p = d_r_c;
//      ROS_INFO("Velocity: %f", v_r);

      Fp_mag = (-1/(1+exp(-1*v_r+2))+1)*(-1/(1+exp(-1*d_r_c+6))+1); // begins to increase in strength when 3 meters away and with negative velocity
      Fp_dir = atan2(y_r,x_r);
//      ROS_INFO("Magnitude: %f", Fp_mag);
//      ROS_INFO("Direction: %f", Fp_dir);

      for(int i=0; i<laser.ranges.size(); i++){
        if(laser.ranges[i] >= laser.range_max || laser.ranges[i] <= laser.range_min){
          continue;
        }
        obs_mag.push_back(laser.ranges[i]);
        obs_dir.push_back(laser.angle_min+i*laser.angle_increment);
//        ROS_INFO("range: %f", laser.ranges[i]);
//        ROS_INFO("angle: %f", laser.angle_min+i*laser.angle_increment);
      }

      d_o_avg_c = 0.0;
      ang_o_avg = 0.0;
      if(obs_mag.size() > 0){
        for(int i=0; i<obs_mag.size(); i++){
          obs_mag_sum += obs_mag[i];
        }
        d_o_avg_c = obs_mag_sum/obs_mag.size();
        for(int i=0; i<obs_dir.size(); i++){
          obs_dir_sum += obs_dir[i];
        }
        ang_o_avg = obs_dir_sum/obs_dir.size();
      }
      obs_mag.clear();
      obs_dir.clear();

//      ROS_INFO("ObsMagSum: %f", obs_mag_sum);
//      ROS_INFO("ObsDirSum: %f", obs_dir_sum);
//      ROS_INFO("ObsMagAvg: %f", d_o_avg_c);
//      ROS_INFO("ObsDirAvg: %f", ang_o_avg);

      obs_mag_sum = 0.0;
      obs_dir_sum = 0.0;

      v_o = (d_o_avg_c-d_o_avg_p)/delta_t;
      d_o_avg_p = d_o_avg_c;

      Fo_mag = (-1/(1+exp(-1*v_o+2))+1)*(-1/(1+exp(-1*d_o_avg_c+10))+1);
      Fo_dir = ang_o_avg;
//      ROS_INFO("Magnitude: %f", Fo_mag);
//      ROS_INFO("Direction: %f", Fo_dir);

//      Ft_mag = (Fg_mag+Fp_mag+Fo_mag)/3;
//      Ft_dir = (Fg_dir-Fp_dir-Fo_dir)/3;
      Ft_x = (-1.85*Fg_mag*cos(Fg_dir-th_m)-0.75*Fp_mag*cos(Fp_dir-th_m)-0.4*Fo_mag*cos(Fo_dir-th_m))/3;
      Ft_y = (-1.85*Fg_mag*sin(Fg_dir-th_m)-0.75*Fp_mag*sin(Fp_dir-th_m)-0.4*Fo_mag*sin(Fo_dir-th_m))/3;
//      ROS_INFO("Magnitude: %f", Ft_mag);
//      ROS_INFO("Direction: %f", Ft_dir);
//      ROS_INFO("X: %f", Ft_x);
//      ROS_INFO("Y: %f", Ft_y);

      geometry_msgs::Quaternion quaternion;
      quaternion = tf::createQuaternionMsgFromYaw(atan2(Ft_y,Ft_x)+th_m);

      force.pose.position.x = x_m;
      force.pose.position.y = y_m;
      force.pose.position.z = 0.0;
      force.pose.orientation = quaternion;
      force.header.stamp = ros::Time::now();
      force.header.frame_id = "/map";

      // convert force to velocity
      cmd_vel.linear.x = l_vel_max*Ft_x;
      cmd_vel.linear.y = 0.0;
      cmd_vel.linear.z = 0.0;
      cmd_vel.angular.x = 0.0;
      cmd_vel.angular.y = 0.0;
      cmd_vel.angular.z = a_vel_max*Ft_y;

//      ROS_INFO("linear: %f", cmd_vel.linear.x);
//      ROS_INFO("angular: %f", cmd_vel.angular.z);
      force_pub.publish(force);
      cmd_pub.publish(cmd_vel);
    }
    else{
      cmd_pub.publish(zero);
//      ROS_INFO("zero");
    }
    ros::spinOnce();
    r.sleep();
  }
}
