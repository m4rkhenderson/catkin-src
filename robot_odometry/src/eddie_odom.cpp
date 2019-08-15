/*
 * 144 encoder positions per revolution results in approximately .14" of linear travel accuracy
 * 14" = 0.3556 m
 * Encoder_max = 2147483648
 * Encoder_min = 2147483648
 *
*/
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <serial/serial.h>
//#include <morelab_robot_platform/DistanceWheel.h>
//
serial::Serial ser;
std::__cxx11::string enc;
int cur_left_dis = 0;
int cur_right_dis = 0;
int old_left_dis = 0;
int old_right_dis = 0;
const char encReq[5] = {'D','I','S','T','\r'};
char encL[4];
char encR[4];

//
//void distanceWheelCallback(const morelab_robot_platform::DistanceWheel::ConstPtr& message)
//{

//    if (message->status.substr(0, 5) == "ERROR") // ERROR messages may be longer than 5 if in VERBOSE mode
//    {
//        ROS_ERROR("ERROR: Unable to read distance wheel data from encoders");
//        return;
//    }
//    //cur_left_dis = 0 - message->value[0];
//    //cur_right_dis = 0 - message->value[1];

//    cur_left_dis = 0 - message->value[1];
//    cur_right_dis = 0 - message->value[0];
//    ROS_INFO("Odom: left = %f, right = %f", cur_left_dis, cur_right_dis);

//}
//
int main(int argc, char** argv){
  ros::init(argc, argv, "eddie_odom");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);
//  ros::Subscriber distance_wheel_sub = n.subscribe("/eddie/distance_wheel", 10, distanceWheelCallback);

  try
  {
      ser.setPort("/dev/ttyUSB0");
      ser.setBaudrate(115200);
      ser.setBytesize(serial::eightbits);
      ser.setParity(serial::parity_none);
      ser.setStopbits(serial::stopbits_one);
      ser.setFlowcontrol(serial::flowcontrol_none);
      serial::Timeout to = serial::Timeout::simpleTimeout(1000);
      ser.setTimeout(to);
      ser.open();
      ser.setRTS(true);
      ser.setDTR(true);
  }
  catch (serial::IOException& e)
  {
      ROS_ERROR_STREAM("Unable to open port ");
      return -1;
  }

  if(ser.isOpen()){
      ROS_INFO_STREAM("Serial Port initialized");
  }else{
      return -1;
  }

  double sticks_meter = 405; // The number of wheel encoder ticks per meter of travel
  double base_width = 0.39; // distance between the center of two wheels

  double d_left = 0.0;
  double d_right = 0.0;

  double delta_s = 0.0;
  double delta_x = 0.0;
  double delta_y = 0.0;
  double delta_th = 0.0;

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  double vx = 0.4;
  double vy = 0.0;
  double vth = 0.4;
  double v = 0.0;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(10); //hz (DSZ-20hz) (TROJ-stationary-10hz)

  tf::TransformBroadcaster odom_broadcaster;
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.frame_id = "odom";
  //odom_trans.header.frame_id = "world";
  odom_trans.child_frame_id = "base_footprint";

  //
  while(n.ok()){

    current_time = ros::Time::now();
    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();

    //request the current encoder values
    ser.write(encReq);
    ROS_INFO("REQUESTED ENCODER MEASUREMENTS");
    if(ser.available() > 7){
      enc = ser.read(ser.available());
      for(int i=0; i<4;i++){
          encR[i] = enc[3-i];
      }
      for(int i=0; i<4;i++){
          encL[i] = enc[7-i];
      }
      cur_left_dis = *(int *)encL;
      cur_right_dis = *(int *)encR;
      ROS_INFO("OBTAINED ENCODER MEASUREMENTS: %d, %d", cur_left_dis, cur_right_dis);
    }

    // traveled distances for the left and right wheel
    d_left = (0.495* (cur_left_dis - old_left_dis)/2800);
    d_right = (0.495* (cur_right_dis - old_right_dis)/2800);
    //ROS_INFO("d_left = %f, d_right = %f", d_left, d_right);

    old_left_dis = cur_left_dis;
    old_right_dis = cur_right_dis;

    // distance traveled is the average of the two wheels
    delta_s = ( d_left + d_right ) / 2;

    // this approximation works (in radians) for small angles
    delta_th = (d_right - d_left) / base_width;
    delta_x = (delta_s * cos(th + delta_th/2));
    delta_y = (delta_s * sin(th + delta_th/2));
    //ROS_INFO("delta_x = %f, delta_y = %f, delta_th = %f", delta_x, delta_y, delta_th);


    // calculate velocities
    vx = delta_x/dt;
    vy = delta_y/dt;
    vth = delta_th/dt;
    v = delta_s/dt;

    //ROS_INFO("x = %f, y = %f, th = %f", x, y, th);
    x += delta_x;
    y += delta_y;
    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,th);

    //first, we'll publish the transform over tf
    odom_trans.header.stamp = current_time;
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(th);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    //odom.header.frame_id = "world";
    odom.child_frame_id = "base_footprint";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    odom.pose.covariance = boost::array<double, 36>{{1e-3, 0, 0, 0, 0, 0,
                                                    0, 1e-3, 0, 0, 0, 0,
                                                    0, 0, 1e-3, 0, 0, 0,
                                                    0, 0, 0, 1e-3, 0, 0,
                                                    0, 0, 0, 0, 1e-3, 0,
                                                    0, 0, 0, 0, 0, 1e-3}};

    //set the velocity
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.linear.z = 0; // used for tcds paper
    //odom.twist.twist.linear.z = v; // used for dsz paper

    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = vth;
    odom.twist.covariance = boost::array<double, 36>{{1e-3, 0, 0, 0, 0, 0,
                                                        0, 1e-3, 0, 0, 0, 0,
                                                        0, 0, 1e-3, 0, 0, 0,
                                                        0, 0, 0, 1e-3, 0, 0,
                                                        0, 0, 0, 0, 1e-3, 0,
                                                        0, 0, 0, 0, 0, 1e-3}};
    last_time = current_time;
    //publish the message
    odom_broadcaster.sendTransform(odom_trans);
    odom_pub.publish(odom);

    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
