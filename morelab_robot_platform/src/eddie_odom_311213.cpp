/*
 * 144 encoder positions per revolution results in approximately .14" of linear travel accuracy
 * 14" = 0.3556 m
 * Encoder_max = 2147483648
 * Encoder_min = 2147483648
 *
*/
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <parallax_eddie_platform/DistanceWheel.h>
//
double cur_left_dis = 0.0;
double cur_right_dis = 0.0;
double old_left_dis = 0.0;
double old_right_dis = 0.0;
//
void distanceWheelCallback(const parallax_eddie_platform::DistanceWheel::ConstPtr& message)
{

    if (message->status.substr(0, 5) == "ERROR") // ERROR messages may be longer than 5 if in VERBOSE mode
    {
        ROS_ERROR("ERROR: Unable to read distance wheel data from encoders");
        return;
    }
    cur_left_dis = message->value[0];
    cur_right_dis = message->value[1];
    ROS_INFO("Odom: left = %f, right = %f", cur_left_dis, cur_right_dis);

}
//
int main(int argc, char** argv){
  ros::init(argc, argv, "eddie_odom");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);
  ros::Subscriber distance_wheel_sub = n.subscribe("/eddie/distance_wheel", 10, distanceWheelCallback);
  tf::TransformBroadcaster odom_broadcaster;

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

  double vx = 0.0;
  double vy = -0.1;
  double vth = 0.0;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(10.0); // 10 hz
  //
  while(n.ok()){

    current_time = ros::Time::now();

    // traveled distances for the left and right wheel
    d_left = (0.3556* (cur_left_dis - old_left_dis)/144);
    d_right = (0.3556* (cur_right_dis - old_right_dis)/144);
    ROS_INFO("d_left = %f, d_right = %f", d_left, d_right);

    old_left_dis = cur_left_dis;
    old_right_dis = cur_right_dis;

    // distance traveled is the average of the two wheels
    delta_s = ( d_left + d_right ) / 2;

    // this approximation works (in radians) for small angles
    delta_th = (d_right - d_left) / base_width;
    delta_x = (delta_s * cos(th + delta_th/2));
    delta_y = (delta_s * sin(th + delta_th/2));
    ROS_INFO("delta_x = %f, delta_y = %f, delta_th = %f", delta_x, delta_y, delta_th);

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();

    // calculate velocities
    vx = delta_s/dt;
    vth = delta_th/dt;

    ROS_INFO("x = %f, y = %f, th = %f", x, y, th);
    x += delta_x;
    y += delta_y;
    th += delta_th;


    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "base_footprint"; //
    odom_trans.child_frame_id = "odom_combined"; // base_link

    //odom_trans.header.frame_id = "odom_combined"; //
    //odom_trans.child_frame_id = "base_footprint"; // base_link

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    //odom_broadcaster.sendTransform(odom_trans);


    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    //odom.header.frame_id = "base_footprint"; //
    //odom.child_frame_id = "odom_combined"; //base_link

    odom.header.frame_id = "base_footprint"; //
    odom.child_frame_id = "odom_combined"; //base_link

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
    /*
    for(int i=0; i<36;i++)
    {
        odom.pose.covariance[i] = 0.01;
    }*/

    //set the velocity
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = vth;
    odom.twist.covariance = boost::array<double, 36>{{1e-3, 0, 0, 0, 0, 0,
                                                        0, 1e-3, 0, 0, 0, 0,
                                                        0, 0, 1e-3, 0, 0, 0,
                                                        0, 0, 0, 1e-3, 0, 0,
                                                        0, 0, 0, 0, 1e-3, 0,
                                                        0, 0, 0, 0, 0, 1e-3}};
    /*
    for(int i=0; i<36;i++)
    {
        odom.twist.covariance[1]=0.01;
    }
    */
    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;

    ros::spinOnce();
    r.sleep();
  }
}
