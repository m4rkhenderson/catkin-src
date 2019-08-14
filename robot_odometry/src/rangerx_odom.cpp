/*
Wheel Diameter: 13" or 33.02cm
Distance Per Revolution: pi*diameter = 40.8" or 103.74cm
Encoder Resolution: 20480p/r or 0.00199"/p or 0.005065cm/p
*/
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>

double x = 0.0;
double y = 0.0;
double th = 0.0;

void xCallback(const std_msgs::Float32& x_pos){
    x  = x_pos.data;
}
void yCallback(const std_msgs::Float32& y_pos){
    y = y_pos.data;
}
void thCallback(const std_msgs::Float32& th_pos){
    th = th_pos.data;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "rangerx_odom");

    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);
    ros::Subscriber x_sub = n.subscribe("/ARDU_X", 10, xCallback);
    ros::Subscriber y_sub = n.subscribe("/ARDU_Y", 10, yCallback);
    ros::Subscriber th_sub = n.subscribe("/ARDU_TH", 10, thCallback);


    double pulse_meter = 19279; //The number of encoder pulses per 1 meter of travel // note: actually somewhat different
    double base_width = 0.56; // The distance between the center of the two wheels in meters

    double d_left = 0.0;
    double d_right = 0.0;

    double delta_s = 0.0;
    double delta_x = 0.0;
    double delta_y = 0.0;
    double delta_th = 0.0;

    //double x = 0.0;
    double x_p = 0.0;
    //double y = 0.0;
    double y_p = 0.0;
    //double th = 0.0;
    double th_p = 0.0;

    double vx = 0.0;
    double vy = 0.0;
    double vth = 0.0;
    double v = 0.0;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::Rate r(10); //hz

    tf::TransformBroadcaster odom_broadcaster;
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";

    while(n.ok()){

        current_time = ros::Time::now();
        double dt = (current_time - last_time).toSec();

        delta_x = x - x_p;
        x_p = x;
        delta_y = y - y_p;
        y_p = y;
        delta_th = th - th_p;
        th_p = th;

        // calculate velocities
        ROS_INFO("dt = %f",dt);
        vx = delta_x/dt;
        vy = delta_y/dt;
        vth = delta_th/dt;
        v = vx*sin(0.785);

        ROS_INFO("x = %f, y = %f, th = %f", x, y, th);

        // since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,th);

        // first, we'll publish the transform over tf
        odom_trans.header.stamp = current_time;
        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(th);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
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
        odom.twist.twist.linear.x = v;//vx;
        odom.twist.twist.linear.y = 0.0;//vy;
        odom.twist.twist.linear.z = 0.0;

        odom.twist.twist.angular.x = 0.0;
        odom.twist.twist.angular.y = 0.0;
        odom.twist.twist.angular.z = vth;
        odom.twist.covariance = boost::array<double, 36>{{1e-3, 0, 0, 0, 0, 0,
                                                            0, 1e-3, 0, 0, 0, 0,
                                                            0, 0, 1e-3, 0, 0, 0,
                                                            0, 0, 0, 1e-3, 0, 0,
                                                            0, 0, 0, 0, 1e-3, 0,
                                                            0, 0, 0, 0, 0, 1e-3}};

        //publish the message
        last_time = current_time;
        odom_broadcaster.sendTransform(odom_trans);
        odom_pub.publish(odom);

        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
