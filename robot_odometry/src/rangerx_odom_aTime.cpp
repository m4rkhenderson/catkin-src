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


double dCountL = 0.0;
double dCountR = 0.0;
double dt = 0;
int sync_c = 0;
int sync_p = 0;

void distanceWheelCallbackL(const std_msgs::Int16& distL){
    dCountL = distL.data;
}
void distanceWheelCallbackR(const std_msgs::Int16& distR){
    dCountR = distR.data;
}
void syncCallback(const std_msgs::Int16& sync){
    sync_c = sync.data;
}
void timeCallback(const std_msgs::Float32& dTime){
    dt = dTime.data;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "rangerx_odom");

    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);
    ros::Subscriber L_distance_wheel_sub = n.subscribe("/delta_L", 10, distanceWheelCallbackL);
    ros::Subscriber R_distance_wheel_sub = n.subscribe("/delta_R", 10, distanceWheelCallbackR);
    ros::Subscriber sync_sub = n.subscribe("/sync", 10, syncCallback);
    ros::Subscriber time_sub = n.subscribe("/delta_T", 10, timeCallback);

    double pulse_meter = 19279; //The number of encoder pulses per 1 meter of travel // note: actually somewhat different
    double base_width = 0.555; // The distance between the center of the two wheels in meters

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

    ros::Rate r(30); //hz

    tf::TransformBroadcaster odom_broadcaster;
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";

    while(n.ok()){

        current_time = ros::Time::now();
        if(dt == 0.0){
            dt = (current_time - last_time).toSec();
        }
        // compute odometry using the given velocities of the robot
//        double dt = (current_time - last_time).toSec();

        if(sync_c == sync_p){
            dCountL = 0.0;
            dCountR = 0.0;
        }
        else{
            sync_p = sync_c;
            last_time = current_time;
        }


        // distance travelled by the left and right wheels
        d_left = 1.0374*dCountL/20480;
        d_right = 1.0374*dCountR/20480;
        ROS_INFO("d_left = %f, d_right = %f", d_left, d_right);

        // distance travelled is the average of the two wheels
        delta_s = (d_left + d_right)/2;

        // this approximation works (in radians) for small angles
        delta_th = (d_right - d_left)/base_width;
        delta_x = (delta_s * cos(th + delta_th/2));
        delta_y = (delta_s * sin(th + delta_th/2));
        ROS_INFO("delta_x = %f, delta_y = %f, delta_th = %f", delta_x, delta_y, delta_th);

        // calculate velocities
        vx = delta_x/dt;
        vy = delta_y/dt;
        vth = delta_th/dt;
        v = delta_s/dt;

        ROS_INFO("x = %f, y = %f, th = %f", x, y, th);
        x += delta_x;
        y += delta_y;
        th += delta_th;

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
//        odom.child_frame_id = "odom";
//        odom.header.frame_id = "world";
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
        odom.twist.twist.linear.z = 0;
        //odom.twist.twist.linear.z = v;

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
        odom_broadcaster.sendTransform(odom_trans);
        odom_pub.publish(odom);

        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
