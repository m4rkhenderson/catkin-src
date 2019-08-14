#include <ros/ros.h>
#include <std_msgs/String.h>

int main (int argc, char** argv){
    ros::init(argc, argv, "encoder_request");
    ros::NodeHandle n;

    ros::Publisher write_pub = n.advertise<std_msgs::String>("write", 10);

    ros::Rate r(20);
    while(ros::ok()){
        std_msgs::String msg;
        msg.data = 101;
        write_pub.publish(msg);
        ros::spinOnce();
        r.sleep();
    }
}
