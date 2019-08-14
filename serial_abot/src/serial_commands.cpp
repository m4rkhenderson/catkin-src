#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

const double maxV = 0.5;

double v = 0.0;
double vth = 0.0;
double v_p = 0.0;
double vth_p = 0.0;
double vdf = 0.0;
double R = 0.0;
int VL = 0;
int VR = 0;
int DR = 0;
bool state = 0;

void moveCallback(const geometry_msgs::Twist& cmd){
    v = cmd.linear.x;
    vth = cmd.angular.z;
    //vdf = vth*0.45;
    //R = v/vdf;
    VL = ((2*v-0.45*vth)/2)*(255/maxV);//vth*(R-0.45/2.0)*(255.0/maxV);// alternatively VL = (2*v-0.45*vth)/2;
    VR = (0.45*vth+VL/((255/maxV)))*(255/maxV);// vth*(R+0.45/2.0)*(255.0/maxV);// alternatively VR = 2*v-VL;

    if(VL > 255){
        VL = 255;
    }
    else if(VL < -255){
        VL = -255;
    }
    if(VR > 255){
        VR = 255;
    }
    else if(VR < -255){
        VR = -255;
    }
    if(VR < 0 && VL < 0){
        DR = 2;
        VR = -1*VR;
        VL = -1*VL;
    }
    else if(VR > 0 && VL > 0){
        DR = 1;
    }
    else{
        DR = 0;
    }
    ROS_INFO("DR = %d, VL = %d, VR = %d", DR, VL, VR);
}

int main (int argc, char** argv){
    ros::init(argc, argv, "serial_commands");
    ros::NodeHandle n;

    ros::Publisher write_pub = n.advertise<std_msgs::String>("write", 100);
    ros::Subscriber move_sub = n.subscribe("cmd_vel", 100, moveCallback);

    ros::Rate r(10);
    while(ros::ok()){
        std_msgs::String msg;
        switch(state){
            case 0:
                msg.data = {114,(char)DR,(char)VL,(char)VR};
                break;
            case 1:
                msg.data = 101;
                break;
        }
        state = !state;
//        if(vth != vth_p || v != v_p){
//            msg.data = {114,(char)DR,(char)VL,(char)VR};
//        }
//        else{
//            msg.data = 101;
//        }
//        v_p = v;
//        vth_p = vth;
        write_pub.publish(msg);
        ros::spinOnce();
        r.sleep();
    }
}
