#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

const double maxV = 0.5;

double v = 0.0;
double vth = 0.0;
double v_p = 0.0;
double vth_p = 0.0;
double vdf = 0.0;

double vel = 0.0;
double velth = 0.0;

double errorL = 0.0;
double errorL_p = 0.0;
double derivativeL = 0.0;
double integralL = 0.0;
double KPL[] = {0.016,0.016};   //0.016<-verified by ziegler-nichols// 0.04
double KIL[] = {0.0001,0.0001};
double KDL[] = {0.00576,0.00576}; //0.00576<-verified by ziegler-nichols//0.00001// 0.01
double cL = 0.0;
double cL_p = 0.0;

double errorA = 0.0;
double errorA_p = 0.0;
double derivativeA = 0.0;
double integralA = 0.0;
double KPA[] = {0.006,0.006}; //0.006<-verified by ziegler-nichols// 0.017
double KIA[] = {0.0001,0.0001};
double KDA[] = {0.0015,0.0015};//0.0015<-verified by ziegler-nichols//0.03
double cA = 0.0;
double cA_p = 0.0;

int MODE = 0;
double cR = 0.0;
const double minRatio = 0.3;

double R = 0.0;
int VL = 0;
int VR = 0;
int DR = 0;
bool state = 0;
int count = 0;

void moveCallback(const geometry_msgs::Twist& cmd){
    v = cmd.linear.x;
    vth = cmd.angular.z;
}

void odomCallback(const nav_msgs::Odometry& odom){
  vel = odom.twist.twist.linear.x;
  velth = odom.twist.twist.angular.z;
}

int main (int argc, char** argv){
    ros::init(argc, argv, "serial_commands");
    ros::NodeHandle n;

    ros::Publisher write_pub = n.advertise<std_msgs::String>("write", 100);
    ros::Subscriber move_sub = n.subscribe("cmd_vel", 100, moveCallback);
    ros::Subscriber odom_sub = n.subscribe("odom", 100, odomCallback);

    ros::Rate r(10);
    while(ros::ok()){
        std_msgs::String msg;

        if(vth < 0){
          MODE = 0;
        }
        else if(vth >= 0){
          MODE = 1;
        }

        errorL = v - vel;
        derivativeL = errorL - errorL_p;
        //integralL = integralL + errorL;
        errorL_p = errorL;
        cL = cL + KPL[MODE]*errorL + KDL[MODE]*derivativeL;// + KIL*integralL;

        errorA = vth - velth;
        derivativeA = errorA - errorA_p;
        //integralA = integralA + errorA;
        errorA_p = errorA;
        cA = cA + KPA[MODE]*errorA + KDA[MODE]*derivativeA;// + KIA*integralA;

//        if(cA != 0.0){
//          cR = cL/cA;
//          if(-minRatio < cR < minRatio){
//            cA = cL/minRatio;
//          }
//        }

        //vdf = vth*0.45;
        //R = v/vdf;
        VL = ((2*cL-0.45*cA)/2)*(255/maxV);//vth*(R-0.45/2.0)*(255.0/maxV);// alternatively VL = (2*v-0.45*vth)/2;
        VR = (0.45*cA+VL/((255/maxV)))*(255/maxV);// vth*(R+0.45/2.0)*(255.0/maxV);// alternatively VR = 2*v-VL;

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
        //ROS_INFO("DR = %d, VL = %d, VR = %d", DR, VL, VR);
        ROS_INFO("Linear Control = %f, Angular Control = %f", cL, cA);

        switch(state){
            case 0:
                if(count < 10){
                    msg.data = {114,(char)DR,(char)VL,(char)VR};
                    count++;
                }
                else{
                    msg.data = 101;
                }
                break;
            case 1:
                msg.data = 101;
                break;
        }
        state = !state;

        if(cL != cL_p || cA != cA_p){
            count = 0;
        }

        cL_p = cL;
        cA_p = cA;
        write_pub.publish(msg);
        ros::spinOnce();
        r.sleep();
    }
}
