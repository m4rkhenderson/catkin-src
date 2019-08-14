#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>

serial::Serial ser;
std::__cxx11::string enc;
char encL[4];
char encR[4];
int left = 0;
int right = 0;

void write_callback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO_STREAM("Writing to serial port " << msg->data);
    ser.write(msg->data);
}

int main (int argc, char** argv){
    ros::init(argc, argv, "serial");
    ros::NodeHandle nh;

    ros::Subscriber write_sub = nh.subscribe("write", 100, write_callback);
    ros::Publisher readL_pub = nh.advertise<std_msgs::Int32>("readL", 1000);
    ros::Publisher readR_pub = nh.advertise<std_msgs::Int32>("readR", 1000);

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

    //ser.setRTS(false);
    //ser.setDTR(false);

    ros::Rate loop_rate(10);
    while(ros::ok()){

        ros::spinOnce();

        if(ser.available()){
            ROS_INFO_STREAM("Reading from serial port");
            std_msgs::Int32 resultL;
            std_msgs::Int32 resultR;
            if(ser.available() > 0)
            {
                enc = ser.read(ser.available());
                for(int i=0; i<4;i++){
                    encR[i] = enc[3-i];
                }
                for(int i=0; i<4;i++){
                    encL[i] = enc[7-i];
                }
                left = *(int *)encL;
                right = *(int *)encR;
                resultL.data = left;
                resultR.data = right;
                ROS_INFO("Read: %d\t%d", left, right);
                //result.data = ser.read(ser.available());
                //ROS_INFO("Read: %d%d%d%d %d%d%d%d",
                //        result.data[0], result.data[1],
                //        result.data[2], result.data[3],
                //        result.data[4], result.data[5],
                //        result.data[6], result.data[7]);
            }
            else
            {
                resultL.data = left;
                resultR.data = right;
                ROS_INFO("Read: %d\t%d", left, right);
            }
            //result.data = ser.read(8);//(ser.available());
            //ROS_INFO_STREAM("Read: " << result.data);
            readL_pub.publish(resultL);
            readR_pub.publish(resultR);
        }
        loop_rate.sleep();

    }
}

