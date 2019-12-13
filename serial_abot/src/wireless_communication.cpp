#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <serial/serial.h>
#include <math.h>
#include <string.h>

serial::Serial ser;
std::string loc;
std::string nn;

geometry_msgs::Pose robot;
geometry_msgs::Pose person;
geometry_msgs::PoseArray people;

int people_num;
int m_x;
int m_y;
bool s_a;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wireless_communication");
  ros::NodeHandle nh;

  nn = ros::this_node::getName();
  nh.param(nn + "/people_num", people_num, 3);
  nh.param(nn + "/mirror_x", m_x, 1);
  nh.param(nn + "/mirror_x", m_y, 1);
  nh.param(nn + "/swap_axes", s_a, false);

  ros::Publisher people_pub = nh.advertise<geometry_msgs::PoseArray>("people", 10);
  people.header.frame_id = "odom";

  try
  {
      ser.setPort("/dev/ttyACM0");
      ser.setBaudrate(38400);
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

  for(int i=0; i<people_num; i++){ // pre-allocate space for the vector
    people.poses.push_back(person);
  }

  ros::Rate r(25);

  while(nh.ok()){
    ros::spinOnce();

    if(ser.available() > 0){
      ROS_INFO("Reading from serial port");
      loc = ser.read(ser.available());
      if(loc.c_str()[0] == 112){
        ROS_INFO("Read: %c %d %d %d %d", char(loc.c_str()[0]), int(loc.c_str()[1]), int(loc.c_str()[2]), int(loc.c_str()[3]), int(loc.c_str()[4]));
        //double magnitude;
        //double angle;
        if(s_a == false){
          person.position.x = m_x*(double(loc.c_str()[2])/10 - robot.position.x); // information is scaled to increase resolution
          person.position.y = m_y*(double(loc.c_str()[3])/10 - robot.position.y); // get position differences to calculate vector magnitude
        }
        else{
          person.position.y = m_x*(double(loc.c_str()[2])/10 - robot.position.x); // information is scaled to increase resolution
          person.position.x = m_y*(double(loc.c_str()[3])/10 - robot.position.y); // get position differences to calculate vector magnitude
        }
        person.orientation.z = double(loc.c_str()[4])/10 - robot.orientation.z;
        person.orientation.w = 1; // might need to change
        ROS_INFO("Person Position: %f, %f", person.position.x, person.position.y);
        //magnitude = sqrt(person.position.x*person.position.x + person.position.y*person.position.y);
        //angle = atan2(person.position.y,person.position.x);
        //person.position.x = magnitude*cos(angle);// we need the position relative to the robot's frame
        //person.position.y = magnitude*sin(angle);

        people.poses[int(loc.c_str()[1])-2] = person; // update the position for the given human id
        people_pub.publish(people);
      }
      else if(loc.c_str()[0] == 114){
        ROS_INFO("Read: %c %d %d %d %d", char(loc.c_str()[0]), int(loc.c_str()[1]), int(loc.c_str()[2]), int(loc.c_str()[3]), int(loc.c_str()[4]));
        robot.position.x = float(loc.c_str()[2])/10; // information is scaled to increase resolution
        robot.position.y = float(loc.c_str()[3])/10;
        robot.orientation.z = float(loc.c_str()[4])/10;
      }
    }

    r.sleep();
  }
}
