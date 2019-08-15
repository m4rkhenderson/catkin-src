/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Haikal Pribadi <haikal.pribadi@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of the Haikal Pribadi nor the names of other
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "eddie.h"
typedef std::map<std::string, unsigned char[6] > CommandMap;

Eddie::Eddie() :
  GPIO_COUNT(10),
  ADC_PIN_COUNT(8),
  DIGITAL_PIN_COUNT(10), //12
  AUXILIARY_POWER_RELAY_COUNT(3),
  PARALLAX_MAX_BUFFER(256), 
  ENCODER_COUNT(2),
  ADC_VOLTAGE_MULTIPLIER(1 / 819),
  BATTERY_VOLTAGE_DIVIDER(3.21), 
  MOTOR_POWER_STOP(0),
  MOTOR_POWER_MAX_FORWARD(127),
  MOTOR_POWER_MAX_REVERSE(-127), 
  TRAVEL_SPEED_MAX_FORWARD(32767),
  TRAVEL_SPEED_MAX_REVERSE(-32767),
  TRAVEL_MAX_SPEED(65535), 
  RELAY_33V_PIN_NUMBER(17),
  RELAY_5V_PIN_NUMBER(17),
  RELAY_12V_PIN_NUMBER(18), 
  PACKET_TERMINATOR('\r'),
  PARAMETER_DELIMITER(' '),
  ERROR("ERROR"), 
  DEFAULT_WHEEL_RADIUS(0.0762),
  DEFAULT_TICKS_PER_REVOLUTION(36),
  GET_VERSION_STRING("VER"), 
  SET_GPIO_DIRECTION_OUT_STRING("OUT"),
  SET_GPIO_DIRECTION_IN_STRING("IN"),
  SET_GPIO_STATE_HIGH_STRING("HIGH"), 
  SET_GPIO_STATE_LOW_STRING("LOW"),
  GET_GPIO_STATE_STRING("READ"),
  GET_ADC_VALUE_STRING("ADC"), 
  GET_PING_VALUE_STRING("PING"),
  SET_DRIVE_POWER_STRING("GO"),
  SET_DRIVE_SPEED_STRING("GOSPD"), 
  SET_DRIVE_DISTANCE_STRING("TRVL"),
  SET_STOP_DISTANCE_STRING("STOP"),
  SET_ROTATE_STRING("TURN"), 
  GET_CURRENT_SPEED_STRING("SPD"),
  GET_CURRENT_HEADING_STRING("HEAD"),
  GET_ENCODER_TICKS_STRING("DIST"), 
  RESET_ENCODER_TICKS_STRING("RST"),
  SET_RAMPING_VALUE_STRING("ACC"),
  FLUSH_BUFFERS_STRING("\r\r\r")
{
  sem_init(&mutex, 0, 1);
  // messages
  ping_pub_ = node_handle_.advertise<morelab_robot_platform::Ping > ("/eddie/ping_data", 1);
  adc_pub_ = node_handle_.advertise<morelab_robot_platform::ADC > ("/eddie/adc_data", 1);
  distance_wheel_pub_ = node_handle_.advertise<morelab_robot_platform::DistanceWheel>("/eddie/distance_wheel", 1);
  speed_wheel_pub_ = node_handle_.advertise<morelab_robot_platform::SpeedWheel>("/eddie/speed_wheel", 1);
  // services
  accelerate_srv_ = node_handle_.advertiseService("acceleration_rate", &Eddie::accelerate, this);
  drive_with_distance_srv_ = node_handle_.advertiseService("drive_with_distance", &Eddie::driveWithDistance, this);
  drive_with_power_srv_ = node_handle_.advertiseService("drive_with_power", &Eddie::driveWithPower, this);
  drive_with_speed_srv_ = node_handle_.advertiseService("drive_with_speed", &Eddie::driveWithSpeed, this);
  get_distance_srv_ = node_handle_.advertiseService("get_distance", &Eddie::getDistance, this);
  get_heading_srv_ = node_handle_.advertiseService("get_heading", &Eddie::getHeading, this);
  get_speed_srv_ = node_handle_.advertiseService("get_speed", &Eddie::GetSpeed, this);
  reset_encoder_srv_ = node_handle_.advertiseService("reset_encoder", &Eddie::resetEncoder, this);
  rotate_srv_ = node_handle_.advertiseService("rotate", &Eddie::rotate, this);
  stop_at_distance_srv_ = node_handle_.advertiseService("stop_at_distance", &Eddie::stopAtDistance, this);

  std::string port = "/dev/ttyUSB0";
  node_handle_.param<std::string>("serial_port", port, port);
  initialize(port);
  
  command(RESET_ENCODER_TICKS_STRING);
}

Eddie::~Eddie()
{
  command("STOP 0");
  close(tty_fd);
}

void Eddie::initialize(std::string port)
{
  ROS_INFO("Initializing Parallax board serial port connection");

  memset(&tio, 0, sizeof (tio));
  tio.c_iflag = 0;
  tio.c_oflag = 0;
  tio.c_cflag = CS8 | CREAD | CLOCAL; // 8n1, see termios.h for more information
  tio.c_lflag = 0;
  tio.c_cc[VMIN] = 1;
  tio.c_cc[VTIME] = 5;

  tty_fd = open(port.data(), O_RDWR | O_NONBLOCK);
  cfsetospeed(&tio, B115200); // 115200 baud
  cfsetispeed(&tio, B115200); // 115200 baud

  tcsetattr(tty_fd, TCSANOW, &tio);
  usleep(100000);
}

std::string Eddie::command(std::string str)
{
  sem_wait(&mutex);
  ssize_t written;
  std::stringstream result("");
  int count = 0;
  unsigned char c;
  int size = str.size() + 1;
  unsigned char command[size];

  for (int i = 0; i < size - 1; i++)
  {
    command[i] = str[i];
  }
  command[size - 1] = PACKET_TERMINATOR; // Having exces terminator is okay, it's good to guarantee

  written = write(tty_fd, command, size);
  while(read(tty_fd, &c, 1) <= 0){
    usleep(1000);
    count++;
    if(count>=80)
    {
      ROS_ERROR("ERROR: NO PARALLAX EDDIE ROBOT IS CONNECTED.");
      break;
    }
  }
  if(count<80){
    result << c;
    count = 0;
    while(c != '\r'){
      if(read(tty_fd, &c, 1)>0)
        result << c;
    }
  }
  sem_post(&mutex);
  return result.str();
}

std::string Eddie::generateCommand(std::string str1)
{
  std::stringstream ss;
  ss << str1 << PACKET_TERMINATOR;
  return ss.str();
}

std::string Eddie::generateCommand(std::string str1, int num1)
{
  std::stringstream ss;
  ss << str1 << PARAMETER_DELIMITER << intToHexString(num1) << PACKET_TERMINATOR;
  return ss.str();
}

std::string Eddie::generateCommand(std::string str1, int num1, int num2)
{
  std::stringstream ss;
  ss << str1 << PARAMETER_DELIMITER << 
    intToHexString(num1) << PARAMETER_DELIMITER <<
    intToHexString(num2) << PACKET_TERMINATOR;
  return ss.str();
}

std::string Eddie::intToHexString(int num)
{
  std::stringstream ss;
  ss << std::hex << num;
  return ss.str();
}
// data from ping sensors
morelab_robot_platform::Ping Eddie::getPingData()
{
  std::string result = command(GET_PING_VALUE_STRING);
  //std::string result = "133 3C9 564 0F9 29B 0F0 31A 566 1E0 A97\r";
  morelab_robot_platform::Ping ping_data;
  if (result.size() <= 1)
  {
    ping_data.status = "EMPTY";
    return ping_data;
  }
  else if (result.size() >= 6)
  {
    if (result.substr(0, 5) == "ERROR")
    {
      ping_data.status = result;
      return ping_data;
    }
  }
  ping_data.status = "SUCCESS";
  std::stringstream value;
  uint16_t data;
  for (ushort i = 0; i < result.size(); i += 4)
  {
    if (result[i] == '\r') break;
    value << std::hex << result.substr(i, 3);
    value >> data;
    ping_data.value.push_back(data);
    value.str(std::string());
    value.clear();
  }
  return ping_data;
}

// data from infared sensors
morelab_robot_platform::ADC Eddie::getADCData()
{
  std::string result = command(GET_ADC_VALUE_STRING);
  //std::string result = "9C7 11E E4E 5AB 20F 97B 767 058\r";
  morelab_robot_platform::ADC adc_data;
  if (result.size() <= 1)
  {
    adc_data.status = "EMPTY";
    return adc_data;
  }
  else if (result.size() >= 6)
  {
    if (result.substr(0, 5) == "ERROR")
    {
      adc_data.status = result;
      return adc_data;
    }
  }
  adc_data.status = "SUCCESS";
  std::stringstream value;
  uint16_t data;
  for (ushort i = 0; i < result.size(); i += 4)
  {
    if (result[i] == '\r') break;
    value << std::hex << result.substr(i, 3);
    value >> data;
    //ROS_INFO("adc_data =  %d", data);
    //printf("%d \n", data);
    adc_data.value.push_back(data);
    value.str(std::string());
    value.clear();
  }
  return adc_data;
}

// data from wheel encoder
morelab_robot_platform::DistanceWheel Eddie::getDistanceWheel()
{
    std::string result = command(GET_ENCODER_TICKS_STRING);
    morelab_robot_platform::DistanceWheel distance_wheel_data;
    if(result.size() <=1)
    {
        distance_wheel_data.status = "EMPTY";
        return distance_wheel_data;
    }
    else if (result.size() >=18)
    {
        if(result.substr(0,5)=="ERROR")
        {
            distance_wheel_data.status = result;
            return distance_wheel_data;
        }
    }
    distance_wheel_data.status = "SUCCESS";
    std::stringstream value;
    uint32_t data;
    for(ushort i=0; i<result.size();i +=9)
    {
        if(result[i]=='\r') break;
        value << std::hex <<result.substr(i,8);
        value >> data;
        ROS_INFO("distance_wheel %d = %d", i, data);
        distance_wheel_data.value.push_back(data);
        value.str(std::string());
        value.clear();

    }
    return distance_wheel_data;
}

// data from speed of wheel
morelab_robot_platform::SpeedWheel Eddie::getSpeedWheel()
{
    std::string result = command(GET_CURRENT_SPEED_STRING);
    morelab_robot_platform::SpeedWheel speed_wheel_data;
    if(result.size() <=1)
    {
        speed_wheel_data.status = "EMPTY";
        return speed_wheel_data;
    }
    else if (result.size() >=10)
    {
        if(result.substr(0,5) == "ERROR")
        {
            speed_wheel_data.status = result;
            return speed_wheel_data;
        }
    }
    speed_wheel_data.status = "SUCCESS";
    std::stringstream value;
    int16_t data;
    for(ushort i =0; i < result.size(); i +=5)
    {
        if(result[i]=='\r') break;
        value << std::hex << result.substr(i,4);
        value >> data;
        ROS_INFO("speed_wheel %d = %d", i, data);
        speed_wheel_data.value.push_back(data);
        value.str(std::string());
        value.clear();
    }
    return speed_wheel_data;

}

// publish data
void Eddie::publishPingData()
{
  ping_pub_.publish(getPingData());
}

void Eddie::publishADCData()
{
  adc_pub_.publish(getADCData());
}

void Eddie::publishDistanceWheel()
{
    distance_wheel_pub_.publish(getDistanceWheel());
}

void Eddie::publishSpeedWheel()
{
    speed_wheel_pub_.publish(getSpeedWheel());
}

// Control the pwm, The left and right power levels are entered as signed (two’s complement) 8-bit hex values(-128:127)

bool Eddie::driveWithPower(morelab_robot_platform::DriveWithPower::Request &req,
  morelab_robot_platform::DriveWithPower::Response &res)
{
  if (req.left > MOTOR_POWER_MAX_FORWARD || req.right > MOTOR_POWER_MAX_FORWARD ||
      req.left < MOTOR_POWER_MAX_REVERSE || req.right < MOTOR_POWER_MAX_REVERSE)
  {
    return false;
  }
  std::string cmd;
  cmd = generateCommand(SET_DRIVE_POWER_STRING, req.left, req.right);
  std::string cmd_response = command(cmd);
  if (cmd_response == "\r")
    return true;
  else{
    ROS_ERROR("%s-DriveWithPower",cmd_response.data());
    return false;
  }
}

/*
* Positions per second and are entered as signed (two’s complement) 16-bit hex values
*/

bool Eddie::driveWithSpeed(morelab_robot_platform::DriveWithSpeed::Request &req,
  morelab_robot_platform::DriveWithSpeed::Response &res)
{
  if (req.left > TRAVEL_SPEED_MAX_FORWARD || req.right > TRAVEL_SPEED_MAX_FORWARD ||
      req.left < TRAVEL_SPEED_MAX_REVERSE || req.right < TRAVEL_SPEED_MAX_REVERSE)
  {
    return false;
  }
  std::string cmd;
  cmd = generateCommand(SET_DRIVE_SPEED_STRING, req.left, req.right);
  std::string cmd_response = command(cmd);
  if (cmd_response == "\r")
  {
      ROS_INFO("Request: left_speed = %d, right_speed = %d", req.left, req.right);
      return true;
  }
  else
  {
      ROS_ERROR("%s",cmd_response.data());
      return false;
  }
}

/*
* Distance of travel, in positions, entered as a signed (two’s complement) 16-bit hex value
* Speed, in positions per second, entered as a 16-bit hex value.
* Example TRVL 1A3 25
*/

bool Eddie::driveWithDistance(morelab_robot_platform::DriveWithDistance::Request &req,
  morelab_robot_platform::DriveWithDistance::Response &res)
{
  //this feature does not need to validate the parameters due the limited range of parameter data type
  std::string cmd;
  cmd = generateCommand(SET_DRIVE_DISTANCE_STRING, req.distance, req.speed);
  std::string cmd_response = command(cmd);
  if (cmd_response == "\r")
    return true;
  else{

      ROS_ERROR("%s", cmd_response.data());
      return false;
  }
}

/*
 * Angle of rotation (in degrees) is entered as a signed (two’s complement) 16-bit hex value.
 * Speed (in positions per second) is entered as a 16-bit hex value.
 * Example: TURN FEF1 4B
*/
bool Eddie::rotate(morelab_robot_platform::Rotate::Request &req,
  morelab_robot_platform::Rotate::Response &res)
{
  std::string cmd;
  cmd = generateCommand(SET_ROTATE_STRING, req.angle, req.speed);
  std::string cmd_response = command(cmd);
  if (cmd_response == "\r")
    return true;
  else{
       ROS_ERROR("%s", cmd_response.data());
       return false;
  }
}
/*
* Stopping distance, in positions, entered as a 16-bit hex value.
*/

bool Eddie::stopAtDistance(morelab_robot_platform::StopAtDistance::Request &req,
  morelab_robot_platform::StopAtDistance::Response &res)
{
  std::string cmd;
  cmd = generateCommand(SET_STOP_DISTANCE_STRING, req.distance);
  std::string cmd_response = command(cmd);
  if (cmd_response == "\r")
    return true;
  else
    return false;
}


/*
 * The rate of acceleration (in positions per second per second) is entered as a 16-bit hex value
 * Example: ACC 100

*/
bool Eddie::accelerate(morelab_robot_platform::Accelerate::Request &req,
  morelab_robot_platform::Accelerate::Response &res)
{
  //this feature does not need to validate the parameters due the limited range of parameter data type
  std::string cmd;
  cmd = generateCommand(SET_RAMPING_VALUE_STRING, req.rate);
  std::string cmd_response = command(cmd);
  if (cmd_response == "\r")
    return true;
  else
    return false;
}


bool Eddie::getDistance(morelab_robot_platform::GetDistance::Request &req,
  morelab_robot_platform::GetDistance::Response &res)
{
  std::string cmd = GET_ENCODER_TICKS_STRING;
  std::string cmd_response = command(cmd);
  if (cmd_response.substr(0, 5) != "ERROR" && cmd_response.size() >= 18)
  {
    std::stringstream value;

    value << std::hex << cmd_response.substr(0, 8); // 32 bit
    value >> res.left;

    value.str(std::string());
    value.clear();
    value << std::hex << cmd_response.substr(9, 8); //
    value >> res.right;
    ROS_INFO("Request: left = %ld, right = %ld", (long int)res.left, (long int)res.right);

    return true;
  }
  else
    return false;
}

bool Eddie::getHeading(morelab_robot_platform::GetHeading::Request &req,
  morelab_robot_platform::GetHeading::Response &res)
{
  std::string cmd = GET_CURRENT_HEADING_STRING;
  std::string cmd_response = command(cmd);
  if (cmd_response.substr(0, 5) != "ERROR" && cmd_response.size() >= 4)
  {
    std::stringstream value;
    value << std::hex << cmd_response.substr(0, 3);
    value >> res.heading;
    return true;
  }
  else
    return false;
}

bool Eddie::GetSpeed(morelab_robot_platform::GetSpeed::Request &req,
  morelab_robot_platform::GetSpeed::Response &res)
{
  std::string cmd = GET_CURRENT_SPEED_STRING;
  std::string cmd_response = command(cmd);
  if (cmd_response.substr(0, 5) != "ERROR" && cmd_response.size() >= 10)
  {
    std::stringstream value;
    value << std::hex << cmd_response.substr(0, 4);
    value >> res.left;
    value.str(std::string());
    value.clear();
    value << std::hex << cmd_response.substr(5, 4);
    value >> res.right;
    ROS_INFO("Request: left_speed = %ld, right_speed = %ld", (long int)res.left, (long int)res.right);

    return true;
  }
  else
    return false;
}

bool Eddie::resetEncoder(morelab_robot_platform::ResetEncoder::Request &req,
  morelab_robot_platform::ResetEncoder::Response &res)
{
  std::string cmd = RESET_ENCODER_TICKS_STRING;
  std::string cmd_response = command(cmd);
  if (cmd_response == "\r")
    return true;
  else
    return false;
}


int main(int argc, char** argv)
{
  ROS_INFO("Parallax Board booting up");
  ros::init(argc, argv, "parallax_board");
  Eddie eddie; //set port to connect to Parallax controller board
  ros::Rate loop_rate(10); // 1/10 hz (DSZ-10hz)

  while (ros::ok())
  {
      //ROS_INFO("MAIN EDDIE");
    eddie.publishPingData();
    eddie.publishADCData();
    eddie.publishDistanceWheel();
    eddie.publishSpeedWheel();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
