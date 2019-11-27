#include <ros/ros.h>
#include <ros/console.h>
#include <interactive_markers/interactive_marker_client.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>

using namespace visualization_msgs;

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler menu_handler;

bool grid_snap_ = false;
unsigned int marker_id_ = 0;
int num_markers_ = 3;
double goal_tolerance_;
std::string base_frame_;

std::vector<geometry_msgs::PoseStamped> waypoints_;

std::string nn;

Marker makeBox(InteractiveMarker &msg){
  Marker marker;
  marker.type = Marker::CUBE;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  return marker;
}

InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg ){
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( makeBox(msg) );
  msg.controls.push_back( control );

  return msg.controls.back();
}

void frameCallback(const ros::TimerEvent&){
  static tf::TransformBroadcaster br;

  tf::Transform t;

  ros::Time time = ros::Time::now();

  t.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  t.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
  br.sendTransform(tf::StampedTransform(t, time, "marker_base_link", "map"));
}

void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){
  std::ostringstream s;
  s << "Feedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'";

  std::ostringstream mouse_point_ss;
  if( feedback->mouse_point_valid )
  {
    mouse_point_ss << " at " << feedback->mouse_point.x
                   << ", " << feedback->mouse_point.y
                   << ", " << feedback->mouse_point.z
                   << " in frame " << feedback->header.frame_id;
  }

  switch ( feedback->event_type )
  {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
      ROS_INFO_STREAM( s.str() << ": button click" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
      ROS_INFO_STREAM( s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
      ROS_INFO_STREAM( s.str() << ": pose changed"
          << "\nposition = "
          << feedback->pose.position.x
          << ", " << feedback->pose.position.y
          << ", " << feedback->pose.position.z
          << "\norientation = "
          << feedback->pose.orientation.w
          << ", " << feedback->pose.orientation.x
          << ", " << feedback->pose.orientation.y
          << ", " << feedback->pose.orientation.z
          << "\nframe: " << feedback->header.frame_id
          << " time: " << feedback->header.stamp.sec << "sec, "
          << feedback->header.stamp.nsec << " nsec" );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
      ROS_INFO_STREAM( s.str() << ": mouse down" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
      ROS_INFO_STREAM( s.str() << ": mouse up" << mouse_point_ss.str() << "." );
      break;
  }

  server->applyChanges();
}

void alignMarker( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){
  geometry_msgs::Pose pose = feedback->pose;

  pose.position.x = round(pose.position.x-0.5)+0.5;
  pose.position.y = round(pose.position.y-0.5)+0.5;

  ROS_INFO_STREAM( feedback->marker_name << ":"
      << " aligning position = "
      << feedback->pose.position.x
      << ", " << feedback->pose.position.y
      << ", " << feedback->pose.position.z
      << " to "
      << pose.position.x
      << ", " << pose.position.y
      << ", " << pose.position.z );

  server->setPose( feedback->marker_name, pose );
  server->applyChanges();
}

double rand( double min, double max ){
  double t = (double)rand() / (double)RAND_MAX;
  return min + t*(max-min);
}

void saveMarker( InteractiveMarker int_marker ){
  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
}

void makeWaypointMarker( const tf::Vector3& position )
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "marker_base_link";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  std::string marker_name = "Waypoint_";
  marker_name += std::to_string(marker_id_);
  int_marker.name = marker_name;
  marker_id_++;
  int_marker.description = marker_name + ": Move to Goal Position";

  InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
  int_marker.controls.push_back(control);

  // make a box which also moves in the plane
  control.markers.push_back( makeBox(int_marker) );
  control.always_visible = true;
  int_marker.controls.push_back(control);

  // add context menu to start goal publishing
  control.interaction_mode = InteractiveMarkerControl::MENU;
  control.description="Options";
  int_marker.controls.push_back(control);

  // we want to use our special callback function
  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);

  // set different callback for POSE_UPDATE feedback
  if(grid_snap_ == true){
    server->setCallback(int_marker.name, &alignMarker, visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE );
  }
  menu_handler.apply( *server, int_marker.name );
}

void setWaypoint(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
  geometry_msgs::PoseStamped waypoint;
  waypoint.pose = feedback->pose;
  waypoint.header.frame_id = "map";
  waypoints_.push_back(waypoint);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "waypoint_markers");
  ros::NodeHandle n;

  ros::Rate r(10);

  nn = ros::this_node::getName();
  n.param(nn + "/grid_snap", grid_snap_, false);
  n.param(nn + "/num_markers", num_markers_, 4);
  n.param(nn + "/goal_tolerance", goal_tolerance_, 2.0);
  n.param<std::string>(nn + "/base_frame", base_frame_, "/base_footprint");

  ros::Publisher waypoint_pub = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 10);
  tf::TransformListener listener;
  tf::StampedTransform transform;

  // create a timer to update the published transforms
  ros::Timer frame_timer = n.createTimer(ros::Duration(0.01), frameCallback);

  server.reset( new interactive_markers::InteractiveMarkerServer("waypoint_markers","",false) );

  ros::Duration(0.1).sleep();

  menu_handler.insert("Set Waypoint", &setWaypoint);

  tf::Vector3 position;
  for(int i=0; i<num_markers_; i++){
    position = tf::Vector3(100, 100 + 2*i, 0);
    makeWaypointMarker(position);
  }

  server->applyChanges();

  while(n.ok()){
    if(waypoints_.size() >= num_markers_){
      for(int i=0; i<num_markers_; i++){
        waypoint_pub.publish(waypoints_[i]);
        double distance = 100;
        while(distance > goal_tolerance_){
          listener.lookupTransform("/map", base_frame_, ros::Time(0), transform);
          distance = sqrt((transform.getOrigin().x() - waypoints_[i].pose.position.x)*
                          (transform.getOrigin().x() - waypoints_[i].pose.position.x)+
                          (transform.getOrigin().y() - waypoints_[i].pose.position.y)*
                          (transform.getOrigin().y() - waypoints_[i].pose.position.y));
          ROS_INFO("Distance to Goal: %f", distance);
          ros::spinOnce();
          r.sleep();
        }
      }
    }
    ros::spinOnce();
    r.sleep();
  }

  server.reset();
}
