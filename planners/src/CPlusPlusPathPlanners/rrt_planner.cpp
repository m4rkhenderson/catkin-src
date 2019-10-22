#include <ros/console.h>
#include <planners/rrt_planner.h>
#include <pluginlib/class_list_macros.h>
#include <base_local_planner/goal_functions.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <stdlib.h>
#include <math.h>

PLUGINLIB_EXPORT_CLASS(rrt_planner::RRTPlanner, nav_core::BaseLocalPlanner)

namespace rrt_planner {



  // Public functions //


/********************************************************************************************************************/
  RRTPlanner::RRTPlanner() : initialized_(false), odom_helper_("odom"){

  }

/********************************************************************************************************************/
  void RRTPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros){
    if(initialized_ == false){

      ros::NodeHandle private_nh("~/" + name);

      private_nh.param("robot_radius", robot_radius_, 5);
      private_nh.param("goal_tolerance", goal_tolerance_, 10);
      private_nh.param("obstacle_inflation_radius", obstacle_inflation_radius_, 1);
      private_nh.param("time_step", time_step_, 0.25);
      private_nh.param("num_step", num_step_, 3);
      private_nh.param("linear_velocity_max", linear_velocity_max_, 0.5);
      private_nh.param("angular_velocity_max", angular_velocity_max_, 0.2);
      private_nh.param("linear_velocity_min", linear_velocity_min_, 0.0);
      private_nh.param("angular_velocity_min", angular_velocity_min_, 0.0);
      private_nh.param("motion_primitive_resolution", motion_primitive_resolution_, 3);


      if(motion_primitive_resolution_ < 2){
        ROS_WARN("Motion Primitive Resolution must be greater than 1,"
                 " it has automatically been set to 2 to avoid an error.");
        motion_primitive_resolution_ = 2;
      }
      RRTPlanner::velocity_t motion_primitive = {0, 0};
      for(int i=0; i<motion_primitive_resolution_; i++){
        for(int j=0; j<motion_primitive_resolution_; j++){
          motion_primitive.v = i*((linear_velocity_max_ - linear_velocity_min_)/(motion_primitive_resolution_ - 1))
              + linear_velocity_min_;  // will make identical motion primitives if max = min
          motion_primitive.vth = j*((angular_velocity_max_ - angular_velocity_min_)/(motion_primitive_resolution_ - 1))
              + angular_velocity_min_; // will make identical motion primitives if max = min
          motion_primitive_array_.push_back(motion_primitive);
        }
      }

      l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);

      tf_ = tf;
      costmap_ros_ = costmap_ros;
      costmap_ros_ -> getRobotPose(current_pose_);

      costmap_2d::Costmap2D* costmap = costmap_ros_ -> getCostmap();
      planner_util_.initialize(tf, costmap, costmap_ros_ -> getGlobalFrameID());

      if( private_nh.getParam("odom_topic", odom_topic_)){
        odom_helper_.setOdomTopic(odom_topic_);
      }

      initialized_ = true;
    }

    else{
      ROS_WARN("This planner has already been inititialized.");
    }
  }

/********************************************************************************************************************/
  bool RRTPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan){
    if(initialized_ == false){
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    global_plan_.clear();
    global_plan_ = plan;

    reached_goal_ = false;

    ROS_INFO("Got new plan");
    return true;
  }

/********************************************************************************************************************/
  bool RRTPlanner::isGoalReached(){
    double infD = 0;
    double d = 0;
    double dX = 0;
    double dY = 0;

    if(initialized_ == false){
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    if(! costmap_ros_ -> getRobotPose(current_pose_)){
      ROS_ERROR("Could not get robot pose");
      return false;
    }

    if(! planner_util_.getGoal(goal_pose_)){
      ROS_ERROR("Could not get goal pose");
      return false;
    }

    costmap_ = costmap_ros_ -> getCostmap();
    if(! costmap_){
      ROS_ERROR("Could not get costmap pointer");
    }

    dX = current_pose_.pose.position.x - goal_pose_.pose.position.x;
    dY = current_pose_.pose.position.y - goal_pose_.pose.position.x;
    d = sqrt(dX*dX + dY*dY)/(costmap_ -> getResolution());
    infD = infD = d + -1*(goal_tolerance_ + robot_radius_);
    if(infD > 0){
      reached_goal_ = true;
      return true;
    }

    return false;
  }

/********************************************************************************************************************/
  bool RRTPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
    if(! costmap_ros_ -> getRobotPose(current_pose_)){
      ROS_ERROR("Could not get robot pose");
      return false;
    }

    costmap_ = costmap_ros_ -> getCostmap();
    if(! costmap_){
      ROS_ERROR("Could not get costmap pointer");
    }

    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    if(! planner_util_.getLocalPlan(current_pose_, transformed_plan)){ // get transformed global plan
      ROS_ERROR("Could not get local plan");
      return false;
    }

    if(transformed_plan.empty()){
      ROS_WARN_NAMED("rrt_planner", "Received an empty transformed plan.");
      return false;
    }
    ROS_DEBUG_NAMED("rrt_planner", "Received a transformed plan with %zu points.", transformed_plan.size());

    RRTPlanner::vertex_t qInit = {0, {current_pose_.pose.position.x/(costmap_ -> getResolution()),
                                      current_pose_.pose.position.y/(costmap_ -> getResolution()),
                                                   2*asin(current_pose_.pose.orientation.z)}, 0, {0, 0}};
    RRTPlanner::vertex_t qGoal = {0, {transformed_plan[0].pose.position.x/(costmap_ -> getResolution()),
                                      transformed_plan[0].pose.position.y/(costmap_ -> getResolution()),
                                                   2*asin(transformed_plan[0].pose.orientation.z)}, 0, {0, 0}};

    RRTPlanner::rrt(qInit, qGoal);
  }



  // Private functions //


/********************************************************************************************************************/
  RRTPlanner::ros_cmd_t RRTPlanner::rrt(RRTPlanner::vertex_t qInit, RRTPlanner::vertex_t qGoal){
    RRTPlanner::vertex_t qRand;
    RRTPlanner::vertex_t qNear;
    RRTPlanner::vertex_t qNew;
    std::vector<RRTPlanner::vertex_t> branch;
    RRTPlanner::ros_cmd_t path_and_cmd;
    int cntID = 0;
    double infD = 0;
    double d = 0;
    double dX = 0;
    double dY = 0;
    bool path_found = false;

    tree_.clear();
    tree_.push_back(qInit);

    while(path_found == false){

      RRTPlanner::localCostmap();

      qRand = RRTPlanner::sampling(l_cm_pose_x_ + l_cm_width_, l_cm_pose_y_ + l_cm_height_,
                                   l_cm_pose_x_, l_cm_pose_y_);

      if(RRTPlanner::collisionTest(qRand, l_cm_obs_, robot_radius_) > 0){
        continue;
      }

      qNear = RRTPlanner::nearestNeighbour(qRand, tree_);

      branch = RRTPlanner::motionPrimitives(qNear, l_cm_obs_, cntID, num_step_, time_step_,
                                            motion_primitive_array_, robot_radius_);

      if(branch.size() > 0){
        cntID = branch.back().id;
      }
      else{
        continue;
      }

      tree_.insert(tree_.end(), branch.begin(), branch.end()); // double check later

      for(int i=0; i<branch.size(); i++){
        dX = branch[i].pose[0] - qGoal.pose[0];
        dY = branch[i].pose[1] - qGoal.pose[1];
        d = sqrt(dX*dX + dY*dY);
        infD = d + -1*(goal_tolerance_ + robot_radius_);
        if(infD > 0){
          path_found = true;
          path_and_cmd = RRTPlanner::extractPath(branch[i], tree_, qInit);
          break;
        }
      }

    }

    return path_and_cmd;
  }

/********************************************************************************************************************/
  void RRTPlanner::localCostmap(){
    costmap_ = costmap_ros_ -> getCostmap();
    if(! costmap_){
      ROS_ERROR("Could not get costmap pointer");
    }

    rrt_planner::RRTPlanner::obstacle_t obs = {{0, 0}, 0};
    l_cm_width_ = costmap_ -> getSizeInCellsX();
    l_cm_height_ = costmap_ -> getSizeInCellsY();
    l_cm_resolution_ = costmap_ -> getResolution();
    l_cm_pose_x_ = (costmap_ -> getOriginX()) / l_cm_resolution_;
    l_cm_pose_y_ = (costmap_ -> getOriginY()) / l_cm_resolution_;

    for(unsigned int i=0; i<(l_cm_width_*l_cm_height_); i++){
      if(rrt_local_costmap_.data[i] > 0){
        obs.pose[0] = (i % l_cm_width_) + l_cm_pose_x_;
        obs.pose[1] = (i / l_cm_width_) + l_cm_pose_y_;
        obs.radius = obstacle_inflation_radius_;
        l_cm_obs_.push_back(obs);
      }
    }

    for(int i=0; i<motion_primitive_array_.size(); i++){ // convert from m/s to cell/s
      motion_primitive_array_[i].v = motion_primitive_array_[i].v/l_cm_resolution_;
    }
  }
/********************************************************************************************************************/
  bool RRTPlanner::collisionTest(RRTPlanner::vertex_t q, std::vector<RRTPlanner::obstacle_t> obs, double radius){
    bool flag = false;
    double infD = 0;
    double d = 0;
    double dX = 0;
    double dY = 0;

    for(int i=0; i<obs.size(); i++){
      dX = q.pose[0] - obs[i].pose[0];
      dY = q.pose[1] - obs[i].pose[1];
      d = sqrt(dX*dX + dY*dY);
      infD = d + -1*(obs[i].radius + radius);
      if(infD > 0){
        flag = false;
      }
      else{
        flag = true;
        break;
      }
    }
    return flag;
  }

/********************************************************************************************************************/
  RRTPlanner::ros_cmd_t RRTPlanner::extractPath(RRTPlanner::vertex_t q, std::vector<RRTPlanner::vertex_t> tree,
                                         RRTPlanner::vertex_t qInit){

    nav_msgs::Path path;
    geometry_msgs::PoseStamped pose;
    std::vector<geometry_msgs::Twist> cmd;
    geometry_msgs::Twist vel;
    RRTPlanner::ros_cmd_t path_and_cmd;
    RRTPlanner::vertex_t v = q;

    while(v.id != qInit.id){
      pose.pose.position.x = v.pose[0];
      pose.pose.position.y = v.pose[1];
      pose.pose.orientation.z = sin(v.pose[2]/2);
      pose.pose.orientation.w = cos(v.pose[2]/2);
      vel.linear.x = v.vel[0];
      vel.angular.z = v.vel[1];
//      pose.header.stamp = ros::Time::now();
//      pose.header.frame_id = "local_plan";
      path.poses.push_back(pose);
      cmd.push_back(vel);
      v = tree[v.pid];
    }

    path.header.stamp = ros::Time::now();
    path.header.frame_id = "local_plan";


    return path_and_cmd;
  }

/********************************************************************************************************************/
  RRTPlanner::vertex_t RRTPlanner::nearestNeighbour(RRTPlanner::vertex_t q, std::vector<RRTPlanner::vertex_t> tree){
    RRTPlanner::vertex_t v = {0, {0, 0, 0}, 0, {0, 0}};
    double minD = 1000000000;
    double d = 0;
    double dX = 0;
    double dY = 0;
    int minV = 0;

    for(int i=0; i<tree.size(); i++){
      dX = q.pose[0] - tree[i].pose[0];
      dY = q.pose[1] - tree[i].pose[1];
      d = sqrt(dX*dX + dY*dY);
      if(d < minD){
        minD = d;
        minV = i;
      }
    }

    v = tree[minV];
    return v;
  }

/********************************************************************************************************************/
  RRTPlanner::vertex_t RRTPlanner::sampling(int xMax, int yMax, int xMin, int yMin){
    RRTPlanner::vertex_t qRand = {0, {double(rand()%(xMax - xMin) + xMin), double(rand()%(yMax - yMin) + yMin),
                                      double(rand()%628)/100}, 0, {0, 0}};
    return qRand;
  }

/********************************************************************************************************************/
  RRTPlanner::vertex_t RRTPlanner::steering(RRTPlanner::vertex_t q, RRTPlanner::vertex_t qNear, int cntID,
                                            double dMax, double aMax){

    RRTPlanner::vertex_t qNew = {0, {0, 0, 0}, 0, {0, 0}};
    double th = 0;
    int thMax = 0;
    int thMin = 0;
    double dth = q.pose[2] - qNear.pose[2];
    if(dth < 0){ // make absolute value
      dth = -dth;
    }

    if(dth > aMax){
      thMax = int((qNear.pose[2] + aMax)*100);
      thMin = int((qNear.pose[2] - aMax)*100);
      th = double(rand()%(thMax - thMin) + thMin)/100;
    }
    else{
      th = q.pose[2];
    }

    qNew = {cntID + 1, {qNear.pose[0] + dMax*cos(th), qNear.pose[1] + dMax*sin(th), th}, 0, {0, 0}};
    return qNew;
  }

/********************************************************************************************************************/
  std::vector<RRTPlanner::vertex_t> RRTPlanner::motionPrimitives(RRTPlanner::vertex_t q,
                                                                 std::vector<RRTPlanner::obstacle_t> obs,
                                                                 int cntID, int nSteps, double tStep,
                                                                 std::vector<RRTPlanner::velocity_t> mpArray,
                                                                 double radius){

    std::vector<RRTPlanner::vertex_t> branch;
    RRTPlanner::vertex_t twig = {0, {0, 0, 0}, 0, {0, 0}};
    cntID = cntID + 1;
    int parent = 0;
    int numV = 0;
    double x = 0;
    double y = 0;
    double th = 0;
    double dd = 0;
    double dx = 0;
    double dy = 0;
    double dth = 0;
    double v = 0;
    double vth = 0;

    for(int i=0; i<mpArray.size(); i++){
      x = q.pose[0];
      y = q.pose[1];
      th = q.pose[2];

      v = mpArray[i].v;
      vth = mpArray[i].vth;

      parent = q.id;

      for(int j=0; j<nSteps; j++){
        dth = vth*tStep;
        th = th + dth;

        dd = v*tStep;
        dx = dd*cos(th);
        dy = dd*sin(th);
        x = x + dx;
        y = y + dy;

        twig = {cntID, {x, y, th}, parent, {v, vth}};
        if(RRTPlanner::collisionTest(twig, obs, radius) > 0){
          break;
        }

        branch.push_back(twig); // append twig to end of branch
        parent = branch[numV].id;
        cntID = cntID + 1;
        numV = numV + 1;
      }
    }

    return branch;
  }
}
