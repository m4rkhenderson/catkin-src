#include <ros/console.h>
#include <planners/rrt_planner.h>
#include <pluginlib/class_list_macros.h>
#include <base_local_planner/goal_functions.h>
#include <tf/tf.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <stdlib.h>
#include <math.h>

PLUGINLIB_EXPORT_CLASS(rrt_planner::RRTPlanner, nav_core::BaseLocalPlanner)

namespace rrt_planner {



  // Public functions //


/********************************************************************************************************************/
  RRTPlanner::RRTPlanner() : initialized_(false), odom_helper_("odom"), tf_(NULL), costmap_ros_(NULL){

  }

/********************************************************************************************************************/
  RRTPlanner::RRTPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) :
    initialized_(false), odom_helper_("odom"), tf_(NULL), costmap_ros_(NULL){

    initialize(name, tf, costmap_ros);
  }

/********************************************************************************************************************/
  void RRTPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros){
    if(initialized_ == false){
      ROS_INFO("Initializing Local PLanner");

      ros::NodeHandle private_nh("~/" + name);

      private_nh.param("robot_radius", robot_radius_, 2);
      private_nh.param("goal_tolerance", goal_tolerance_, 10);
      private_nh.param("path_tolerance", path_tolerance_, 5);
      private_nh.param("path_checkpoint_resolution", path_checkpoint_resolution_, 1);
      private_nh.param("obstacle_inflation_radius", obstacle_inflation_radius_, 1);
      private_nh.param("time_step", time_step_, 0.3);
      private_nh.param("num_step", num_step_, 3);
      private_nh.param("linear_velocity_max", linear_velocity_max_, 0.5);
      private_nh.param("angular_velocity_max", angular_velocity_max_, 1.0);
      private_nh.param("linear_velocity_min", linear_velocity_min_, 0.0);
      private_nh.param("angular_velocity_min", angular_velocity_min_, 0.0);
      private_nh.param("motion_primitive_resolution", motion_primitive_resolution_, 3);
      private_nh.param("max_iterations", max_iterations_, 100);
      private_nh.param("controller_frequency", controller_frequency_, 10.0);
      private_nh.param("local_costmap_border", l_cm_border_, true);
      private_nh.param("forward_bias", forward_bias_, 2);
      private_nh.param<std::string>("velocity_topic", velocity_topic_, "cmd_vel");

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
          motion_primitive_array_const_.push_back(motion_primitive);
        }
        for(int j=0; j<motion_primitive_resolution_; j++){
          motion_primitive.v = i*((linear_velocity_max_ - linear_velocity_min_)/(motion_primitive_resolution_ - 1))
              + linear_velocity_min_;  // will make identical motion primitives if max = min
          motion_primitive.vth = -1*(j*((angular_velocity_max_ - angular_velocity_min_)/(motion_primitive_resolution_ - 1))
              + angular_velocity_min_); // will make identical motion primitives if max = min
          motion_primitive_array_const_.push_back(motion_primitive);
        }

        // Comment out section to disable backwards motion
        for(int j=0; j<int(motion_primitive_resolution_/forward_bias_); j++){
          motion_primitive.v = -1*(i*((linear_velocity_max_ - linear_velocity_min_)/(motion_primitive_resolution_ - 1))
              + linear_velocity_min_);  // will make identical motion primitives if max = min
          motion_primitive.vth = j*((angular_velocity_max_ - angular_velocity_min_)/(motion_primitive_resolution_ - 1))
              + angular_velocity_min_; // will make identical motion primitives if max = min
          motion_primitive_array_const_.push_back(motion_primitive);
        }
        for(int j=0; j<int(motion_primitive_resolution_/forward_bias_); j++){
          motion_primitive.v = -1*(i*((linear_velocity_max_ - linear_velocity_min_)/(motion_primitive_resolution_ - 1))
              + linear_velocity_min_);  // will make identical motion primitives if max = min
          motion_primitive.vth = -1*(j*((angular_velocity_max_ - angular_velocity_min_)/(motion_primitive_resolution_ - 1))
              + angular_velocity_min_); // will make identical motion primitives if max = min
          motion_primitive_array_const_.push_back(motion_primitive);
        }
      }

      l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
      g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
      cmd_vel_pub_ = private_nh.advertise<geometry_msgs::Twist>(velocity_topic_, 10);
      tree_pub_ = private_nh.advertise<geometry_msgs::PoseArray>("tree", 100);
      trajectory_pub_ = private_nh.advertise<nav_msgs::Path>("trajectory", 10); // for visualization (CCECE 2020)
      trajectory_.header.frame_id = "map";
      time_cost_pub_ = private_nh.advertise<std_msgs::Float64>("time_cost", 10);

      tf_ = tf;
      costmap_ros_ = costmap_ros;
      costmap_ros_ -> getRobotPose(current_pose_);
      global_frame_ = costmap_ros_ -> getGlobalFrameID();

      ROS_INFO("Global_Frame: %s", global_frame_.c_str());

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

//    trajectory_.poses.clear(); // for visualization (CCECE 2020)
    //ROS_INFO("Got new plan");
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

    if(! base_local_planner::getGoalPose(*tf_, global_plan_, global_frame_, goal_pose_)){
      ROS_ERROR("Could not get goal pose");
      return false;
    }

    costmap_ = costmap_ros_ -> getCostmap();
    if(! costmap_){
      ROS_ERROR("Could not get costmap pointer");
    }

    dX = current_pose_.pose.position.x - goal_pose_.pose.position.x;
    dY = current_pose_.pose.position.y - goal_pose_.pose.position.y;
    d = sqrt(dX*dX + dY*dY)/(costmap_ -> getResolution());
    infD = d + -1*(goal_tolerance_ + robot_radius_);
    if(infD <= 0){
      reached_goal_ = true;
      geometry_msgs::Twist cmd;
      cmd.linear.x = 0;
      cmd.linear.y = 0;
      cmd.linear.z = 0;
      cmd.angular.x = 0;
      cmd.angular.y = 0;
      cmd.angular.z = 0;
      cmd_vel_pub_.publish(cmd); // Tell Robot to Stop Upon Reaching Goal
      ROS_INFO("The Goal Has Been Reached");
      return true;
    }

    //ROS_INFO("The Goal Has Not Been Reached Yet");
    return false;
  }

/********************************************************************************************************************/
  bool RRTPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
    ROS_INFO("Computing Velocity Commands");
    if(! costmap_ros_ -> getRobotPose(current_pose_)){
      ROS_ERROR("Could not get robot pose");
      return false;
    }
    trajectory_.poses.push_back(current_pose_);// for visualization (CCECE 2020)
    trajectory_pub_.publish(trajectory_);// for visualization (CCECE 2020)

    costmap_ = costmap_ros_ -> getCostmap();
    if(! costmap_){
      ROS_ERROR("Could not get costmap pointer");
    }

    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    if(! base_local_planner::transformGlobalPlan(*tf_, global_plan_, current_pose_, *costmap_, global_frame_, transformed_plan)){ // get transformed global plan
      ROS_ERROR("Could not transform global plan");
      return false;
    }

    if(transformed_plan.empty()){
      ROS_WARN_NAMED("rrt_planner", "Received an empty transformed plan.");
      return false;
    }

    base_local_planner::prunePlan(current_pose_, transformed_plan, global_plan_); // Trim off parts of the global plan that are far enough behind the robot
    //ROS_INFO("Received a transformed plan with %zu points.", transformed_plan.size());

    tf::Quaternion q1(current_pose_.pose.orientation.x,
                     current_pose_.pose.orientation.y,
                     current_pose_.pose.orientation.z,
                     current_pose_.pose.orientation.w);
    tf::Matrix3x3 m1(q1);
    double roll1, pitch1, yaw1;
    m1.getRPY(roll1, pitch1, yaw1);

    tf::Quaternion q2(current_pose_.pose.orientation.x,
                     current_pose_.pose.orientation.y,
                     current_pose_.pose.orientation.z,
                     current_pose_.pose.orientation.w);
    tf::Matrix3x3 m2(q2);
    double roll2, pitch2, yaw2;
    m2.getRPY(roll2, pitch2, yaw2);

    RRTPlanner::vertex_t qInit = {0, {current_pose_.pose.position.x/(costmap_ -> getResolution()),
                                      current_pose_.pose.position.y/(costmap_ -> getResolution()),
                                                   yaw1}, 0, {0, 0}};
    RRTPlanner::vertex_t qGoal = {0, {transformed_plan[int(transformed_plan.size()/path_checkpoint_resolution_) -1].pose.position.x/(costmap_ -> getResolution()),
                                      transformed_plan[int(transformed_plan.size()/path_checkpoint_resolution_) -1].pose.position.y/(costmap_ -> getResolution()),
                                                   yaw2}, 0, {0, 0}};

    path_and_cmd_ = RRTPlanner::rrt(qInit, qGoal);

    //ROS_INFO("Got path and velocity commands");

    base_local_planner::publishPlan(path_and_cmd_.path, l_plan_pub_);
    base_local_planner::publishPlan(transformed_plan, g_plan_pub_);

    RRTPlanner::velocityManager();

    return true;
  }



  // Private functions //


/********************************************************************************************************************/
  RRTPlanner::ros_cmd_t RRTPlanner::rrt(RRTPlanner::vertex_t qInit, RRTPlanner::vertex_t qGoal){ // change to only activate after a set period of time to maintain controller frequency
    RRTPlanner::vertex_t qRand;
    RRTPlanner::vertex_t qNear;
    RRTPlanner::vertex_t qNew;
    std::vector<RRTPlanner::vertex_t> branch;
    RRTPlanner::ros_cmd_t path_and_cmd;
    geometry_msgs::PoseStamped goalCheck = goal_pose_;
    geometry_msgs::Pose point;
    geometry_msgs::PoseArray points;
    std::vector<geometry_msgs::Pose> tempPoints;
    points.header.frame_id = global_frame_.c_str();
    ros::WallTime current_time, previous_time, c_planner_time, p_planner_time;
    p_planner_time = ros::WallTime::now();

    int iterations = 0;
    int cntID = 0;
    double infD = 0;
    double d = 0;
    double dX = 0;
    double dY = 0;
    bool path_found = false;

    tree_.clear();
    tree_.push_back(qInit);

//    if(path_and_cmd_.cmd.size() > 0){
//      cmd_vel_pub_.publish(path_and_cmd_.cmd[0]);
//      path_and_cmd_.cmd.erase(path_and_cmd_.cmd.begin());
//    }
    previous_time = ros::WallTime::now();

    //ROS_INFO("RRT Planner Called");
    while(path_found == false && iterations < max_iterations_){
      iterations++;

      current_time = ros::WallTime::now();
      if(path_and_cmd_.cmd.size() > 0 && (current_time.toSec() - previous_time.toSec()) >= time_step_){
        //ROS_INFO("TIME PASSED: %f", (current_time - previous_time).toSec());
        cmd_vel_pub_.publish(path_and_cmd_.cmd[0]);
        path_and_cmd_.cmd.erase(path_and_cmd_.cmd.begin());
        previous_time = ros::WallTime::now();
      }

      if(base_local_planner::getGoalPose(*tf_, global_plan_, global_frame_, goal_pose_)){ //reset rrt planner if
        if(goal_pose_.pose.position.x != goalCheck.pose.position.x ||                     //global goal changes
           goal_pose_.pose.position.y != goalCheck.pose.position.y){
          return path_and_cmd;
        }
      }

      RRTPlanner::localCostmap();

      qRand = RRTPlanner::sampling(l_cm_pose_x_ + l_cm_width_, l_cm_pose_y_ + l_cm_height_,
                                   l_cm_pose_x_, l_cm_pose_y_);

      if(RRTPlanner::collisionTest(qRand, l_cm_obs_, robot_radius_) > 0){
        //ROS_INFO("Invalid Random Configuration");
        continue;
      }

      qNear = RRTPlanner::nearestNeighbour(qRand, tree_);

      branch = RRTPlanner::motionPrimitives(qNear, l_cm_obs_, cntID, num_step_, time_step_,
                                            motion_primitive_array_, robot_radius_);

      if(branch.size() > 0){
        cntID = branch.back().id;
      }
      else{
        //ROS_INFO("Failed to Form Valid Branch");
        continue;
      }

      //ROS_INFO("Added Branch to Tree");
      tree_.insert(tree_.end(), branch.begin(), branch.end()); // double check later

      for(int i=0; i<branch.size(); i++){
        point.position.x = branch[i].pose[0]*l_cm_resolution_;
        point.position.y = branch[i].pose[1]*l_cm_resolution_;
        point.position.z = 0.0;
        point.orientation.x = 0.0;
        point.orientation.y = 0.0;
        point.orientation.z = sin(branch[i].pose[2]/2);
        point.orientation.w = cos(branch[i].pose[2]/2);
        tempPoints.push_back(point);
      }
      points.poses = tempPoints;
      tree_pub_.publish(points);

      for(int i=0; i<branch.size(); i++){
        dX = branch[i].pose[0] - qGoal.pose[0];
        dY = branch[i].pose[1] - qGoal.pose[1];
        d = sqrt(dX*dX + dY*dY);
        infD = d + -1*(path_tolerance_ + robot_radius_);
        if(infD <= 0){
          path_found = true;
          path_and_cmd = RRTPlanner::extractPath(branch[i], tree_, qInit);
          c_planner_time = ros::WallTime::now();
          ROS_INFO("Planner Success: %f", (c_planner_time.toSec() - p_planner_time.toSec())); // Notify of planner success and time cost
          time_cost_.data = (c_planner_time.toSec()-p_planner_time.toSec());
          time_cost_pub_.publish(time_cost_);
          break;
        }
      }
    }
    if(path_and_cmd.cmd.size() <= 0){
      c_planner_time = ros::WallTime::now();
      ROS_INFO("Planner Failure: %f", (c_planner_time.toSec() - p_planner_time.toSec())); // Notify of planner failure and time cost
      time_cost_.data = (c_planner_time.toSec()-p_planner_time.toSec());
      time_cost_pub_.publish(time_cost_);
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
    obs.radius = obstacle_inflation_radius_;
    l_cm_obs_.clear();
    l_cm_width_ = costmap_ -> getSizeInCellsX();
    l_cm_height_ = costmap_ -> getSizeInCellsY();
    l_cm_resolution_ = costmap_ -> getResolution();
    l_cm_pose_x_ = (costmap_ -> getOriginX()) / l_cm_resolution_;
    l_cm_pose_y_ = (costmap_ -> getOriginY()) / l_cm_resolution_;

    for(unsigned int i=0; i<l_cm_width_; i++){
      for(unsigned int j=0; j<l_cm_height_; j++){
        if((costmap_ -> getCost(i,j)) > 0){
          obs.pose[0] = i + l_cm_pose_x_;
          obs.pose[1] = j + l_cm_pose_y_;
          l_cm_obs_.push_back(obs);
        }
      }
    }

    // generates a thin border around the local costmap to prevent expanding beyond it
    if(l_cm_border_ == true){
      for(unsigned int i=0; i<l_cm_width_; i++){
        obs.pose[0] = i + l_cm_pose_x_;
        obs.pose[1] = l_cm_pose_y_;
        l_cm_obs_.push_back(obs);
      }
      for(unsigned int i=0; i<l_cm_height_; i++){
        obs.pose[0] = l_cm_pose_x_;
        obs.pose[1] = i + l_cm_pose_y_;
        l_cm_obs_.push_back(obs);
      }
      for(unsigned int i=0; i<l_cm_width_; i++){
        obs.pose[0] = i + l_cm_pose_x_;
        obs.pose[1] = l_cm_height_ + l_cm_pose_y_;
        l_cm_obs_.push_back(obs);
      }
      for(unsigned int i=0; i<l_cm_height_; i++){
        obs.pose[0] = l_cm_width_ + l_cm_pose_x_;
        obs.pose[1] = i + l_cm_pose_y_;
        l_cm_obs_.push_back(obs);
      }
    }

    //ROS_INFO("Number of Obstacles in Local Costmap: %d", int(l_cm_obs_.size()));

    motion_primitive_array_ = motion_primitive_array_const_;

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

    for(int i=0; i<obs.size(); i++){        // NOTE: CostmapModel functions may be more useful here than other methods
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

    std::vector<geometry_msgs::PoseStamped> path;
    geometry_msgs::PoseStamped pose;
    std::vector<geometry_msgs::Twist> cmd;
    geometry_msgs::Twist vel;
    RRTPlanner::ros_cmd_t path_and_cmd;
    RRTPlanner::vertex_t v = q;

    while(v.id != qInit.id){
      pose.pose.position.x = v.pose[0]*l_cm_resolution_;
      pose.pose.position.y = v.pose[1]*l_cm_resolution_;
      pose.pose.orientation.z = sin(v.pose[2]/2);
      pose.pose.orientation.w = cos(v.pose[2]/2);
      vel.linear.x = v.vel[0];
      vel.angular.z = v.vel[1];
      pose.header.frame_id = global_frame_.c_str();
      path.emplace(path.begin(), pose);
      cmd.emplace(cmd.begin(), vel);
      v = tree[v.pid];
    }

    path_and_cmd.path = path;
    path_and_cmd.cmd = cmd;

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

        twig = {cntID, {x, y, th}, parent, {v*l_cm_resolution_, vth}};
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

/********************************************************************************************************************/
  void RRTPlanner::velocityManager(){
    ros::WallTime current_time, previous_time, initial_time;
    if(path_and_cmd_.cmd.size() > 0){
      cmd_vel_pub_.publish(path_and_cmd_.cmd[0]);
      path_and_cmd_.cmd.erase(path_and_cmd_.cmd.begin());
    }
    else{
      geometry_msgs::Twist stop;
      stop.linear.x = 0.0;
      stop.linear.y = 0.0;
      stop.linear.z = 0.0;
      stop.angular.x = 0.0;
      stop.angular.y = 0.0;
      stop.angular.z = 0.0;
      cmd_vel_pub_.publish(stop);
    }
    initial_time = ros::WallTime::now();
    previous_time = ros::WallTime::now();
    while(path_and_cmd_.cmd.size() > 0){
      current_time = ros::WallTime::now();
      if((current_time.toSec() - previous_time.toSec()) >= time_step_){
        cmd_vel_pub_.publish(path_and_cmd_.cmd[0]);
        path_and_cmd_.cmd.erase(path_and_cmd_.cmd.begin());
        //ROS_INFO("TIME PASSED: %f", (current_time.toSec() - previous_time.toSec()));
        previous_time = ros::WallTime::now();
      }
      if(double(current_time.toSec() - initial_time.toSec()) >= double(1/controller_frequency_)){
        break;
      }
    }
  }
}
