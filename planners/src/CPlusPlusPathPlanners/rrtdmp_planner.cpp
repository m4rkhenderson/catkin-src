#include <ros/console.h>
#include <planners/rrtdmp_planner.h>
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

PLUGINLIB_EXPORT_CLASS(rrtdmp_planner::RRTDMPPlanner, nav_core::BaseLocalPlanner)

namespace rrtdmp_planner {

geometry_msgs::PoseArray RRTDMPPlanner::people_;

  // Public functions //


/********************************************************************************************************************/
  RRTDMPPlanner::RRTDMPPlanner() : initialized_(false), odom_helper_("odom"), tf_(NULL), costmap_ros_(NULL){

  }

/********************************************************************************************************************/
  RRTDMPPlanner::RRTDMPPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) :
    initialized_(false), odom_helper_("odom"), tf_(NULL), costmap_ros_(NULL){

    initialize(name, tf, costmap_ros);
  }

/********************************************************************************************************************/
  void RRTDMPPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros){
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
      private_nh.param("rule_offset", rule_offset_, 20.0);
      private_nh.param("goal_offset", goal_offset_, 0.0);
      private_nh.param("person_offset", person_offset_, 10.0);
      private_nh.param("rule_scale", rule_scale_, 1.0);
      private_nh.param("goal_scale", goal_scale_, 50.0);
      private_nh.param("person_scale", person_scale_, 2.0);
      private_nh.param("rule_weight", rule_weight_, 1.0);
      private_nh.param("goal_weight", goal_weight_, 1.0);
      private_nh.param("person_weight", person_weight_, 1.0);
      private_nh.param("avoid_ang", avoid_ang_, 0.5);
      private_nh.param("p_dist_weight", p_dist_weight_, 1.5);
      private_nh.param("p_ang_weight", p_ang_weight_, 0.5);
      private_nh.param("tilt_bias", tilt_bias_, 0.785);
      private_nh.param("mp_range_scale", mp_range_scale_, 0.125);
      private_nh.param("stop_loops", stop_loops_, 10);

      if(motion_primitive_resolution_ < 2){
        ROS_WARN("Motion Primitive Resolution must be greater than 1,"
                 " it has automatically been set to 2 to avoid an error.");
        motion_primitive_resolution_ = 2;
      }
      RRTDMPPlanner::velocity_t motion_primitive = {0, 0};
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
      f_rule_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("rule_force", 10);
      f_goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("goal_force", 10);
      f_person_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("person_force", 10);
      f_total_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("total_force", 10);

      people_sub_ = private_nh.subscribe("people", 10, RRTDMPPlanner::peopleCallback);

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
  bool RRTDMPPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan){
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
  bool RRTDMPPlanner::isGoalReached(){
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

    ROS_INFO("The Goal Has Not Been Reached Yet");
    return false;
  }

/********************************************************************************************************************/
  bool RRTDMPPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
    ROS_INFO("Computing Velocity Commands");
    if(! costmap_ros_ -> getRobotPose(current_pose_)){
      ROS_ERROR("Could not get robot pose");
      return false;
    }

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
      ROS_WARN_NAMED("rrtdmp_planner", "Received an empty transformed plan.");
      return false;
    }

    base_local_planner::prunePlan(current_pose_, transformed_plan, global_plan_); // Trim off parts of the global plan that are far enough behind the robot
    ROS_INFO("Received a transformed plan with %zu points.", transformed_plan.size());

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

    RRTDMPPlanner::vertex_t qInit = {0, {current_pose_.pose.position.x/(costmap_ -> getResolution()),
                                         current_pose_.pose.position.y/(costmap_ -> getResolution()),
                                         yaw1}, 0, {0, 0}};
    RRTDMPPlanner::vertex_t qGoal = {0, {transformed_plan[int(transformed_plan.size()/path_checkpoint_resolution_) -1].pose.position.x/(costmap_ -> getResolution()),
                                         transformed_plan[int(transformed_plan.size()/path_checkpoint_resolution_) -1].pose.position.y/(costmap_ -> getResolution()),
                                         yaw2}, 0, {0, 0}};

    path_and_cmd_ = RRTDMPPlanner::rrt(qInit, qGoal);
    ROS_INFO("Got path and velocity commands");

    base_local_planner::publishPlan(path_and_cmd_.path, l_plan_pub_);
    base_local_planner::publishPlan(transformed_plan, g_plan_pub_);

    RRTDMPPlanner::velocityManager();

    return true;
  }

/********************************************************************************************************************/
  RRTDMPPlanner::~RRTDMPPlanner(){ // clean up stuff
    delete &people_;
  }

  // Private functions //


/********************************************************************************************************************/
  RRTDMPPlanner::ros_cmd_t RRTDMPPlanner::rrt(RRTDMPPlanner::vertex_t qInit, RRTDMPPlanner::vertex_t qGoal){ // change to only activate after a set period of time to maintain controller frequency
    RRTDMPPlanner::vertex_t qRand;
    RRTDMPPlanner::vertex_t qNear;
    RRTDMPPlanner::vertex_t qNew;
    std::vector<RRTDMPPlanner::vertex_t> branch;
    RRTDMPPlanner::ros_cmd_t path_and_cmd;
    geometry_msgs::PoseStamped goalCheck = goal_pose_;
    geometry_msgs::Pose point;
    geometry_msgs::PoseArray points;
    std::vector<geometry_msgs::Pose> tempPoints;
    points.header.frame_id = global_frame_.c_str();
    ros::WallTime current_time, previous_time;

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

    ROS_INFO("RRT Planner Called");
    while(path_found == false && iterations < max_iterations_){
      iterations++;

      current_time = ros::WallTime::now();
      if(path_and_cmd_.cmd.size() > 0 && (current_time.toSec() - previous_time.toSec()) >= time_step_){
        //ROS_INFO("TIME PASSED: %f", (current_time - previous_time).toSec());
        cmd_vel_pub_.publish(path_and_cmd_.cmd[0]);
        cmd_prev_ = path_and_cmd_.cmd[0];
        //ROS_INFO("Velocity: %f", path_and_cmd_.cmd[0].linear.x);
        path_and_cmd_.cmd.erase(path_and_cmd_.cmd.begin());
        previous_time = ros::WallTime::now();
      }

      if(base_local_planner::getGoalPose(*tf_, global_plan_, global_frame_, goal_pose_)){ //reset rrt planner if
        if(goal_pose_.pose.position.x != goalCheck.pose.position.x ||                     //global goal changes
           goal_pose_.pose.position.y != goalCheck.pose.position.y){
          return path_and_cmd;
        }
      }

      RRTDMPPlanner::localCostmap();
      RRTDMPPlanner::socialForceModel(qGoal); // call on dynamic motion primitive selector

      qRand = RRTDMPPlanner::sampling(l_cm_pose_x_ + l_cm_width_, l_cm_pose_y_ + l_cm_height_,
                                   l_cm_pose_x_, l_cm_pose_y_);

      if(RRTDMPPlanner::collisionTest(qRand, l_cm_obs_, robot_radius_) > 0){
        //ROS_INFO("Invalid Random Configuration");
        continue;
      }

      qNear = RRTDMPPlanner::nearestNeighbour(qRand, tree_);

      branch = RRTDMPPlanner::motionPrimitives(qNear, l_cm_obs_, cntID, num_step_, time_step_,
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
          path_and_cmd = RRTDMPPlanner::extractPath(branch[i], tree_, qInit);
          break;
        }
      }
    }

    return path_and_cmd;
  }

/********************************************************************************************************************/
  void RRTDMPPlanner::localCostmap(){
    costmap_ = costmap_ros_ -> getCostmap();
    if(! costmap_){
      ROS_ERROR("Could not get costmap pointer");
    }

    rrtdmp_planner::RRTDMPPlanner::obstacle_t obs = {{0, 0}, 0};
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

//    for(int i=0; i<motion_primitive_array_.size(); i++){ // convert from m/s to cell/s
//      motion_primitive_array_[i].v = motion_primitive_array_[i].v/l_cm_resolution_;
//    }
  }
/********************************************************************************************************************/
  bool RRTDMPPlanner::collisionTest(RRTDMPPlanner::vertex_t q, std::vector<RRTDMPPlanner::obstacle_t> obs, double radius){
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
  RRTDMPPlanner::ros_cmd_t RRTDMPPlanner::extractPath(RRTDMPPlanner::vertex_t q, std::vector<RRTDMPPlanner::vertex_t> tree,
                                         RRTDMPPlanner::vertex_t qInit){

    std::vector<geometry_msgs::PoseStamped> path;
    geometry_msgs::PoseStamped pose;
    std::vector<geometry_msgs::Twist> cmd;
    geometry_msgs::Twist vel;
    RRTDMPPlanner::ros_cmd_t path_and_cmd;
    RRTDMPPlanner::vertex_t v = q;

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
  RRTDMPPlanner::vertex_t RRTDMPPlanner::nearestNeighbour(RRTDMPPlanner::vertex_t q, std::vector<RRTDMPPlanner::vertex_t> tree){
    RRTDMPPlanner::vertex_t v = {0, {0, 0, 0}, 0, {0, 0}};
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
  RRTDMPPlanner::vertex_t RRTDMPPlanner::sampling(int xMax, int yMax, int xMin, int yMin){
    RRTDMPPlanner::vertex_t qRand = {0, {double(rand()%(xMax - xMin) + xMin), double(rand()%(yMax - yMin) + yMin),
                                      double(rand()%628)/100}, 0, {0, 0}};
    return qRand;
  }

/********************************************************************************************************************/
  RRTDMPPlanner::vertex_t RRTDMPPlanner::steering(RRTDMPPlanner::vertex_t q, RRTDMPPlanner::vertex_t qNear, int cntID,
                                            double dMax, double aMax){

    RRTDMPPlanner::vertex_t qNew = {0, {0, 0, 0}, 0, {0, 0}};
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
  std::vector<RRTDMPPlanner::vertex_t> RRTDMPPlanner::motionPrimitives(RRTDMPPlanner::vertex_t q,
                                                                 std::vector<RRTDMPPlanner::obstacle_t> obs,
                                                                 int cntID, int nSteps, double tStep,
                                                                 std::vector<RRTDMPPlanner::velocity_t> mpArray,
                                                                 double radius){

    std::vector<RRTDMPPlanner::vertex_t> branch;
    RRTDMPPlanner::vertex_t twig = {0, {0, 0, 0}, 0, {0, 0}};
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
        if(RRTDMPPlanner::collisionTest(twig, obs, radius) > 0){
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
  void RRTDMPPlanner::velocityManager(){
    ros::WallTime current_time, previous_time, initial_time;
    if(path_and_cmd_.cmd.size() > 0){
      cmd_vel_pub_.publish(path_and_cmd_.cmd[0]);
      cmd_prev_ = path_and_cmd_.cmd[0];
      //ROS_INFO("Velocity: %f", path_and_cmd_.cmd[0].linear.x);
      path_and_cmd_.cmd.erase(path_and_cmd_.cmd.begin());
    }
    else{
      geometry_msgs::Twist stop;
      for(int i=0; i<stop_loops_; i++){ // produce a set of motion commands to create a smooth stop for the robot
        stop.linear.x = cmd_prev_.linear.x - cmd_prev_.linear.x/(stop_loops_-i);
        stop.linear.y = cmd_prev_.linear.y - cmd_prev_.linear.y/(stop_loops_-i);
        stop.linear.z = cmd_prev_.linear.z - cmd_prev_.linear.z/(stop_loops_-i);
        stop.angular.x = cmd_prev_.angular.x - cmd_prev_.angular.x/(stop_loops_-i);
        stop.angular.y = cmd_prev_.angular.y - cmd_prev_.angular.y/(stop_loops_-i);
        stop.angular.z = cmd_prev_.angular.z - cmd_prev_.angular.z/(stop_loops_-i);
        path_and_cmd_.cmd.push_back(stop);
      }
      cmd_vel_pub_.publish(path_and_cmd_.cmd[0]);
      cmd_prev_ = path_and_cmd_.cmd[0];
      //ROS_INFO("Velocity: %f", path_and_cmd_.cmd[0].linear.x);
      path_and_cmd_.cmd.erase(path_and_cmd_.cmd.begin());
    }
    initial_time = ros::WallTime::now();
    previous_time = ros::WallTime::now();
    while(path_and_cmd_.cmd.size() > 0){
      current_time = ros::WallTime::now();
      if((current_time.toSec() - previous_time.toSec()) >= time_step_){
        cmd_vel_pub_.publish(path_and_cmd_.cmd[0]);
        cmd_prev_ = path_and_cmd_.cmd[0];
        //ROS_INFO("Velocity: %f", path_and_cmd_.cmd[0].linear.x);
        path_and_cmd_.cmd.erase(path_and_cmd_.cmd.begin());
        //ROS_INFO("TIME PASSED: %f", (current_time.toSec() - previous_time.toSec()));
        previous_time = ros::WallTime::now();
      }
      if(double(current_time.toSec() - initial_time.toSec()) >= double(1/controller_frequency_)){
        break;
      }
    }
  }


    // DMP Functions //

  /********************************************************************************************************************/
  void RRTDMPPlanner::socialForceModel(RRTDMPPlanner::vertex_t Goal){
    RRTDMPPlanner::force_t f_rule;
    RRTDMPPlanner::force_t f_goal;
    RRTDMPPlanner::force_t f_person;
    RRTDMPPlanner::force_t force;
    double force_x;
    double force_y;
    double v_norm;
    double vth_norm;
    double v_st_dev;
    v_st_dev = 2*linear_velocity_max_/motion_primitive_resolution_;
    double vth_st_dev;
    vth_st_dev = 2*angular_velocity_max_/motion_primitive_resolution_;

    f_rule = ruleForce();
    f_goal = goalForce(Goal);
    f_person = personForce();
    ROS_INFO("f_rule: %f, f_goal: %f, f_person: %f", f_rule.magnitude, f_goal.magnitude, f_person.magnitude);
    //ROS_INFO("th_rule: %f, th_goal: %f, th_person: %f", f_rule.angle, f_goal.angle, f_person.angle);
    force_x = (rule_weight_*f_rule.magnitude*cos(f_rule.angle) *
               (1-person_weight_*f_person.magnitude) + //as the person force gets larger, the rule force is less important
               goal_weight_*f_goal.magnitude*cos(f_goal.angle) *
               (1-0*person_weight_*f_person.magnitude) +
               person_weight_*f_person.magnitude*cos(f_person.angle) *
               (1+person_weight_*f_person.magnitude))/3; //as the person force gets larger, the person force gets more important
    force_y = (rule_weight_*f_rule.magnitude*sin(f_rule.angle) *
               (1-person_weight_*f_person.magnitude) +
               goal_weight_*f_goal.magnitude*sin(f_goal.angle) *
               (1-0*person_weight_*f_person.magnitude) +
               person_weight_*f_person.magnitude*sin(f_person.angle) *
               (1+person_weight_*f_person.magnitude))/3;

    ROS_INFO("force_x: %f, force_y: %f", force_x, force_y);

    //------------ only used for publishing pose ------------//
    if(! costmap_ros_ -> getRobotPose(current_pose_)){
      ROS_ERROR("Could not get robot pose");
    }
    tf::Quaternion q(current_pose_.pose.orientation.x,
                     current_pose_.pose.orientation.y,
                     current_pose_.pose.orientation.z,
                     current_pose_.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    force.magnitude = sqrt(force_x*force_x + force_y*force_y);
    force.angle = atan2(force_y,force_x);
    ///////////////////////////////////////////////////////////

    v_norm = linear_velocity_max_*force_x; // normalize to v range
    //ROS_INFO("v_norm: %f", v_norm);
    if(v_norm > linear_velocity_max_){
      v_norm = linear_velocity_max_;
//      v_min = linear_velocity_max_ - 2*(linear_velocity_max_-linear_velocity_min_)*mp_range_scale_;
    }
    else if(v_norm < -linear_velocity_max_){
      v_norm = -linear_velocity_max_;
//      v_max = -linear_velocity_max_ + 2*(linear_velocity_max_-linear_velocity_min_)*mp_range_scale_;
    }
//    else{
//      v_max = v_norm + 2*(linear_velocity_max_-linear_velocity_min_)*mp_range_scale_;
//      v_min = v_norm - 2*(linear_velocity_max_-linear_velocity_min_)*mp_range_scale_;
//    }

    vth_norm = angular_velocity_max_*force_y; // normalize to vth range
    //ROS_INFO("vth_norm: %f", vth_norm);
    if(vth_norm > angular_velocity_max_){
      vth_norm = angular_velocity_max_;
//      vth_min = angular_velocity_max_ - 2*(angular_velocity_max_-angular_velocity_min_)*mp_range_scale_;
    }
    else if(vth_norm < -angular_velocity_max_){
      vth_norm = -angular_velocity_max_;
//      vth_max = -angular_velocity_max_ + 2*(angular_velocity_max_-angular_velocity_min_)*mp_range_scale_;
    }
//    else{
//      vth_max = vth_norm + 2*(angular_velocity_max_-angular_velocity_min_)*mp_range_scale_;
//      vth_min = vth_norm - 2*(angular_velocity_max_-angular_velocity_min_)*mp_range_scale_;
//    }

    // Start choosing MP Array
    motion_primitive_array_const_.clear(); // reset the MP Array

    RRTDMPPlanner::velocity_t motion_primitive = {0, 0};
    for(int i=0; i<motion_primitive_resolution_+1; i++){
      motion_primitive.v = ((linear_velocity_max_*2.0)*(2.0*double(i)*v_st_dev-2.0*linear_velocity_max_)*(1.0/(v_st_dev*sqrt(2.0*3.14)))*
                            exp(-0.5*pow(((2.0*double(i)*v_st_dev-2.0*linear_velocity_max_)/v_st_dev), 2))+v_norm)/l_cm_resolution_;
      //ROS_INFO("v: %f", motion_primitive.v);
      if(motion_primitive.v > linear_velocity_max_/l_cm_resolution_){
        motion_primitive.v = linear_velocity_max_/l_cm_resolution_;
      }
      else if(motion_primitive.v < -linear_velocity_max_/l_cm_resolution_){
        motion_primitive.v = -linear_velocity_max_/l_cm_resolution_;
      }
      for(int j=0; j<motion_primitive_resolution_+1; j++){
            //((i+1)*((v_max - v_min)/(motion_primitive_resolution_))
            //+ v_min)/l_cm_resolution_;
            //(i*((v_max - v_min)/(motion_primitive_resolution_ - 1))
            //+ v_min)/l_cm_resolution_;  // will make identical motion primitives if max = min
        motion_primitive.vth = ((angular_velocity_max_*2.0)*(2.0*double(j)*vth_st_dev-2.0*angular_velocity_max_)*(1.0/(vth_st_dev*sqrt(2.0*3.14)))*
                                exp(-0.5*pow(((2.0*double(j)*vth_st_dev-2.0*angular_velocity_max_)/vth_st_dev), 2))+vth_norm);
        //ROS_INFO("vth: %f", motion_primitive.vth);
        if(motion_primitive.vth > angular_velocity_max_){
          motion_primitive.vth = angular_velocity_max_;
        }
        else if(motion_primitive.vth < -angular_velocity_max_){
          motion_primitive.vth = -angular_velocity_max_;
        }
            //((j+1)*((vth_max - vth_min)/(motion_primitive_resolution_))
            //+ vth_min);
            //(j*((vth_max - vth_min)/(motion_primitive_resolution_ - 1))
            //+ vth_min); // will make identical motion primitives if max = min
        motion_primitive_array_const_.push_back(motion_primitive);
        //ROS_INFO("MP: [%f, %f]", motion_primitive.v, motion_primitive.vth);
      }
    }

    geometry_msgs::PoseStamped pose; // stuff for visualizing the force
    pose.pose.position.x = current_pose_.pose.position.x;
    pose.pose.position.y = current_pose_.pose.position.y;
    pose.pose.position.z = 0.0;
    geometry_msgs::Quaternion quaternion;
    double angle;
    angle = force.angle + yaw;
    quaternion = tf::createQuaternionMsgFromYaw(angle);
    pose.pose.orientation = quaternion;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "/map";

    f_total_pub_.publish(pose);

  }

  /********************************************************************************************************************/
  RRTDMPPlanner::force_t RRTDMPPlanner::ruleForce(){
    RRTDMPPlanner::force_t force;
    std::vector<RRTDMPPlanner::force_t> force_array;
    double force_x = 0.0;
    double force_y = 0.0;

    if(! costmap_ros_ -> getRobotPose(current_pose_)){
      ROS_ERROR("Could not get robot pose");
      force.magnitude = 0.0;
      force.angle = 0.0;
      return force;
    }
    tf::Quaternion q(current_pose_.pose.orientation.x,
                     current_pose_.pose.orientation.y,
                     current_pose_.pose.orientation.z,
                     current_pose_.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    double rob_x = current_pose_.pose.position.x/l_cm_resolution_;
    double rob_y = current_pose_.pose.position.y/l_cm_resolution_;

    double obs_x;
    double obs_y;
    double obs_th;
    double obs_m;

    for(int i=0; i<l_cm_obs_.size(); i++){
      obs_x = l_cm_obs_[i].pose[0] - rob_x;
      obs_y = l_cm_obs_[i].pose[1] - rob_y;
      obs_m = sqrt(obs_x*obs_x + obs_y*obs_y);
      obs_th = atan2(obs_y,obs_x) - yaw;
      obs_y = obs_m*sin(obs_th);
      obs_x = obs_m*cos(obs_th);

      if(obs_y < 0 && obs_x > 0){ // obstacle is on the right
        force.magnitude = 2/(1+exp(-1*(fabs(obs_m)-rule_offset_)/rule_scale_))-1; // adjusted sigmoid 1/(1+e^-x) // it's centered on 20 cells
        force.angle = obs_th; // angle of the vector
        force_array.push_back(force);
      }
    }

    if(force_array.size() > 0){
      for(int i=0; i<force_array.size(); i++){
        force_x += force_array[i].magnitude*cos(force_array[i].angle);
        force_y += force_array[i].magnitude*sin(force_array[i].angle);
      }

      force_x /= force_array.size();
      force_y /= force_array.size();

      force.magnitude = sqrt(force_x*force_x + force_y*force_y);
      force.angle = atan2(force_y,force_x);
    }
    else{
      force.magnitude = 0.0;
      force.angle = 0.0;
    }

    geometry_msgs::PoseStamped pose; // stuff for visualizing the force
    pose.pose.position.x = current_pose_.pose.position.x;
    pose.pose.position.y = current_pose_.pose.position.y;
    pose.pose.position.z = 0.0;
    geometry_msgs::Quaternion quaternion;
    double angle;
    angle = force.angle + yaw;
    quaternion = tf::createQuaternionMsgFromYaw(angle);
    pose.pose.orientation = quaternion;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "/map";

    f_rule_pub_.publish(pose);

    return force;
  }

  /********************************************************************************************************************/
  RRTDMPPlanner::force_t RRTDMPPlanner::goalForce(RRTDMPPlanner::vertex_t Goal){
    RRTDMPPlanner::force_t force;

    if(! costmap_ros_ -> getRobotPose(current_pose_)){
      ROS_ERROR("Could not get robot pose");
      force.magnitude = 0.0;
      force.angle = 0.0;
      return force;
    }
    tf::Quaternion q(current_pose_.pose.orientation.x,
                     current_pose_.pose.orientation.y,
                     current_pose_.pose.orientation.z,
                     current_pose_.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    double rob_x = current_pose_.pose.position.x/l_cm_resolution_;
    double rob_y = current_pose_.pose.position.y/l_cm_resolution_;

    double goal_x = Goal.pose[0] - rob_x;
    double goal_y = Goal.pose[1] - rob_y;
    double goal_m = sqrt(goal_x*goal_x + goal_y*goal_y);

    force.magnitude = 2/(1+exp(-1*(fabs(goal_m)-goal_offset_)/goal_scale_))-1; // adjusted sigmoid 1/(1+e^-x) // it's centered on 0 cells -> reaches 1 at approximately 415 cells
    force.angle = atan2(goal_y,goal_x) - yaw;

    geometry_msgs::PoseStamped pose; // stuff for visualizing the force
    pose.pose.position.x = current_pose_.pose.position.x;
    pose.pose.position.y = current_pose_.pose.position.y;
    pose.pose.position.z = 0.0;
    geometry_msgs::Quaternion quaternion;
    double angle;
    angle = force.angle + yaw;
    quaternion = tf::createQuaternionMsgFromYaw(angle);
    pose.pose.orientation = quaternion;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "/map";

    f_goal_pub_.publish(pose);

    return force;
  }

  /********************************************************************************************************************/
  RRTDMPPlanner::force_t RRTDMPPlanner::personForce(){
    RRTDMPPlanner::force_t force;
    std::vector<RRTDMPPlanner::force_t> force_array;
    double force_x = 0.0;
    double force_y = 0.0;

    if(! costmap_ros_ -> getRobotPose(current_pose_)){
      ROS_ERROR("Could not get robot pose");
      force.magnitude = 0.0;
      force.angle = 0.0;
      return force;
    }
    tf::Quaternion q(current_pose_.pose.orientation.x,
                     current_pose_.pose.orientation.y,
                     current_pose_.pose.orientation.z,
                     current_pose_.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    double rob_x = current_pose_.pose.position.x/l_cm_resolution_;
    double rob_y = current_pose_.pose.position.y/l_cm_resolution_;

    double per_x;
    double per_y;
    double per_th;
    double per_m;
    double density;

    //ROS_INFO("# people: %d", people_.poses.size());
    //ROS_INFO("people pose: x:%f, y:%f", people_.poses[0].position.x, people_.poses[0].position.y);

    if(people_.poses.size() > 1){ // calculate environment density based on human proximity to each other
      double peri_x;
      double perj_x;
      double peri_y;
      double perj_y;
      double peri_m;
      double perj_m;
      for(int i=0; i<people_.poses.size(); i++){
        for(int j=0; j<people_.poses.size(); j++){
          if(i == j){
            continue;
          }
          peri_x = people_.poses[i].position.x/l_cm_resolution_;
          perj_x = people_.poses[j].position.x/l_cm_resolution_;
          peri_y = people_.poses[i].position.y/l_cm_resolution_;
          perj_y = people_.poses[j].position.y/l_cm_resolution_;
//          peri_m = sqrt(peri_x*peri_x + peri_y*peri_y);
//          perj_m = sqrt(perj_x*perj_x + perj_y*perj_y);
          density += 1/(1+exp(((((fabs(perj_x-peri_x)/20)+(fabs(perj_y-peri_y)/20))/2)-0.5)/0.1)); // density is a sigmoid function of the difference in x and y of the people
        }
      }
//      density /= people_.poses.size();
//      ROS_INFO("Density: %f", density);
      if(density < 1){
        density = 1;
      }
    }
    else{
      density = 1;
    }

    if(people_.poses.size() > 0){
      for(int i=0; i<people_.poses.size(); i++){
        per_x = people_.poses[i].position.x/l_cm_resolution_;// - rob_x;
        per_y = people_.poses[i].position.y/l_cm_resolution_;// - rob_y;
        if(per_x < 0){ // ignore people behind the robot
          continue;
        }

        per_th = people_.poses[i].orientation.z;
        per_m = sqrt(per_x*per_x + per_y*per_y);
        //ROS_INFO("per_x:%f, per_y:%f, per_m:%f", per_x, per_y, per_m);

        force.magnitude = 1/(1+exp(-1*(fabs(per_m)-person_offset_*density)/person_scale_))-1;// adjusted sigmoid 1/(1+e^-x) // offset is a function of the environment density <- maybe not
        //force.magnitude = (p_dist_weight_*force.magnitude + p_ang_weight_*((atan2((3.14-per_th)*(3.14-per_th),1)/1.5)-1))/2;// average the force from the person's proximity and heading

        force.angle = atan2(per_y,per_x);
        if(force.angle < 1.57 && force.angle > -1.57){
          force.angle = force.angle + -1*tilt_bias_*((1.25332/(0.5*sqrt(2*3.14)))*exp(-0.5*pow((force.angle-avoid_ang_)/0.5, 2)));// tilt force away from the person the closer the robot is to it // 0.785rad ~=45deg // create a gradiant of effect using gaussian
        }
        //ROS_INFO("magnitude: %f", force.magnitude);
        force_array.push_back(force);
      }

      //ROS_INFO("force_array.size() = %d", force_array.size());

      if(force_array.size() <= 0){ // a final check to see if the robot has ignored all people around it
        force.magnitude = 0.0;
        force.angle = 0.0;
        return force;
      }

      for(int i=0; i<force_array.size(); i++){ // get sum of x and y forces
        force_x += force_array[i].magnitude*cos(force_array[i].angle);
        force_y += force_array[i].magnitude*sin(force_array[i].angle);
      }

      if(fabs(force_x) > 0){ // if the force is not zero then produce an average force based on impact of each person
        double force_x_sum = force_x;
        double force_x_i;
        force_x = 0;
        for(int i=0; i<force_array.size(); i++){
          force_x_i = force_array[i].magnitude*cos(force_array[i].angle);
          force_x += (fabs(force_x_i)/force_x_i)*pow(force_x_i,2)/fabs(force_x_sum); // multiplied by the signage of the force
        }
      }
      if(fabs(force_y) > 0){ // if the force is not zero then produce an average force based on impact of each person
        double force_y_sum = force_y;
        double force_y_i;
        force_y = 0;
        for(int i=0; i<force_array.size(); i++){
          force_y_i = force_array[i].magnitude*sin(force_array[i].angle);
          force_y += (fabs(force_y_i)/force_y_i)*pow(force_y_i,2)/fabs(force_y_sum); // multiplied by the signage of the force
        }
      }
//      force_x /= force_array.size();
//      force_y /= force_array.size();
      //ROS_INFO("force_x: %f, force_y: %f", force_x, force_y);

      force.magnitude = sqrt(force_x*force_x + force_y*force_y);
      force.angle = atan2(force_y,force_x);
    }
    else{
      ROS_INFO("No People Detected");
      force.magnitude = 0.0;
      force.angle = 0.0;
    }

    geometry_msgs::PoseStamped pose; // stuff for visualizing the force
    pose.pose.position.x = current_pose_.pose.position.x;
    pose.pose.position.y = current_pose_.pose.position.y;
    pose.pose.position.z = 0.0;
    geometry_msgs::Quaternion quaternion;
    double angle;
    angle = force.angle + yaw;
    quaternion = tf::createQuaternionMsgFromYaw(angle);
    pose.pose.orientation = quaternion;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "/map";

    f_person_pub_.publish(pose);

    return force;
  }
  void RRTDMPPlanner::peopleCallback(const geometry_msgs::PoseArray& data){
    people_ = data;
  }
}
