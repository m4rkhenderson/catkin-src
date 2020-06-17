#ifndef RRTDMP_PLANNER_H
#define RRTDMP_PLANNER_H

#include <ros/ros.h>
#include <vector>
#include <nav_msgs/Path.h>
#include <tf2_ros/buffer.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float64.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <base_local_planner/local_planner_util.h>
#include <nav_core/base_local_planner.h>

namespace rrtdmp_planner{

  class RRTDMPPlanner : public nav_core::BaseLocalPlanner{
    public:

      RRTDMPPlanner();

      RRTDMPPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

      void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

      bool isGoalReached();

      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);

      ~RRTDMPPlanner();

      // define RRT data structures
      struct vertex_t{
        int id;
        double pose[3];
        int pid;
        double vel[2];
        double cost;
      };
      struct obstacle_t{
        double pose[2];
        double radius;
      };
      struct velocity_t{
        double v;
        double vth;
      };
      struct ros_cmd_t{
        std::vector<geometry_msgs::PoseStamped> path;
        std::vector<geometry_msgs::Twist> cmd;
        double cost;
      };
      struct force_t{
        double magnitude;
        double angle;
      };

    private:

      // define private RRT variables
      bool initialized_;
      tf2_ros::Buffer* tf_;
      ros::Publisher l_plan_pub_;
      ros::Publisher g_plan_pub_;
      ros::Publisher cmd_vel_pub_;
      ros::Publisher tree_pub_;
      costmap_2d::Costmap2DROS* costmap_ros_;
      costmap_2d::Costmap2D* costmap_;
      geometry_msgs::PoseStamped current_pose_;
      std::vector<int> obstacle_inflation_radius_;
      base_local_planner::LocalPlannerUtil planner_util_;
      base_local_planner::OdometryHelperRos odom_helper_;
      std::string odom_topic_;
      std::vector<geometry_msgs::PoseStamped> global_plan_;
      std::string global_frame_;
      bool reached_goal_;
      geometry_msgs::PoseStamped goal_pose_;
      std::vector<int> robot_radius_;
      std::vector<int> goal_tolerance_;
      std::vector<int> path_tolerance_;
      std::vector<int> path_checkpoint_resolution_;
      std::vector<velocity_t> motion_primitive_array_;
      std::vector<velocity_t> motion_primitive_array_const_;
      std::vector<double> linear_velocity_max_;
      std::vector<double> angular_velocity_max_;
      std::vector<double> linear_velocity_min_;
      std::vector<double> angular_velocity_min_;
      std::vector<int> motion_primitive_resolution_;
      std::vector<double> time_step_;
      std::vector<int> num_step_;
      std::vector<int> max_iterations_;
      std::vector<vertex_t> tree_;
      nav_msgs::OccupancyGrid rrt_local_costmap_;
      unsigned int l_cm_width_;
      unsigned int l_cm_height_;
      double l_cm_resolution_;
      double l_cm_pose_x_;
      double l_cm_pose_y_;
      std::vector<bool> l_cm_border_;
      std::vector<rrtdmp_planner::RRTDMPPlanner::obstacle_t> l_cm_obs_;
      ros_cmd_t path_and_cmd_;
      std::vector<double> controller_frequency_;
      std::vector<int> forward_bias_;
      std::string velocity_topic_;
      ros::WallTime current_time_, previous_time_;


      // define RRT private member functions
      ros_cmd_t rrt(vertex_t qInit, vertex_t qGoal);
      void localCostmap();
      bool collisionTest(vertex_t q, std::vector<obstacle_t> obs, double radius);
      ros_cmd_t extractPath(vertex_t q, std::vector<vertex_t> tree, vertex_t qInit);
      vertex_t nearestNeighbour(vertex_t q, std::vector<vertex_t> tree);
      vertex_t sampling(int xMax, int yMax, int xMin, int yMin);
      vertex_t steering(vertex_t q, vertex_t qNear, int cntID, double dMax, double aMax);
      std::vector<vertex_t> motionPrimitives(vertex_t q, std::vector<RRTDMPPlanner::obstacle_t> obs, int cntID, int nSteps, double tStep, std::vector<velocity_t> mpArray, double radius);
      void velocityManager();

      // define private Dynamic Motion Primitive variables
      ros::Subscriber people_sub_;
      ros::Publisher f_rule_pub_;
      ros::Publisher f_goal_pub_;
      ros::Publisher f_person_pub_;
      ros::Publisher f_total_pub_;
      ros::Publisher trajectory_pub_; // added for comparison in CCECE 2020 Paper
      ros::Publisher time_cost_pub_;
      ros::Publisher comparison_plan_pub_; // added for better comparison
      static geometry_msgs::PoseArray people_;
      std::vector<double> rule_offset_;
      std::vector<double> goal_offset_;
      std::vector<double> person_offset_;
      std::vector<double> rule_scale_;
      std::vector<double> goal_scale_;
      std::vector<double> person_scale_;
      std::vector<double> rule_weight_;
      std::vector<double> goal_weight_;
      std::vector<double> person_weight_;
      std::vector<double> avoid_ang_;
      std::vector<double> p_dist_weight_;
      std::vector<double> p_ang_weight_;
      std::vector<double> tilt_bias_;
      std::vector<double> mp_range_scale_;
      std::vector<int> stop_loops_;
      std::vector<double> linear_acceleration_max_;
      std::vector<double> angular_acceleration_max_;
      geometry_msgs::Twist cmd_prev_;
      nav_msgs::Path trajectory_;
      std_msgs::Float64 time_cost_;

      // new parameters
      static double person_time_;
      static ros::WallTime person_time_c_, person_time_p_;
      std::vector<double> people_p_;
      std::vector<double> rule_max_;
      std::vector<double> goal_max_;
      std::vector<double> person_max_;
      std::vector<int> failure_max_;
      int failures_;
      bool use_backup_;
      bool safety_;
      int safety_loops_;
      std::vector<bool> carlike_;
      double min_d_;
      std::vector<double> e_distance_;

      // define Dynamic Motion Primitive private member functions
      void socialForceModel(vertex_t Goal);
      force_t ruleForce();
      force_t goalForce(vertex_t Goal);
      force_t personForce();
      static void peopleCallback(const geometry_msgs::PoseArray& data);

      // define Parameter Selection private member function and parameters
      void selectParameters(double v_h); //currently unused
      int k_;
  };
};

#endif // rrtdmp_planner_H
