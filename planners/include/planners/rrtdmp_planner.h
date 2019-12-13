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
      int obstacle_inflation_radius_;
      base_local_planner::LocalPlannerUtil planner_util_;
      base_local_planner::OdometryHelperRos odom_helper_;
      std::string odom_topic_;
      std::vector<geometry_msgs::PoseStamped> global_plan_;
      std::string global_frame_;
      bool reached_goal_;
      geometry_msgs::PoseStamped goal_pose_;
      int robot_radius_;
      int goal_tolerance_;
      int path_tolerance_;
      int path_checkpoint_resolution_;
      std::vector<velocity_t> motion_primitive_array_;
      std::vector<velocity_t> motion_primitive_array_const_;
      double linear_velocity_max_;
      double angular_velocity_max_;
      double linear_velocity_min_;
      double angular_velocity_min_;
      int motion_primitive_resolution_;
      double time_step_;
      int num_step_;
      int max_iterations_;
      std::vector<vertex_t> tree_;
      nav_msgs::OccupancyGrid rrt_local_costmap_;
      unsigned int l_cm_width_;
      unsigned int l_cm_height_;
      double l_cm_resolution_;
      double l_cm_pose_x_;
      double l_cm_pose_y_;
      bool l_cm_border_;
      std::vector<rrtdmp_planner::RRTDMPPlanner::obstacle_t> l_cm_obs_;
      ros_cmd_t path_and_cmd_;
      double controller_frequency_;
      int forward_bias_;
      std::string velocity_topic_;


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
      static geometry_msgs::PoseArray people_;
      double rule_offset_;
      double goal_offset_;
      double person_offset_;
      double rule_scale_;
      double goal_scale_;
      double person_scale_;
      double rule_weight_;
      double goal_weight_;
      double person_weight_;
      double mp_range_scale_;
      int stop_loops_;
      geometry_msgs::Twist cmd_prev_;

      // define Dynamic Motion Primitive private member functions
      void socialForceModel(vertex_t Goal);
      force_t ruleForce();
      force_t goalForce(vertex_t Goal);
      force_t personForce();
      static void peopleCallback(const geometry_msgs::PoseArray& data);
  };
};

#endif // rrtdmp_planner_H
