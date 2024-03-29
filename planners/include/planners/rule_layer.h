#ifndef RULE_LAYER_H
#define RULE_LAYER_H
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/tf.h>
#include <tf2/convert.h>
#include <tf2/utils.h>

namespace rule_layer_namespace
{
  class RuleLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D
  {
    public:
      RuleLayer();

      virtual void onInitialize();
      virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y);
      virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
      bool isDiscretized()
      {
        return true;
      }
      virtual void matchSize();

    private:
      void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
      dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;

      int map_width_;
      int map_height_;
      double robot_yaw_;
      double robot_x_;
      double robot_y_;

  };
}
#endif // RULE_LAYER_H
