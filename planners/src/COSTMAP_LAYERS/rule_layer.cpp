#include <planners/rule_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(rule_layer_namespace::RuleLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace rule_layer_namespace{

  RuleLayer::RuleLayer(){}

  void RuleLayer::onInitialize(){
    ros::NodeHandle nh("~/" + name_);
    current_ = true;
    default_value_ = NO_INFORMATION;
    matchSize();

    nh.param("map_width", map_width_, 240);
    nh.param("map_height", map_height_, 240);

    dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(&RuleLayer::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
  }

  void RuleLayer::matchSize(){
    Costmap2D* master = layered_costmap_->getCostmap();
    resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(), master->getOriginX(), master->getOriginY());
  }

  void RuleLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level){
    enabled_ = config.enabled;
  }

  void RuleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y){
    if(!enabled_){
      return;
    }
    Costmap2D* master = layered_costmap_->getCostmap();

//    *min_x = (robot_x/(master->getResolution()))-(map_width_/2)+(master->getSizeInCellsX())/2;
//    *min_y = (robot_y/(master->getResolution()))-(map_height_/2)+(master->getSizeInCellsY())/2;
//    *max_x = (robot_x/(master->getResolution()))+(map_width_/2)+(master->getSizeInCellsX())/2;
//    *max_y = (robot_y/(master->getResolution()))+(map_height_/2)+(master->getSizeInCellsY())/2;

    *min_x = (master->getOriginX())-(master->getSizeInCellsX())/2;
    *min_y = (master->getOriginY())-(master->getSizeInCellsY())/2;
    *max_x = (master->getOriginX())+(master->getSizeInCellsX())/2;
    *max_y = (master->getOriginY())+(master->getSizeInCellsY())/2;

    robot_yaw_ = robot_yaw;
    robot_x_ = robot_x;
    robot_y_ = robot_y;
  }

  void RuleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j){
    if(!enabled_){
      return;
    }

    Costmap2D* master = layered_costmap_->getCostmap();

    robot_x_ = (robot_x_/(master->getResolution()))+(master->getSizeInCellsX())/2;
    robot_y_ = (robot_y_/(master->getResolution()))+(master->getSizeInCellsY())/2;

    min_i = robot_x_-(map_width_/2);
    min_j = robot_y_-(map_height_/2);
    max_i = robot_x_+(map_width_/2);
    max_j = robot_y_+(map_height_/2);


    for(int j=min_j; j<max_j; j++){
      for(int i=min_i; i<max_i; i++){
        double d = sqrt((i-robot_x_)*(i-robot_x_)+
                        (j-robot_y_)*(j-robot_y_));
        double th = atan2((j-robot_y_),(i-robot_x_))-robot_yaw_;

        double x = -d*sin(th);
        //ROS_INFO("X: %d", x);
        int cost = 253/(1+exp(x/(map_width_/5)));
        //ROS_INFO("Cost: %d", cost);

        if(cost <= 0 || cost <= (master->getCost(i,j))){
          continue;
        }
        else if(cost > 253){
          cost = 253;
        }
        master_grid.setCost(i, j, cost);
      }
    }
  }
}
