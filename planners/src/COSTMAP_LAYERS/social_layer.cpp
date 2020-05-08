#include <planners/social_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(social_layer_namespace::SocialLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace social_layer_namespace{
  geometry_msgs::PoseArray SocialLayer::people_;

  SocialLayer::SocialLayer(){}

  void SocialLayer::onInitialize(){
    ros::NodeHandle nh("~/" + name_);
    current_ = true;
    default_value_ = NO_INFORMATION;
    matchSize();

    nh.param("max_radius", max_radius_, 30);
    nh.param("distance_weight", distance_weight_, 1000.0);
    nh.param("angle_weight", angle_weight_, 10.0);
    nh.param("search_width", search_width_, 75);
    nh.param("search_height", search_height_, 75);

    people_sub_ = nh.subscribe("people", 10, SocialLayer::peopleCallback);
    dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(&SocialLayer::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
  }

  void SocialLayer::matchSize(){
    Costmap2D* master = layered_costmap_->getCostmap();
    resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(), master->getOriginX(), master->getOriginY());
  }

  void SocialLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level){
    enabled_ = config.enabled;
  }

  void SocialLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y){
    if(!enabled_){
      return;
    }

    Costmap2D* master = layered_costmap_->getCostmap();

    robot_x_ = robot_x;
    robot_y_ = robot_y;
    robot_yaw_ = robot_yaw;

//    double mark_x = robot_x + cos(robot_yaw), mark_y = robot_y + sin(robot_yaw);
//    unsigned int mx;
//    unsigned int my;
//    if(worldToMap(mark_x, mark_y, mx, my)){
//      setCost(mx, my, LETHAL_OBSTACLE);
//    }

//    *min_x = std::min(*min_x, mark_x);
//    *min_y = std::min(*min_y, mark_y);
//    *max_x = std::max(*max_x, mark_x);
//    *max_y = std::max(*max_y, mark_y);


    *min_x = (master->getOriginX())-(master->getSizeInCellsX())/2;
    *min_y = (master->getOriginY())-(master->getSizeInCellsY())/2;
    *max_x = (master->getOriginX())+(master->getSizeInCellsX())/2;
    *max_y = (master->getOriginY())+(master->getSizeInCellsY())/2;
    // customize
  }

  void SocialLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j){
    if(!enabled_){
      return;
    }
    if(!people_.poses.size()){ // if there are no people, don't update costs
      return;
    }

    Costmap2D* master = layered_costmap_->getCostmap(); // get reference to costmap for calculations

    for(int k=0; k<people_.poses.size(); k++){
      double person_d = sqrt(people_.poses[k].position.x*people_.poses[k].position.x+
                             people_.poses[k].position.y*people_.poses[k].position.y);
      double th_pr = atan2(people_.poses[k].position.y, people_.poses[k].position.x)+robot_yaw_;

      double person_x = (person_d*cos(th_pr)+robot_x_)/(master->getResolution()) + (master->getSizeInCellsX())/2;
      double person_y = (person_d*sin(th_pr)+robot_y_)/(master->getResolution()) + (master->getSizeInCellsY())/2;

      tf::Quaternion q(people_.poses[k].orientation.x,
                       people_.poses[k].orientation.y,
                       people_.poses[k].orientation.z,
                       people_.poses[k].orientation.w);
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      double person_th = yaw + robot_yaw_; // may need to add stuff to this

      //ROS_INFO("YAW: %f", robot_yaw_);
      // reduce search area for each costmap
      min_i = int(person_x - search_width_/2);
      min_j = int(person_y - search_height_/2);
      max_i = int(person_x + search_width_/2);
      max_j = int(person_y + search_height_/2);

      //ROS_INFO("X: %f, Y: %f", person_x, person_y);
      for(int j=min_j; j<max_j; j++){
        for(int i=min_i; i<max_i; i++){
          int index = getIndex(i, j);
          //ROS_INFO("i: %d, j: %d", i, j);
          //if(costmap_[index] == NO_INFORMATION){
          //  continue;
          //}
          double d = sqrt((i-person_x)*(i-person_x)+
                          (j-person_y)*(j-person_y));
          //ROS_INFO("Dist: %f", d);

          double th = atan2((j-person_y),(i-person_x))-person_th+3.14;
          if(th > 3.14){
            th = th-6.28;
          }
          else if(th < -3.14){
            th = th + 6.28;
          }

          int cost = ((distance_weight_/sqrt(2*3.14))*exp(-0.5*pow(d/(max_radius_/3),2)))*
                     (((angle_weight_/sqrt(2*3.14))*exp(-0.5*pow(th/(1.57/3),2)))+1); // 2-dimensional gaussian for generating cost values
          //ROS_INFO("Cost: %d", cost);
          if(cost <= 0 || cost <= (master->getCost(i,j))){
            continue;
          }
          else if(cost > 253){
            cost = 253;
          }
          master_grid.setCost(i, j, cost); // last parameter is cost
        }
      }
    }

    // customize
  }

  void SocialLayer::peopleCallback(const geometry_msgs::PoseArray& data){
    people_ = data;
  }
}
