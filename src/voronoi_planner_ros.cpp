#include <pluginlib/class_list_macros.hpp>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <string.h>
#include "voronoi_planner/voronoi_planner_ros.h"

PLUGINLIB_EXPORT_CLASS(voronoi_planner::VoronoiPlannerROS, nav_core::BaseGlobalPlanner)

namespace voronoi_planner
{
VoronoiPlannerROS::VoronoiPlannerROS()
  : voronoi_planner_(NULL), initialized_(false), costmap_ros_(NULL), size_x_(0), size_y_(0), voronoi_diagram_(NULL)
{
}

VoronoiPlannerROS::VoronoiPlannerROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  : voronoi_planner_(NULL), initialized_(false), costmap_ros_(NULL), size_x_(0), size_y_(0), voronoi_diagram_(NULL)
{
  initialize(name, costmap_ros);
}

VoronoiPlannerROS::~VoronoiPlannerROS()
{
  if (voronoi_planner_)
    delete voronoi_planner_;

  destroy();
}

void VoronoiPlannerROS::destroy()
{
  if (voronoi_diagram_)
  {
    for (int i = 0; i < size_x_; i++)
      delete[] voronoi_diagram_[i];
    delete[] voronoi_diagram_;
  }
}

void VoronoiPlannerROS::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  if (!initialized_)
  {
    ros::NodeHandle private_nh("~/" + name);

    ROS_DEBUG("Name is %s", name.c_str());

    name_ = name;
    costmap_ros_ = costmap_ros;

    voronoi_planner_ = new VoronoiPlanner();

    path_pub_ = private_nh.advertise<nav_msgs::Path>("voronoi_path", 1);

    initialized_ = true;
  }
}

bool VoronoiPlannerROS::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                                 std::vector<geometry_msgs::PoseStamped>& plan)
{
  if (!initialized_)
  {
    ROS_ERROR("voronoi_planner is not initialized");
    return false;
  }

  double circumscribed_radius = costmap_ros_->getLayeredCostmap()->getCircumscribedRadius();
  int size_x = costmap_ros_->getCostmap()->getSizeInCellsX();
  int size_y = costmap_ros_->getCostmap()->getSizeInCellsY();
  double resolution = costmap_ros_->getCostmap()->getResolution();
  double origin_x = costmap_ros_->getCostmap()->getOriginX();
  double origin_y = costmap_ros_->getCostmap()->getOriginY();

  if (size_x != size_x_ || size_y != size_y_)
  {
    destroy();

    size_x_ = size_x;
    size_y_ = size_y;

    voronoi_diagram_ = new VoronoiData*[size_x_];
    for (int i = 0; i < size_x_; i++)
      voronoi_diagram_[i] = new VoronoiData[size_y_];
  }

  bool has_voronoi_layer = false;
  // check if the costmap has a Voronoi layer
  for (std::vector<boost::shared_ptr<costmap_2d::Layer> >::const_iterator layer =
           costmap_ros_->getLayeredCostmap()->getPlugins()->begin();
       layer != costmap_ros_->getLayeredCostmap()->getPlugins()->end(); ++layer)
  {
    boost::shared_ptr<costmap_2d::VoronoiLayer> voronoi_layer =
        boost::dynamic_pointer_cast<costmap_2d::VoronoiLayer>(*layer);
    if (voronoi_layer)
    {
      has_voronoi_layer = true;
      boost::unique_lock<boost::mutex> lock(*(voronoi_layer->getMutex()));

      const DynamicVoronoi* voronoi = voronoi_layer->getVoronoi();
      for (int j = 0; j < size_y_; j++)
      {
        for (int i = 0; i < size_x_; i++)
        {
          voronoi_diagram_[i][j].dist = voronoi->getDistance(i, j) * resolution;  // important!
          voronoi_diagram_[i][j].isVoronoi = voronoi->isVoronoi(i, j);
        }
      }

      break;
    }
  }

  if (!has_voronoi_layer)
  {
    ROS_ERROR("Failed to get a Voronoi layer for Voronoi planner");
    return false;
  }

  int start_x = CONTXY2DISC(start.pose.position.x - origin_x, resolution);
  int start_y = CONTXY2DISC(start.pose.position.y - origin_y, resolution);
  int goal_x = CONTXY2DISC(goal.pose.position.x - origin_x, resolution);
  int goal_y = CONTXY2DISC(goal.pose.position.y - origin_y, resolution);

  std::vector<std::pair<int, int> > path;
  int pathCost;

  if (!voronoi_planner_->search(start_x, start_y, goal_x, goal_y, pathCost, &path, size_x_, size_y_, voronoi_diagram_,
                                circumscribed_radius))
  {
    ROS_WARN("Failed to find the shortest Voronoi path");
    return false;
  }

  plan.clear();
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = costmap_ros_->getGlobalFrameID();
  pose.header.stamp = ros::Time::now();
  for (int i = 0; i < (int)path.size(); i++)
  {
    pose.pose.position.x = DISCXY2CONT(path[i].first, resolution) + origin_x;
    pose.pose.position.y = DISCXY2CONT(path[i].second, resolution) + origin_y;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    plan.push_back(pose);
  }

  publishVoronoiPath(plan);

  return true;
}

void VoronoiPlannerROS::publishVoronoiPath(const std::vector<geometry_msgs::PoseStamped>& plan)
{
  nav_msgs::Path gui_path;
  gui_path.header.frame_id = costmap_ros_->getGlobalFrameID();
  gui_path.header.stamp = ros::Time::now();

  gui_path.poses = plan;
  path_pub_.publish(gui_path);
}

}  // namespace voronoi_planner
