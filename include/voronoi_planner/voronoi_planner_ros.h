#ifndef _VORONOI_PLANNER_ROS_H_
#define _VORONOI_PLANNER_ROS_H_

// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

// global representation
#include <nav_core/base_global_planner.h>

// Costmap used for the map representation
#include <costmap_2d/costmap_2d_ros.h>

#include "voronoi_layer/voronoi_layer.h"
#include "voronoi_planner/voronoi_planner.h"

#define CONTXY2DISC(X, CELLSIZE) (((X) >= 0) ? ((int)((X) / (CELLSIZE))) : ((int)((X) / (CELLSIZE)) - 1))
#define DISCXY2CONT(X, CELLSIZE) ((X) * (CELLSIZE) + (CELLSIZE) / 2.0)

namespace voronoi_planner
{
class VoronoiPlannerROS : public nav_core::BaseGlobalPlanner
{
public:
  /**
   * @brief  Default constructor for the VoronoiPlannerROS object
   */
  VoronoiPlannerROS();

  /**
   * @brief  Constructor for the VoronoiPlannerROS object
   * @param  name The name of this planner
   * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use
   */
  VoronoiPlannerROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief  Initialization function for the VoronoiPlannerROS object
   * @param  name The name of this planner
   * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use
   */
  virtual void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief Given a goal pose in the world, compute a plan
   * @param start The start pose
   * @param goal The goal pose
   * @param plan The plan... filled by the planner
   * @return True if a valid plan was found, false otherwise
   */
  virtual bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                        std::vector<geometry_msgs::PoseStamped>& plan);

  virtual ~VoronoiPlannerROS();

private:
  void publishVoronoiPath(const std::vector<geometry_msgs::PoseStamped>& plan);
  void destroy();

private:
  VoronoiPlanner* voronoi_planner_;
  costmap_2d::Costmap2DROS* costmap_ros_;  //!< manages the cost map for us
  std::string name_;
  int size_x_;
  int size_y_;
  VoronoiData** voronoi_diagram_;

  ros::Publisher path_pub_;

  bool initialized_;
};

}  // namespace voronoi_planner

#endif