/******************************************************************************
 * Copyright (c) 2022, NKU Mobile & Flying Robotics Lab
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#include "voronoi_planner/voronoi_planner_ros.h"

#include <string>

#include "nav_msgs/Path.h"
#include "pluginlib/class_list_macros.hpp"
#include "tf/tf.h"

PLUGINLIB_EXPORT_CLASS(voronoi_planner::VoronoiPlannerROS,
                       nav_core::BaseGlobalPlanner)

namespace voronoi_planner {

VoronoiPlannerROS::VoronoiPlannerROS(std::string name,
                                     costmap_2d::Costmap2DROS* costmap_ros) {
  initialize(name, costmap_ros);
}

VoronoiPlannerROS::~VoronoiPlannerROS() {
  if (voronoi_planner_) {
    delete voronoi_planner_;
  }

  destroy();
}

void VoronoiPlannerROS::destroy() {
  if (voronoi_diagram_) {
    for (unsigned int i = 0; i < last_size_x_; i++) {
      delete[] voronoi_diagram_[i];
    }
    delete[] voronoi_diagram_;
  }
}

void VoronoiPlannerROS::initialize(std::string name,
                                   costmap_2d::Costmap2DROS* costmap_ros) {
  if (!initialized_) {
    ros::NodeHandle private_nh("~/" + name);

    ROS_DEBUG("Name is %s", name.c_str());

    name_ = name;
    costmap_ros_ = costmap_ros;

    voronoi_planner_ = new VoronoiPlanner();

    path_pub_ = private_nh.advertise<nav_msgs::Path>("voronoi_path", 1);

    initialized_ = true;
  }
}

bool VoronoiPlannerROS::makePlan(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    std::vector<geometry_msgs::PoseStamped>& plan) {
  if (!initialized_) {
    ROS_ERROR("voronoi_planner is not initialized");
    return false;
  }

  double circumscribed_radius =
      costmap_ros_->getLayeredCostmap()->getCircumscribedRadius();
  unsigned int size_x = costmap_ros_->getCostmap()->getSizeInCellsX();
  unsigned int size_y = costmap_ros_->getCostmap()->getSizeInCellsY();
  double resolution = costmap_ros_->getCostmap()->getResolution();
  double origin_x = costmap_ros_->getCostmap()->getOriginX();
  double origin_y = costmap_ros_->getCostmap()->getOriginY();

  if (size_x != last_size_x_ || size_y != last_size_y_) {
    destroy();

    voronoi_diagram_ = new VoronoiData*[size_x];
    for (unsigned int i = 0; i < size_x; i++) {
      voronoi_diagram_[i] = new VoronoiData[size_y];
    }

    last_size_x_ = size_x;
    last_size_y_ = size_y;
  }

  bool has_voronoi_layer = false;
  // check if the costmap has a Voronoi layer
  for (auto layer = costmap_ros_->getLayeredCostmap()->getPlugins()->begin();
       layer != costmap_ros_->getLayeredCostmap()->getPlugins()->end();
       ++layer) {
    boost::shared_ptr<costmap_2d::VoronoiLayer> voronoi_layer =
        boost::dynamic_pointer_cast<costmap_2d::VoronoiLayer>(*layer);
    if (voronoi_layer) {
      has_voronoi_layer = true;
      boost::unique_lock<boost::mutex> lock(voronoi_layer->getMutex());

      const DynamicVoronoi& voronoi = voronoi_layer->getVoronoi();
      for (unsigned int j = 0; j < size_y; j++) {
        for (unsigned int i = 0; i < size_x; i++) {
          voronoi_diagram_[i][j].dist =
              voronoi.getDistance(i, j) * resolution;  // important!
          voronoi_diagram_[i][j].is_voronoi = voronoi.isVoronoi(i, j);
        }
      }

      break;
    }
  }

  if (!has_voronoi_layer) {
    ROS_ERROR("Failed to get a Voronoi layer for Voronoi planner");
    return false;
  }

  int start_x = CONTXY2DISC(start.pose.position.x - origin_x, resolution);
  int start_y = CONTXY2DISC(start.pose.position.y - origin_y, resolution);
  int goal_x = CONTXY2DISC(goal.pose.position.x - origin_x, resolution);
  int goal_y = CONTXY2DISC(goal.pose.position.y - origin_y, resolution);

  std::vector<std::pair<int, int>> path;
  int path_cost;

  if (!voronoi_planner_->search(start_x, start_y, goal_x, goal_y, &path_cost,
                                &path, size_x, size_y, voronoi_diagram_,
                                circumscribed_radius)) {
    ROS_WARN("Failed to find the shortest Voronoi path");
    return false;
  }

  plan.clear();
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = costmap_ros_->getGlobalFrameID();
  pose.header.stamp = ros::Time::now();
  for (size_t i = 0; i < path.size(); i++) {
    pose.pose.position.x = DISCXY2CONT(path[i].first, resolution) + origin_x;
    pose.pose.position.y = DISCXY2CONT(path[i].second, resolution) + origin_y;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    plan.push_back(pose);
  }

  publishVoronoiPath(plan);

  return true;
}

void VoronoiPlannerROS::publishVoronoiPath(
    const std::vector<geometry_msgs::PoseStamped>& plan) {
  nav_msgs::Path gui_path;
  gui_path.header.frame_id = costmap_ros_->getGlobalFrameID();
  gui_path.header.stamp = ros::Time::now();

  gui_path.poses = plan;
  path_pub_.publish(gui_path);
}

}  // namespace voronoi_planner
