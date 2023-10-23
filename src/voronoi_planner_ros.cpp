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

#include "costmap_2d/inflation_layer.h"
#include "glog/logging.h"
#include "nav_msgs/Path.h"
#include "pluginlib/class_list_macros.hpp"
#include "tf/tf.h"
#include "voronoi_layer/voronoi_layer.h"

#include "voronoi_planner/common/util.h"

PLUGINLIB_EXPORT_CLASS(voronoi_planner::VoronoiPlannerROS,
                       nav_core::BaseGlobalPlanner)

namespace voronoi_planner {

VoronoiPlannerROS::VoronoiPlannerROS(std::string name,
                                     costmap_2d::Costmap2DROS* costmap_ros) {
  initialize(name, costmap_ros);
}

void VoronoiPlannerROS::initialize(std::string name,
                                   costmap_2d::Costmap2DROS* costmap_ros) {
  // Check if VoronoiPlannerROS has been initialized.
  if (initialized_) {
    LOG(INFO) << "VoronoiPlannerROS has been initialized.";
    return;
  }

  ros::NodeHandle private_nh("~/" + name);
  VLOG(4) << "Name is " << name;
  name_ = name;

  // Check and update costmap.
  if (!UpdateCostmap(costmap_ros)) {
    LOG(ERROR) << "Failed to update costmap.";
    return;
  }

  voronoi_planner_ = std::make_unique<VoronoiPlanner>(
      costmap_2d_->getSizeInCellsX(), costmap_2d_->getSizeInCellsY());

  path_pub_ = private_nh.advertise<nav_msgs::Path>("voronoi_path", 1);

  initialized_ = true;
  LOG(INFO) << "VoronoiPlannerROS is initialized successfully.";
}

bool VoronoiPlannerROS::UpdateCostmap(costmap_2d::Costmap2DROS* costmap_ros) {
  if (costmap_ros == nullptr) {
    LOG(ERROR) << "costmap_ros == nullptr";
    return false;
  }

  costmap_ros_ = costmap_ros;
  costmap_2d_ = costmap_ros->getCostmap();
  layered_costmap_ = costmap_ros->getLayeredCostmap();
  if (costmap_2d_ == nullptr || layered_costmap_ == nullptr) {
    LOG(ERROR) << "costmap_2d_ == nullptr || layered_costmap_ == nullptr";
    return false;
  }
  return true;
}

void VoronoiPlannerROS::GetStartAndEndConfigurations(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal, double resolution, double origin_x,
    double origin_y, int* start_x, int* start_y, int* end_x, int* end_y) {
  // Sanity checks.
  CHECK_NOTNULL(start_x);
  CHECK_NOTNULL(start_y);
  CHECK_NOTNULL(end_x);
  CHECK_NOTNULL(end_y);

  VLOG(4) << std::fixed << "start point: " << start.pose.position.x << ","
          << start.pose.position.y;
  VLOG(4) << std::fixed << "end point: " << goal.pose.position.x << ","
          << goal.pose.position.y;

  // Start configuration.
  *start_x = common::ContXY2Disc(start.pose.position.x - origin_x, resolution);
  *start_y = common::ContXY2Disc(start.pose.position.y - origin_y, resolution);

  // End configuration.
  *end_x = common::ContXY2Disc(goal.pose.position.x - origin_x, resolution);
  *end_y = common::ContXY2Disc(goal.pose.position.y - origin_y, resolution);
}

std::vector<std::vector<VoronoiData>> VoronoiPlannerROS::GetVoronoiDiagram(
    unsigned int size_x, unsigned int size_y, double resolution) {
  // Sanity checks.
  CHECK_NOTNULL(layered_costmap_);
  CHECK_NOTNULL(costmap_2d_);

  std::vector<boost::shared_ptr<costmap_2d::Layer>>* plugins =
      layered_costmap_->getPlugins();
  if (plugins == nullptr) {
    LOG(ERROR) << "plugins == nullptr";
    return std::vector<std::vector<VoronoiData>>();
  }

  // Check if costmap has a Voronoi layer.
  for (auto layer = plugins->begin(); layer != plugins->end(); ++layer) {
    auto voronoi_layer =
        boost::dynamic_pointer_cast<costmap_2d::VoronoiLayer>(*layer);
    if (voronoi_layer == nullptr) {
      continue;
    }

    std::lock_guard<std::mutex> lock(voronoi_layer->mutex());

    const DynamicVoronoi& voronoi = voronoi_layer->voronoi();
    std::vector<std::vector<VoronoiData>> gvd_map;
    gvd_map.resize(size_x);
    for (unsigned int i = 0U; i < size_x; ++i) {
      gvd_map[i].resize(size_y);
      for (unsigned int j = 0U; j < size_y; ++j) {
        gvd_map[i][j].dist = voronoi.getDistance(i, j) * resolution;
        gvd_map[i][j].is_voronoi = voronoi.isVoronoi(i, j);
      }
    }
    return gvd_map;
  }

  LOG(ERROR) << "Failed to get a Voronoi layer for Voronoi planner";
  return std::vector<std::vector<VoronoiData>>();
}

bool VoronoiPlannerROS::makePlan(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    std::vector<geometry_msgs::PoseStamped>& plan) {
  // Check if VoronoiPlannerROS has been initialized.
  if (!initialized_) {
    ROS_ERROR("voronoi_planner is not initialized");
    return false;
  }

  // Get start and end configurations.
  int start_x = 0;
  int start_y = 0;
  int end_x = 0;
  int end_y = 0;
  GetStartAndEndConfigurations(
      start, goal, costmap_2d_->getResolution(), costmap_2d_->getOriginX(),
      costmap_2d_->getOriginY(), &start_x, &start_y, &end_x, &end_y);

  // Get Voronoi diagram.
  std::vector<std::vector<VoronoiData>> gvd_map = GetVoronoiDiagram(
      costmap_2d_->getSizeInCellsX(), costmap_2d_->getSizeInCellsY(),
      costmap_2d_->getResolution());
  if (gvd_map.empty()) {
    LOG(ERROR) << "Voronoi layer is not available.";
    return false;
  }

  // Search path via Voronoi planner.
  std::vector<std::pair<int, int>> path;
  if (!voronoi_planner_->Search(
          start_x, start_y, end_x, end_y, std::move(gvd_map),
          layered_costmap_->getCircumscribedRadius(), &path)) {
    LOG(ERROR) << "Failed to find the shortest Voronoi path";
    return false;
  }

  // Populate global path.
  PopulateVoronoiPath(path, start.header, costmap_2d_->getResolution(),
                      costmap_2d_->getOriginX(), costmap_2d_->getOriginY(),
                      &plan);

  // Publish Voronoi path.
  PublishVoronoiPath(plan, path_pub_);

  return true;
}

void VoronoiPlannerROS::PopulateVoronoiPath(
    const std::vector<std::pair<int, int>>& searched_result,
    const std_msgs::Header& header, double resolution, double origin_x,
    double origin_y, std::vector<geometry_msgs::PoseStamped>* plan) {
  // Sanity checks.
  CHECK_NOTNULL(plan);

  plan->clear();
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header = header;
  for (const auto& pose : searched_result) {
    pose_stamped.pose.position.x =
        common::DiscXY2Cont(pose.first, resolution) + origin_x;
    pose_stamped.pose.position.y =
        common::DiscXY2Cont(pose.second, resolution) + origin_y;
    pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    plan->push_back(pose_stamped);
  }
}

void VoronoiPlannerROS::PublishVoronoiPath(
    const std::vector<geometry_msgs::PoseStamped>& plan,
    const ros::Publisher& pub) {
  if (plan.empty()) {
    return;
  }

  nav_msgs::Path gui_path;
  gui_path.header = plan.front().header;
  gui_path.poses = plan;
  pub.publish(gui_path);
}

}  // namespace voronoi_planner
