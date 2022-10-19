# voronoi_planner
This repo contains a global planner plugin for ROS navigation stack, in which A* search on a discrete gneralized Voronoi diagram (GVD) is implemented.

## How to use?

### 1. Clone and build voronoi_layer and voronoi_planner
```
$ cd ~/catkin_ws/src/
$ git clone https://github.com/nkuwenjian/voronoi_layer.git
$ git clone https://github.com/nkuwenjian/voronoi_planner.git
$ cd ../
$ catkin_make -DCMAKE_BUILD_TYPE=Release
```

### 2. Setup the configurations for ROS navigation stack

In the move_base launch file, use voronoi_planner to override the default global planner, which may like this:
```
<node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen" >
  <rosparam file="$(find ${YOUR_PROJECT_NAME})/params/costmap_common_params.yaml" command="load" ns="global_costmap" />
  <rosparam file="$(find ${YOUR_PROJECT_NAME})/params/costmap_common_params.yaml" command="load" ns="local_costmap" />
  <rosparam file="$(find ${YOUR_PROJECT_NAME})/params/global_costmap_params.yaml" command="load" />
  <rosparam file="$(find ${YOUR_PROJECT_NAME})/params/local_costmap_params.yaml" command="load" />

  <param name="base_global_planner" value="voronoi_planner/VoronoiPlannerROS" />
  <param name="planner_frequency" value="1.0" />
  <param name="planner_patience" value="5.0" />

  ...
</node>
```

In the global_costmap_params.yaml file, add voronoi_layer plugin, which may like this:
```
global_costmap:
  global_frame: map
  robot_base_frame: base_link
  update_frequency: 1.0
  publish_frequency: 0.5

  transform_tolerance: 0.5
  plugins:
    - { name: static_layer, type: "costmap_2d::StaticLayer" }
    - { name: obstacle_layer, type: "costmap_2d::ObstacleLayer" }
    - { name: inflation_layer, type: "costmap_2d::InflationLayer" }
    - { name: voronoi_layer, type: "costmap_2d::VoronoiLayer" }
```
