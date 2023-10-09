# TSDF-based 3D Monte Carlo Localization ROS Package

This package consist of different nodes and tools to perform a 3D Monte Carlo Localization in TSDF maps based a 3D laser scanners.
The sensor update is accelerated by GPU-based implementation, but can also be executed on the CPU.
The repository also provides a Simulation environment based Gazebo, where the implementation can be tested in.
As an alternative, a launch file to setup the node for the HILTI SLAM Challenge 2021 is also included. 

## Prerequisites
* ROS Noetic (*ros-noetic-desktop-full*)
* ROS packages
  * Ceres Robot (https://gitlab.informatik.uni-osnabrueck.de/robotik_ss21_packages/ceres_robot.git)
  * UOS Tools (https://gitlab.informatik.uni-osnabrueck.de/robotik_ss21_packages/uos_tools.git)
  * Velodyne Simulator (https://bitbucket.org/DataspeedInc/velodyne_simulator.git)
  * Sick Tim (*ros-noetic-sick-tim*)
  * Robot Pose EKF (*ros-noetic-robot-pose-ekf*)
  * TF2 Sensor Messages (*ros-noetic-sensor-msgs*)
* CUDA (recommended for acceleration)
* OpenMP
* TSDF maps of the environments
* (optional) Datasets of he HILTI SLAM Challenge 2021 (https://hilti-challenge.com/dataset-2021.html) 

## Installation

1. Clone the respository into your ROS workspace
```console
$ git clone --recursive https://github.com/uos/tsdf_localization.git
```
2. Make shure have also installed the required external packages or also cloned them into the local ROS workspace

3. Build the ROS workspace
```console
$ catkin build
```

## Quick startup with default simulation and acceleration on the GPU (tmux recommended)

To start the ROS node of the Monte Carlo localization with the simulation, a few terminals are needed to launch different required nodes including the simulation and visualization.

**Important: You need a generated TSDF map of the default AVZ simulation environment**

1. Start your roscore
```console
$ roscore
```

2. Start the simulation
```console
$ roslaunch mcl ceres_velodyne.launch
```

3. Start the visualization
```console
$ rviz
```

4. Start the map visualization
```console
$ rosrun tsdf_map <path-to-your-tsdf-map>
```

5. Set the correct map path in the launch file of the MCL node (*mcl/launch/mcl_3d_sim.launch*)

6. Launch the MCL node
  ```console
  $ roslaunch mcl mcl_3d_sim.launch
  ```
7. Start dynmic parametrization tool 
```console
$ rosrun rqt_reconfigure rqt_reconfigure
```

8. Choose the TSDF map, the particle cloud and the estimated pose to be visualized in RViz

9. Drive with the robot
```console
$ roslaunch uos_diffdrive_teleop key.launch
```

## Quick startup with HILTI SLAM Chalange 2021 and acceleration on the GPU (tmux recommended)

To start the ROS node of the Monte Carlo localization with the simulation, a few terminals are needed to launch different required nodes including the bag file of the HILTI dataset and visualization.

**Important: You need a generated TSDF map of the environment of the used HILTI dataset**

1. Start your roscore
```console
$ roscore
```

2. Start the visualization
```console
$ rviz
```

3. Start the map visualization
```console
$ rosrun tsdf_map <path-to-your-tsdf-map>
```

4. Set the correct map path in the launch file of the MCL node (*mcl/launch/mcl_3d_hilti.launch*)

5. Launch the MCL node
  ```console
  $ roslaunch mcl mcl_3d_hilti.launch
  ```
6. Start dynmic parametrization tool 
```console
$ rosrun rqt_reconfigure rqt_reconfigure
```

7. Choose the TSDF map, the particle cloud and the estimated pose to be visualized in RViz

8. Start the bag file
```console
$ rosbag play <path-zo-hilti-bag-file>
```

You can chance different parameters of the Monte Carlo Localization in the rqt window and restart the localization setting a new initial pose in RViz. If the global localization flag is set in the MCL launch file, the initial pose is only used as a trigger to distribute the particles uniformly in the environment of the robot.

## Citation

Please reference the following papers when using `tsdf_localization` in your scientific work.

- Title: "Towards 6D MCL for LiDARs in 3D TSDF Maps on Embedded Systems with GPUs"
- Preprint: https://arxiv.org/abs/2310.04172