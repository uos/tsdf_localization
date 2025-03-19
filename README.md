# TSDF-based 3D Monte Carlo Localization ROS Package

This package consists of several nodes and tools to perform a 6D Monte Carlo Localization of robots equipped with a 3D LiDAR in 3D TSDF maps.
The sensor update is massively accelerated by a GPU-based implementation, but can also be executed on the CPU.

[![Demo GIF](doc/tsdf_loc_teaser.gif)](https://github.com/uos/tsdf_localization_demo.git)

## Prerequisites
* ROS Noetic (*ros-noetic-desktop-full*)
* ROS packages: See `package.xml`
* OpenMP (for CPU acceleration)
* CUDA (optional, recommended for acceleration)

## Installation

1. Clone this repository into your ROS workspace
```console
$ git clone --recursive https://github.com/uos/tsdf_localization.git
```
2. Make sure have also installed the required external packages or also cloned them into the local ROS workspace

3. Build the ROS workspace
```console
$ catkin build
```

## Demo

A quick startup including how to use tsdf_localization within your package is shown here: https://github.com/uos/tsdf_localization_demo.git

## Citation

Please reference the following papers when using `tsdf_localization` in your scientific work.
 
```bib
@inproceedings{eisoldt2023tsdfmcl,
  author={Eisoldt, Marc and Mock, Alexander and Porrmann, Mario and Wiemann, Thomas},
  booktitle={2023 Seventh IEEE International Conference on Robotic Computing (IRC)}, 
  title={{Towards 6D MCL for LiDARs in 3D TSDF Maps on Embedded Systems with GPUs}}, 
  year={2023},
  pages={158-165},
  doi={10.1109/IRC59093.2023.00035}
}
```

The paper is available on [IEEE Xplore](https://ieeexplore.ieee.org/document/10473560) and as preprint on [arXiv](https://arxiv.org/abs/2310.04172).


```bib
@article{eisoldt2025tsdfmcl,
  author={Eisoldt, Marc and Mock, Alexander and Wiemann, Thomas and Porrmann, Mario},
  title={Efficient Global 6D Localization in 3D TSDF Maps Using Point-wise and Scan-wise Reduction Methods on Embedded GPUs},
  journal={International Journal of Semantic Computing},
  doi={10.1142/S1793351X25410053}
}
```

The paper is available on [World Scientific](https://www.worldscientific.com/doi/abs/10.1142/S1793351X25410053).


## Nodes

### mcl_3d

Starts MCL in a given TSDF map.

#### Subscribed Topics:

`initialpose (geometry_msgs/PoseWithCovarianceStamped)`

Initial pose guess can be provided using RViz.

`/cloud (sensor_msgs/PointCloud2)` 

PointCloud topic for sensor update.

`/odom (nav_msgs/Odometry)`

Odometry message for motion update.

(optional) `/imu_data (sensor_msgs/Imu)`

#### Services

Start global localization:

`/global_localization`


## Contributions

We are happy about issues and pull requests or other feedback. Please let us know if something did not work out as expected.

