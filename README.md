# TSDF-based 3D Monte Carlo Localization ROS Package

This package consists of several nodes and tools to perform a 6D Monte Carlo Localization of robots equipped with a 3D LiDAR in 3D TSDF maps.
The sensor update is massively accelerated by a GPU-based implementation, but can also be executed on the CPU.

## Prerequisites
* ROS Noetic (*ros-noetic-desktop-full*)
* ROS packages
  * TF2 Sensor Messages (*ros-noetic-sensor-msgs*)
* OpenMP (for CPU acceleration)
* CUDA (optional, recommended for acceleration)

## Installation

1. Clone the respository into your ROS workspace
```console
$ git clone --recursive https://github.com/uos/tsdf_localization.git
```
2. Make sure have also installed the required external packages or also cloned them into the local ROS workspace

3. Build the ROS workspace
```console
$ catkin build
```

## Demo

A quick startup and an example usage of tsdf_localization within your package is shown here: https://github.com/uos/tsdf_localization_demo.git

## Citation

Please reference the following papers when using `tsdf_localization` in your scientific work.

- Title: "Towards 6D MCL for LiDARs in 3D TSDF Maps on Embedded Systems with GPUs"
- Preprint: https://arxiv.org/abs/2310.04172

The paper is accepted to 2023 IEEE International Conference on Robotic Computing (IRC). The citation will be updated soon.

## Contributions

We are happy about issues and pull requests or other feedback. Please let us know if something did not work out as expected.
