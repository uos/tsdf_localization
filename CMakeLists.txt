cmake_minimum_required(VERSION 3.18)
project(tsdf_localization 
        VERSION 1.0.0)

# RELEASE PER DEFAULT
if (NOT EXISTS ${CMAKE_BINARY_DIR}/CMakeCache.txt)
  if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
  endif()
endif()

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(ament_cmake REQUIRED 
  COMPONENTS
    #diagnostic_updater
    #dynamic_reconfigure
    #geometry_msgs
    #rclcpp
    #sensor_msgs
    #visualization_msgs
    #nav_msgs
    #tf2
    #tf2_sensor_msgs
    #message_runtime
    #message_generation
)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(CUDA)

# let cmake find OpenMP and set some variables
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})  # or if you use C: ${OpenMP_C_FLAGS}
  set(OpenMP_LIBS gomp)
endif()

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

find_package(Boost REQUIRED COMPONENTS filesystem)

# dynamic reconfigure
# generate_dynamic_reconfigure_options(
#     cfg/MCL.cfg
#     cfg/RING.cfg
#)

# catkin_package(
#   CATKIN_DEPENDS
#     diagnostic_updater
#     dynamic_reconfigure
#     geometry_msgs
#     roscpp
#     sensor_msgs
#     visualization_msgs
#     nav_msgs
#     tf2
#     tf2_sensor_msgs
#     message_runtime
#     message_generation
#    INCLUDE_DIRS include
#    DEPENDS OpenMP
# )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  #${catkin_INCLUDE_DIRS}
)

set(TSDF_LOC_SRC 
  src/particle_cloud.cpp 
  src/util/util.cpp 
  src/evaluation/model/naiv_evaluation.cpp 
  src/evaluation/model/likelihood_evaluation.cpp 
  src/evaluation/model/omp_likelihood_evaluation.cpp 
  src/resampling/wheel_resampler.cpp 
  src/evaluation/tsdf_evaluator.cpp 
  src/util/runtime_evaluator.cpp 
  src/util/imu_accumulator.cpp
)

# External interface for HDF5 (libhdf5) HighFive (https://github.com/BlueBrain/HighFive).
set(HIGHFIVE_EXAMPLES FALSE CACHE INTERNAL "")
set(HIGHFIVE_UNIT_TESTS FALSE CACHE INTERNAL "")
add_subdirectory(ext/HighFive)
include_directories("ext/HighFive/include")
find_package(HDF5 REQUIRED COMPONENTS CXX C HL)
include_directories(${HDF5_INCLUDE_DIRS})

function(MAKE_NODE NAME CPP)
  add_executable(${NAME} ${CPP} src/util/runtime_evaluator.cpp)

  target_compile_options(${NAME} PRIVATE ${OpenMP_FLAGS})

  ament_target_dependencies(${NAME} 
    rclcpp
    sensor_msgs
    geometry_msgs
    visualization_msgs
    nav_msgs
    tf2_sensor_msgs 
    tf2_geometry_msgs 
    std_srvs
  )

  target_link_libraries(${NAME}
    ${OpenMP_LIBS}
    ${HDF5_LIBRARIES}
    ${HDF5_HL_LIBRARIES}
    Eigen3::Eigen
  )
endfunction()

### CUDA libraries

if (CUDA_FOUND)

set(CUDA_NVCC_FLAGS "--std=c++14" CACHE STRING "nvcc flags" FORCE)
set(LIB_TYPE STATIC)
cuda_add_library(CUDAEvaluator ${LIB_TYPE} src/cuda/cuda_evaluator.cu src/cuda/cuda_sum.cu src/cuda/cuda_util.cu src/util/runtime_evaluator.cpp)
target_link_libraries(CUDAEvaluator ${HDF5_LIBRARIES} ${HDF5_HL_LIBRARIES} ${Boost_LIBRARIES})

cuda_add_executable(cuda_test_eval src/cuda/cuda_test_eval.cu src/cuda/cuda_evaluator.cu src/cuda/cuda_sum.cu src/cuda/cuda_util.cu src/util/mcl_file.cpp src/util/runtime_evaluator.cpp)
target_link_libraries(cuda_test_eval ${HDF5_LIBRARIES} ${HDF5_HL_LIBRARIES} ${Boost_LIBRARIES})

endif(CUDA_FOUND)

### ROS nodes

add_executable(mcl_3d src/mcl_3d.cpp ${TSDF_LOC_SRC})

ament_target_dependencies(mcl_3d
    rclcpp
    sensor_msgs
    geometry_msgs
    visualization_msgs
    nav_msgs
    tf2_sensor_msgs 
    tf2_geometry_msgs
    std_srvs
)


if (CUDA_FOUND)

add_dependencies(mcl_3d CUDAEvaluator)
target_link_libraries(mcl_3d ${Boost_LIBRARIES} CUDAEvaluator)

else (CUDA_FOUND)

target_link_libraries(mcl_3d ${Boost_LIBRARIES})

endif (CUDA_FOUND)

target_compile_options(mcl_3d PRIVATE ${OpenMP_FLAGS})

target_link_libraries(mcl_3d 
    ${OpenMP_LIBS}
    ${HDF5_LIBRARIES}
    ${HDF5_HL_LIBRARIES}
    Eigen3::Eigen
)

# add_executable(num_particles_eval src/num_particles_eval.cpp src/util/mcl_file.cpp ${TSDF_LOC_SRC})

# if (CUDA_FOUND)

# add_dependencies(num_particles_eval CUDAEvaluator)
# target_link_libraries(num_particles_eval ${Boost_LIBRARIES} CUDAEvaluator)

# else (CUDA_FOUND)

# target_link_libraries(num_particles_eval ${Boost_LIBRARIES})

# endif (CUDA_FOUND)

# target_compile_options(num_particles_eval PRIVATE ${OpenMP_FLAGS})

# target_link_libraries(num_particles_eval
#     ${OpenMP_LIBS}
#     ${HDF5_LIBRARIES}
#     ${HDF5_HL_LIBRARIES}
#     Eigen3::Eigen
# )

# if (CUDA_FOUND)


# find_package(PkgConfig REQUIRED)
# pkg_check_modules(LZ4 REQUIRED liblz4)

# endif (CUDA_FOUND)

# MAKE_NODE(tsdf_vis src/tsdf_vis.cpp)

# if (CUDA_FOUND)

# add_executable(snap_shot_node src/snap_shot_node.cpp src/util/mcl_file.cpp ${TSDF_LOC_SRC})
# add_dependencies(snap_shot_node CUDAEvaluator)
# target_link_libraries(snap_shot_node ${Boost_LIBRARIES} CUDAEvaluator)

# target_compile_options(snap_shot_node PRIVATE ${OpenMP_FLAGS})

# target_link_libraries(snap_shot_node
#     ${OpenMP_LIBS}
#     ${HDF5_LIBRARIES}
#     ${HDF5_HL_LIBRARIES}
#     Eigen3::Eigen
# )

# add_executable(snap_vis_node src/snap_vis_node.cpp src/util/mcl_file.cpp ${TSDF_LOC_SRC})
# add_dependencies(snap_vis_node CUDAEvaluator)
# target_link_libraries(snap_vis_node ${Boost_LIBRARIES} CUDAEvaluator)

# target_compile_options(snap_vis_node PRIVATE ${OpenMP_FLAGS})

# target_link_libraries(snap_vis_node
#     ${OpenMP_LIBS}
#     ${HDF5_LIBRARIES}
#     ${HDF5_HL_LIBRARIES}
#     Eigen3::Eigen
# )

# endif (CUDA_FOUND)

ament_package()