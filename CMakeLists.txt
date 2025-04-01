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
find_package(tf2_ros REQUIRED)
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
    tf2_ros 
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


if(CUDA_FOUND)

    # copied from rmagine:
    # I am aware find_package(CUDA) is deprecated.
    # - TODO: support find_package(CUDAToolkit) as well
    # -- Example: https://github.com/ceres-solver/ceres-solver/blob/master/CMakeLists.txt
    # - TODO: this becomes quite messy. make a seperate file from this
    # Or wait until all targeted platforms use `find_package(CUDAToolkit)`, then remove everything

    # default flags are not set when including cuda
    if (NOT EXISTS ${CMAKE_BINARY_DIR}/CMakeCache.txt)
        if(NOT CUDA_NVCC_FLAGS_DEBUG)
            set(CUDA_NVCC_FLAGS_DEBUG "-g" CACHE STRING "" FORCE)
        endif()
        if(NOT CUDA_NVCC_FLAGS_MINSIZEREL)
            set(CUDA_NVCC_FLAGS_MINSIZEREL "-Os -DNDEBUG" CACHE STRING "" FORCE)
        endif()
        if(NOT CUDA_NVCC_FLAGS_RELEASE)
            set(CUDA_NVCC_FLAGS_RELEASE "-O3 -DNDEBUG" CACHE STRING "" FORCE)
        endif()
        if(NOT CUDA_NVCC_FLAGS_RELWITHDEBINFO)
            set(CUDA_NVCC_FLAGS_RELWITHDEBINFO "-O2 -g -DNDEBUG" CACHE STRING "" FORCE)
        endif()
    endif()

    include(CheckLanguage)
    check_language(CUDA)
    if(CMAKE_CUDA_COMPILER)
        message(STATUS "Cuda language available!")

        find_package(CUDAToolkit QUIET)
        if(CUDAToolkit_FOUND)
            message(STATUS "Found Cuda Toolkit!")
            enable_language(CUDA)
            set(CUDA_FOUND True)
            set(CUDA_LIBRARIES CUDA::cudart)
            set(CUDA_cusolver_LIBRARY CUDA::cusolver)
            set(CUDA_cublas_LIBRARY CUDA::cublas)
            set(CUDA_DRIVER_LIBRARY CUDA::cuda_driver)
            set(CUDA_INCLUDE_DIRS "") # is in target instead
        else()
            find_package(CUDA)
            if(CUDA_FOUND)
                message(STATUS "Found Cuda!")
                enable_language(CUDA)
                set(CUDA_DRIVER_LIBRARY cuda)
            else()
                message(STATUS "Neither CudaToolkit nor CUDA found!")
            endif(CUDA_FOUND)
        endif(CUDAToolkit_FOUND)
    endif(CMAKE_CUDA_COMPILER)
endif()


if (CUDA_FOUND)

  # set(CUDA_NVCC_FLAGS "--std=c++17" CACHE STRING "nvcc flags" FORCE)
  # set(LIB_TYPE STATIC)

  add_library(tsdf_localization_cuda SHARED
    src/cuda/cuda_evaluator.cu 
    src/cuda/cuda_sum.cu 
    src/cuda/cuda_util.cu 
    src/util/runtime_evaluator.cpp
  )

  target_link_libraries(tsdf_localization_cuda
    ${CUDA_LIBRARIES}
    ${HDF5_LIBRARIES}
    ${HDF5_HL_LIBRARIES}
    ${Boost_LIBRARIES}
  )

  ament_target_dependencies(tsdf_localization_cuda
    sensor_msgs
    geometry_msgs
    tf2_sensor_msgs 
    tf2_ros
    tf2_geometry_msgs
  )

  set_target_properties(tsdf_localization_cuda
    PROPERTIES
      SOVERSION ${rmcl_VERSION_MAJOR}
      VERSION ${rmcl_VERSION}
      # CXX_STANDARD 17
  )

  install(TARGETS tsdf_localization_cuda
    EXPORT tsdf-localization-targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  )

  add_definitions(-DTSDF_LOCALIZATION_CUDA)

  # cuda_add_library(CUDAEvaluator ${LIB_TYPE} src/cuda/cuda_evaluator.cu src/cuda/cuda_sum.cu src/cuda/cuda_util.cu src/util/runtime_evaluator.cpp)
  # target_link_libraries(CUDAEvaluator ${HDF5_LIBRARIES} ${HDF5_HL_LIBRARIES} ${Boost_LIBRARIES})
  # ament_target_dependencies(CUDAEvaluator
  #   sensor_msgs
  #   geometry_msgs
  #   tf2_sensor_msgs 
  #   tf2_ros
  #   tf2_geometry_msgs
  # )

  # cuda_add_executable(cuda_test_eval src/cuda/cuda_test_eval.cu src/cuda/cuda_evaluator.cu src/cuda/cuda_sum.cu src/cuda/cuda_util.cu src/util/mcl_file.cpp src/util/runtime_evaluator.cpp)
  # target_link_libraries(cuda_test_eval ${HDF5_LIBRARIES} ${HDF5_HL_LIBRARIES} ${Boost_LIBRARIES})
  # ament_target_dependencies(cuda_test_eval
  #   sensor_msgs
  #   geometry_msgs
  #   tf2_sensor_msgs 
  #   tf2_ros
  #   tf2_geometry_msgs
  # )

endif(CUDA_FOUND)

### ROS nodes

####### PCL2 to SCAN CONVERTER
add_library(tsdf_mcl_component SHARED 
  src/tsdf_mcl.cpp
  ${TSDF_LOC_SRC})

## Specify libraries to link a library or executable target against
target_link_libraries(tsdf_mcl_component
  ${Boost_LIBRARIES}
  ${OpenMP_LIBS}
  ${HDF5_LIBRARIES}
  ${HDF5_HL_LIBRARIES}
  Eigen3::Eigen
)

target_compile_options(tsdf_mcl_component PRIVATE ${OpenMP_FLAGS})

ament_target_dependencies(tsdf_mcl_component
    rclcpp
    rclcpp_components
    sensor_msgs
    geometry_msgs
    visualization_msgs
    nav_msgs
    tf2_sensor_msgs 
    tf2_ros
    tf2_geometry_msgs
    std_srvs
)

# Enable cuda by linking
if (CUDA_FOUND)
  add_dependencies(tsdf_mcl_component 
    tsdf_localization_cuda
  )
  target_link_libraries(tsdf_mcl_component 
    tsdf_localization_cuda
  )
endif (CUDA_FOUND)

rclcpp_components_register_node(tsdf_mcl_component 
    PLUGIN "tsdf_localization::TSDFMCLNode" 
    EXECUTABLE tsdf_mcl)

install(TARGETS 
    tsdf_mcl_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


# TSDF Viz Node
add_library(tsdf_viz_component SHARED 
  src/tsdf_viz.cpp)

target_compile_options(tsdf_viz_component PRIVATE ${OpenMP_FLAGS})

ament_target_dependencies(tsdf_viz_component 
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  visualization_msgs
  nav_msgs
  tf2_sensor_msgs
  tf2_ros 
  tf2_geometry_msgs 
  std_srvs
)

target_link_libraries(tsdf_viz_component
  ${OpenMP_LIBS}
  ${HDF5_LIBRARIES}
  ${HDF5_HL_LIBRARIES}
  Eigen3::Eigen
)

rclcpp_components_register_node(tsdf_viz_component 
    PLUGIN "tsdf_localization::TSDFVizNode" 
    EXECUTABLE tsdf_viz)

install(TARGETS 
    tsdf_viz_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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