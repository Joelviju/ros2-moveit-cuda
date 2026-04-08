cmake_minimum_required(VERSION 3.8)
project(my_robot_cpp_moveit)

add_compile_options(-Wall -Wextra -Wpedantic)

# ============================================================
# 📦 FIND PACKAGES
# ============================================================

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(CUDAToolkit REQUIRED)

# ============================================================
# 🚀 ENABLE CUDA
# ============================================================

enable_language(CUDA)
set(CMAKE_CUDA_ARCHITECTURES 75)

include_directories(
  include
  ${CUDAToolkit_INCLUDE_DIRS}
)

# ============================================================
# 🔥 GPU CORE (CUDA)
# ============================================================

add_library(gpu_collision_lib SHARED
  src/gpu_collision.cu
)

set_target_properties(gpu_collision_lib PROPERTIES
  CUDA_SEPARABLE_COMPILATION ON
  POSITION_INDEPENDENT_CODE ON
)

target_link_libraries(gpu_collision_lib
  CUDA::cudart
)

# ============================================================
# 🎯 MAIN EXECUTABLE
# ============================================================

add_executable(cpu_planning_benchmark_CUDA
  src/cpu_planning_benchmark_CUDA.cpp
  src/batch_manager.cpp
  src/gpu_collision_manager.cpp   # 🔥 ADD THIS LINE
)

ament_target_dependencies(cpu_planning_benchmark_CUDA
  rclcpp
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
  geometry_msgs
)

target_link_libraries(cpu_planning_benchmark_CUDA
  gpu_collision_lib
  CUDA::cudart
)

# ============================================================
# 📦 INSTALL
# ============================================================

install(TARGETS
  cpu_planning_benchmark_CUDA
  gpu_collision_lib
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()