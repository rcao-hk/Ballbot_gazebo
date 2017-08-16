cmake_minimum_required(VERSION 2.8.3)
project(ballbot_plugin)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  gazebo_ros
  sensor_msgs
)

# Depend on system install of Gazebo
find_package(cmake_modules REQUIRED)
find_package(gazebo REQUIRED)
find_package(Eigen3 REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

link_directories(/usr/local/include)
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS
    roscpp
    gazebo_ros
    sensor_msgs
  LIBRARIES ${PROJECT_NAME}
  DEPENDS
    EIGEN3
)

add_executable(imu_transformer src/imu_transformer.cpp)
target_link_libraries(imu_transformer ${catkin_LIBRARIES})

add_executable(velo_transformer src/velo_transformer.cpp)
target_link_libraries(velo_transformer ${catkin_LIBRARIES}${EIGEN_INCLUDE_DIRS})

add_library(BallbotPlugin src/simple_world_plugin.cpp)
target_link_libraries(BallbotPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(FrictionModifier src/friction_modifier.cpp)
target_link_libraries(FrictionModifier ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(apply_joint_effort_client src/apply_joint_effort_client.cpp)
target_link_libraries(apply_joint_effort_client ${catkin_LIBRARIES})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
