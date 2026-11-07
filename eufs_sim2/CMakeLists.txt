cmake_minimum_required(VERSION 3.8)
project(eufs_sim2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Use C99 and C++20
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(state_lib REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(eufs_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(vehicle_models REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geodesy REQUIRED)
find_package(spdlog REQUIRED)
find_package(std_srvs REQUIRED)
find_package(map_lib REQUIRED)

# Create dependencies variable
set(dependencies
        rclcpp
        state_lib
        ackermann_msgs
        eufs_msgs
        std_srvs
        geometry_msgs
        sensor_msgs
        nav_msgs
        vehicle_models
        tf2_geometry_msgs
        geodesy
        map_lib
        spdlog)

# Create the sim library
add_library(${PROJECT_NAME}
        src/time.cpp
        src/simulation.cpp
        src/plugin/control_input.cpp
        src/control.cpp
        src/state_machine.cpp
        src/plugin/wheel_speed.cpp
        src/plugin/gnss.cpp
        src/plugin/imu_plugin.cpp
        src/plugin/vehicle_state.cpp
        src/plugin/oss_plugin.cpp
        src/eufs_core.cpp
        src/plugin/track_changer.cpp
        src/plugin/state_machine_plugin.cpp
        src/plugin/state_publisher.cpp
        src/plugin/gt_transform.cpp
        src/plugin/base.cpp
        src/plugin/force_publisher.cpp
        src/plugin/twist_publisher.cpp
        src/plugin/cone_fusion.cpp
        src/independent_gaussian.cpp
        src/plugin/cone_collision_tracker.cpp)

ament_target_dependencies(${PROJECT_NAME} PUBLIC ${dependencies})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Create the executable
add_executable(${PROJECT_NAME}_node
        src/sim_node.cpp)
target_include_directories(${PROJECT_NAME}_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_node
        ${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}_node
        ${dependencies})

# Export
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

# Install library
install(
        DIRECTORY include/
        DESTINATION include
)
install(
        TARGETS ${PROJECT_NAME}
        EXPORT  ${PROJECT_NAME}Targets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

# Install executable
install(TARGETS ${PROJECT_NAME}_node
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
        eufs_racecar
        launch
        config
        DESTINATION share/${PROJECT_NAME})

# Create tests
# if(BUILD_TESTING)
#  find_package(ament_cmake_gtest REQUIRED)
#  ament_add_gtest(${PROJECT_NAME}_test test/state_machine_test.cpp)
#  ament_target_dependencies(${PROJECT_NAME}_test
#    ament_cmake_gtest
#  )
#  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
# endif()

ament_package()
