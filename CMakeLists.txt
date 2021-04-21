cmake_minimum_required(VERSION 3.5)
project(inertial_sense)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror=return-type)
endif()

find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
# find_package(rosidl_default_generators REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(Threads)

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11 -fms-extensions -Wl,--no-as-needed -DPLATFORM_IS_LINUX" )
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -fms-extensions -Wl,--no-as-needed -DPLATFORM_IS_LINUX")

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GTime.msg"
  "msg/SatInfo.msg"
  "msg/GPS.msg"
  "msg/GPSInfo.msg"
  "msg/PreIntIMU.msg"
  "msg/RTKInfo.msg"
  "msg/RTKRel.msg"
  "msg/GlonassEphemeris.msg"
  "msg/GNSSEphemeris.msg"
  "msg/GNSSObservation.msg"
  "msg/GNSSObsVec.msg"
  "srv/FirmwareUpdate.srv"
  "srv/RefLLAUpdate.srv"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)


include_directories(include
  lib/InertialSenseSDK/src
)

add_subdirectory(lib/InertialSenseSDK)

ament_export_include_directories(include)

add_library(inertial_sense_ros
        src/inertial_sense.cpp
)
ament_target_dependencies(inertial_sense_ros
  "rclcpp"
  "geometry_msgs"
  "sensor_msgs"
  "nav_msgs"
  "std_msgs"
  "std_srvs"
  "tf2_ros"
  "diagnostic_msgs")
target_link_libraries(inertial_sense_ros InertialSense pthread)
target_include_directories(inertial_sense_ros PUBLIC include lib/InertialSenseSDK/src)
rosidl_target_interfaces(inertial_sense_ros ${PROJECT_NAME} "rosidl_typesupport_cpp")




add_executable(inertial_sense_node src/inertial_sense_node.cpp)
target_link_libraries(inertial_sense_node inertial_sense_ros)
ament_target_dependencies(inertial_sense_node
  "rclcpp"
  "geometry_msgs"
  "sensor_msgs"
  "nav_msgs"
  "std_msgs"
  "std_srvs"
  "tf2_ros"
  "diagnostic_msgs")
rosidl_target_interfaces(inertial_sense_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
install(TARGETS ${PROJECT_NAME}_node
   DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()