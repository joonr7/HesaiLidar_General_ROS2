cmake_minimum_required(VERSION 3.5)
project(hesai_lidar)

# Compile as C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost REQUIRED)

set(ament_dependencies
  rclcpp
  std_msgs
  sensor_msgs
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR NAMES yaml_cpp.h PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY NAMES YAML_CPP PATHS ${YAML_CPP_LIBRARIES})

### custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
"msg/PandarPacket.msg"
"msg/PandarScan.msg"
DEPENDENCIES builtin_interfaces std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

find_package(PCL REQUIRED)

###PandarGeneralRaw
add_library(PandarGeneral
    src/HesaiLidar_General_SDK/src/PandarGeneralRaw/src/input.cc
    src/HesaiLidar_General_SDK/src/PandarGeneralRaw/src/pandarGeneral_internal.cc
    src/HesaiLidar_General_SDK/src/PandarGeneralRaw/src/pcap_reader.cpp
    src/HesaiLidar_General_SDK/src/PandarGeneralRaw/src/pandarGeneral.cc
)
target_include_directories(PandarGeneral PRIVATE
    src/HesaiLidar_General_SDK/src/PandarGeneralRaw/include
    src/HesaiLidar_General_SDK/src/PandarGeneralRaw/
    ${Boost_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    # ${catkin_INCLUDE_DIRS}
)
target_link_libraries(PandarGeneral
    ${Boost_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    pcap
)
ament_target_dependencies(PandarGeneral)
rosidl_target_interfaces(PandarGeneral ${PROJECT_NAME} "rosidl_typesupport_cpp")

###PandarGeneralSDK
add_library(PandarGeneralSDK SHARED
    src/HesaiLidar_General_SDK/src/pandarGeneral_sdk.cc
    src/HesaiLidar_General_SDK/src/tcp_command_client.c
    src/HesaiLidar_General_SDK/src/util.c
)
target_include_directories(PandarGeneralSDK PRIVATE
    src/HesaiLidar_General_SDK/
    src/HesaiLidar_General_SDK/include/
    src/HesaiLidar_General_SDK/src/PandarGeneralRaw/include
    ${Boost_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
)
target_link_libraries(PandarGeneralSDK 
    PandarGeneral
    ${Boost_LIBRARIES}
    ${PCL_IO_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    pcap
)
ament_target_dependencies(PandarGeneralSDK)
rosidl_target_interfaces(PandarGeneralSDK ${PROJECT_NAME} "rosidl_typesupport_cpp")

###hesai node
add_executable(hesai_lidar_node
    src/main.cc
)
target_include_directories(hesai_lidar_node PRIVATE
    src/HesaiLidar_General_SDK/include
    src/HesaiLidar_General_SDK/src/PandarGeneralRaw/include
    ${PCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)
target_link_libraries(hesai_lidar_node 
    ${PCL_IO_LIBRARIES}
    PandarGeneral
    PandarGeneralSDK
)
ament_target_dependencies(hesai_lidar_node
  ${ament_dependencies}
)

install(TARGETS hesai_lidar_node PandarGeneralSDK PandarGeneral
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
