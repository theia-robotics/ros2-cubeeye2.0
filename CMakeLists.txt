cmake_minimum_required(VERSION 3.5)
project(cubeeye_camera)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
add_compile_options(-Wno-unused-parameter)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# TODO: Add this back in as a separate repo
# rosidl_generate_interfaces(${PROJECT_NAME}_interfaces
#   "srv/LastState.srv"
#   "srv/LastError.srv"
#   "srv/Scan.srv"
#   "srv/Connect.srv"
#   "srv/Run.srv"
#   "srv/Stop.srv"
#   "srv/Disconnect.srv"
# )

add_library(${PROJECT_NAME} SHARED
  src/CameraModule.cpp
  src/CubeEyeCameraNode.cpp
  src/ModelParameter.cpp
  src/SCubeModelParameter.cpp
  src/ICubeModelParameter.cpp
)
# rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_include_directories(${PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/cubeeye2.0/include/CubeEye
)

target_link_directories(${PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/cubeeye2.0/lib
  ${CMAKE_CURRENT_SOURCE_DIR}/cubeeye2.0/thirdparty/liblive555/lib/Release
  ${CMAKE_CURRENT_SOURCE_DIR}/cubeeye2.0/thirdparty/libopencv/lib
)

ament_target_dependencies(${PROJECT_NAME}
   rclcpp
   rclcpp_components
   rclcpp_action
   cv_bridge
   std_srvs
   std_msgs
   sensor_msgs
)

target_link_libraries(${PROJECT_NAME}
  CubeEye
)

add_executable(${PROJECT_NAME}_node
  src/CubeEyeCameraNodeStandalone.cpp
)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

# Use old RPATH.
# https://stackoverflow.com/questions/58997230/cmake-project-fails-to-find-shared-library
# target_link_libraries(${PROJECT_NAME}_node "-Wl,--disable-new-dtags")

# Install binaries
install(TARGETS
  ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install third party dependencies:
# NOTE: Shouldn't conflict with system libraries if linking the version
# with the explicit name
install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/cubeeye2.0/lib/libCubeEye.so
  ${CMAKE_CURRENT_SOURCE_DIR}/cubeeye2.0/thirdparty/libopencv/lib/libopencv_core.so.3.4
  ${CMAKE_CURRENT_SOURCE_DIR}/cubeeye2.0/thirdparty/libopencv/lib/libopencv_imgproc.so.3.4
  ${CMAKE_CURRENT_SOURCE_DIR}/cubeeye2.0/thirdparty/liblive555/lib/Release/libLive555ClientSource.so
  ${CMAKE_CURRENT_SOURCE_DIR}/cubeeye2.0/thirdparty/liblive555/lib/Release/libliveMedia.so.107
  ${CMAKE_CURRENT_SOURCE_DIR}/cubeeye2.0/thirdparty/liblive555/lib/Release/libBasicUsageEnvironment.so.1
  ${CMAKE_CURRENT_SOURCE_DIR}/cubeeye2.0/thirdparty/liblive555/lib/Release/libUsageEnvironment.so.3
  ${CMAKE_CURRENT_SOURCE_DIR}/cubeeye2.0/thirdparty/liblive555/lib/Release/libgroupsock.so.30.0.10
  ${CMAKE_CURRENT_SOURCE_DIR}/cubeeye2.0/thirdparty/liblive555/lib/Release/libgroupsock.so.30
  DESTINATION lib
)

rclcpp_components_register_nodes(${PROJECT_NAME} "CubeEyeCameraNode")

ament_export_dependencies(rosidl_default_runtime)
ament_package()
