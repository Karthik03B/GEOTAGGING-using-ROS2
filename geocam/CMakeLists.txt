cmake_minimum_required(VERSION 3.8)
project(geocam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
 

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)
add_executable(campub src/cam_pub.cpp)
ament_target_dependencies( campub rclcpp sensor_msgs cv_bridge OpenCV)
add_executable(camsub src/cam_sub.cpp)
ament_target_dependencies( camsub rclcpp sensor_msgs cv_bridge OpenCV)
add_executable(capim src/imagesave.cpp)
ament_target_dependencies(capim rclcpp sensor_msgs OpenCV cv_bridge)
add_executable(trigger src/trigger.cpp)
ament_target_dependencies(trigger rclcpp std_msgs)
add_executable(fb src/buffer.cpp)
ament_target_dependencies(fb rclcpp sensor_msgs OpenCV cv_bridge)
add_executable(bytes src/image_bytes.cpp)
ament_target_dependencies( bytes rclcpp sensor_msgs cv_bridge OpenCV)
#add_executable(save_trig src/new_save_trig.cpp)
#ament_target_dependencies( save_trig rclcpp sensor_msgs cv_bridge OpenCV)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

 
  install(TARGETS
  campub
  camsub
  capim
  trigger 
  fb
  bytes
  #save_trig
  DESTINATION lib/${PROJECT_NAME})

  #ament_python_install_packages(${PROJECT_NAME})
  install(PROGRAMS
  scripts/rc_trigger.py
  DESTINATION lib/${PROJECT_NAME})

  install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

endif()

ament_package()
