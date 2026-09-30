cmake_minimum_required(VERSION 3.8)
project(multi_tracking)
find_package(ament_cmake REQUIRED)

# Keep launch/src layout. Scripts import the sibling, ROS-independent core.
install(PROGRAMS src/real_time_6_5.py src/getdata.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES src/tracking_core.py DESTINATION lib/${PROJECT_NAME})
install(FILES src/adaboost_trained_data_mess_430.txt
  DESTINATION share/${PROJECT_NAME}/src)
install(DIRECTORY launch rviz config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(core test/test_tracking_core.py TIMEOUT 90)
  ament_add_pytest_test(ros_nodes test/test_ros_nodes.py TIMEOUT 90)
  ament_add_pytest_test(launch_files test/test_launch.py TIMEOUT 60)
endif()
ament_package()
