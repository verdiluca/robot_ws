# CMake generated Testfile for 
# Source directory: /home/vp/robot_ws/src/ros2_odometry_estimation/test
# Build directory: /home/vp/robot_ws/build/odometry_estimator/test
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(vehicle_models_test "/usr/bin/python3" "-u" "/opt/ros/foxy/share/ament_cmake_test/cmake/run_test.py" "/home/vp/robot_ws/build/odometry_estimator/test_results/odometry_estimator/vehicle_models_test.gtest.xml" "--package-name" "odometry_estimator" "--output-file" "/home/vp/robot_ws/build/odometry_estimator/ament_cmake_gtest/vehicle_models_test.txt" "--command" "/home/vp/robot_ws/build/odometry_estimator/test/vehicle_models_test" "--gtest_output=xml:/home/vp/robot_ws/build/odometry_estimator/test_results/odometry_estimator/vehicle_models_test.gtest.xml")
set_tests_properties(vehicle_models_test PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/vp/robot_ws/build/odometry_estimator/test/vehicle_models_test" TIMEOUT "60" WORKING_DIRECTORY "/home/vp/robot_ws/build/odometry_estimator/test" _BACKTRACE_TRIPLES "/opt/ros/foxy/share/ament_cmake_test/cmake/ament_add_test.cmake;118;add_test;/opt/ros/foxy/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/foxy/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/vp/robot_ws/src/ros2_odometry_estimation/test/CMakeLists.txt;1;ament_add_gtest;/home/vp/robot_ws/src/ros2_odometry_estimation/test/CMakeLists.txt;0;")
subdirs("../gtest")
