# CMake generated Testfile for 
# Source directory: /home/xingxun/uav_control/src/utils/uav_utils
# Build directory: /home/xingxun/uav_control/build/uav_utils
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_uav_utils_gtest_uav_utils-test "/home/xingxun/uav_control/build/uav_utils/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/xingxun/uav_control/build/uav_utils/test_results/uav_utils/gtest-uav_utils-test.xml" "--return-code" "/home/xingxun/uav_control/devel/.private/uav_utils/lib/uav_utils/uav_utils-test --gtest_output=xml:/home/xingxun/uav_control/build/uav_utils/test_results/uav_utils/gtest-uav_utils-test.xml")
set_tests_properties(_ctest_uav_utils_gtest_uav_utils-test PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/catkin/cmake/test/gtest.cmake;98;catkin_run_tests_target;/opt/ros/noetic/share/catkin/cmake/test/gtest.cmake;37;_catkin_add_google_test;/home/xingxun/uav_control/src/utils/uav_utils/CMakeLists.txt;94;catkin_add_gtest;/home/xingxun/uav_control/src/utils/uav_utils/CMakeLists.txt;0;")
subdirs("gtest")