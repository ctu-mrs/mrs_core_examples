get_filename_component(TEST_NAME "${CMAKE_CURRENT_SOURCE_DIR}" NAME)

add_ros_isolated_launch_test(test.py TIMEOUT 300)
