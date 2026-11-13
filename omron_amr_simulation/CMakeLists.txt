cmake_minimum_required(VERSION 3.8)
project(omron_amr_simulation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch models urdf worlds DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
	find_package(ament_cmake_pytest REQUIRED)
	ament_add_pytest_test(test_object_layouts test/test_object_layouts.py)
endif()

ament_package()