cmake_minimum_required(VERSION 3.5)
project(teslatronic_client)

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(_INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME})
set(_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(teslatronic_interfaces REQUIRED)

add_executable(
	${PROJECT_NAME}
		${_INC_DIR}/external_api/TeslatronicClientExternalBridge.h
	
		${_SRC_DIR}/external_api/TeslatronicClientExternalBridge.cpp
		${_SRC_DIR}/main.cpp
)

target_include_directories(
	${PROJECT_NAME}
		PRIVATE
			${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(
	${PROJECT_NAME}
		rclcpp
		rclcpp_action
		teslatronic_interfaces
)

install(
	TARGETS
  		${PROJECT_NAME}
	DESTINATION
		lib/${PROJECT_NAME}
)

ament_package()
