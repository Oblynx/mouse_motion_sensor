cmake_minimum_required(VERSION 2.8.3)
project(pandora_mouse_hardware_interface)
find_package(catkin REQUIRED COMPONENTS
	roscpp
	roslint
	std_msgs
  message_generation
)
# delete_this github
############## generate messages #####################
add_message_files(
	FILES
	mouseMeasurement.msg
)
generate_messages(
	DEPENDENCIES
	std_msgs
)
############## export dependencies ###################
catkin_package(
	# INCLUDE_DIRS include
	# LIBRARIES pandora_mouse_hardware_interface
	CATKIN_DEPENDS
		roscpp
		roslint
		message_runtime
	# DEPENDS system_lib
)
include_directories(
	include
	${catkin_INCLUDE_DIRS}
)
############## mouse hardware interface ##############
add_library(mouse_hardware_interface
	src/mouse_hardware_interface.cpp)
target_link_libraries(mouse_hardware_interface
	${CATKIN_LIBRARIES})
###...${catkin_LIBRARIES}) ???
add_executable(mouse_hardware_interface_node
	src/mouse_hardware_interface_node.cpp)
target_link_libraries(mouse_hardware_interface_node
	${catkin_LIBRARIES}
	mouse_hardware_interface
)
# set(ROSLINT_CPP_OPTS
	# "--filter=-whitespace/end_of_line,-build/include_order,-whitespace/blank_line,-whitespace/parens,-whitespace/comments,-whitespace/braces")
FILE(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
	RELATIVE ${PROJECT_SOURCE_DIR}
		src/*.cpp
)
roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
