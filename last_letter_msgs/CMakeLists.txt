cmake_minimum_required(VERSION 2.8.3)
project(last_letter_msgs)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	nav_msgs
	geometry_msgs
	message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
	FILES
	Accelerations.msg
	SimPWM.msg
	SimServos.msg
	SimStates.msg
	Geoid.msg
	ElectricEng.msg
	Environment.msg
	SimSensor.msg
	SimSensor3.msg
	SimBarometer.msg
	SimGPS.msg
	SimSat.msg
	SimSats.msg
	DOP.msg
	RefCommands.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
	std_msgs
	nav_msgs
	geometry_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
   CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Generate package documentation with rosdoc_lite
execute_process(COMMAND rospack find last_letter_msgs
	OUTPUT_VARIABLE LAST_LETTER_MSGS_DIR OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND rosdoc_lite -o ${LAST_LETTER_MSGS_DIR}/doc/doxygen_rosdoc_lite ${LAST_LETTER_MSGS_DIR})