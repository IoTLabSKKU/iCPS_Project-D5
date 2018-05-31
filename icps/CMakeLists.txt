cmake_minimum_required(VERSION 2.8.3)
project(icps)

add_compile_options(-std=c++11)

#set(PCL_DIR "/usr/local/share/pcl-1.8/")


#for new message
#find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation tf PCL)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation tf)
add_message_files(FILES 
	GPSSample.msg
	MobilitySummary.msg
	NeighborInfo.msg
	PedestrianInfo.msg
	RSUInfo.msg
	VehicleInfo.msg
	ObjectInfo.msg
	ObjectList.msg
	SensorSpec.msg
	CommSpec.msg
	AppSpec.msg
	FuncSpec.msg
	ConfSpec.msg
	LaptopSettings.msg
)
add_service_files(FILES 
)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} include)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## middleware nodes
add_executable(spInterfaceBoost src/spInterfaceBoost.cpp src/spInterfaceConnection.cpp)
target_link_libraries(spInterfaceBoost ${catkin_LIBRARIES})
add_dependencies(spInterfaceBoost icps_generate_messages_cpp)

add_executable(obuInterfaceBoost src/obuInterfaceBoost.cpp src/obuInterfaceConnection.cpp)
target_link_libraries(obuInterfaceBoost ${catkin_LIBRARIES})
add_dependencies(obuInterfaceBoost icps_generate_messages_cpp)

add_executable(mobilityManager src/mobilityManager.cpp)
target_link_libraries(mobilityManager ${catkin_LIBRARIES})
add_dependencies(mobilityManager icps_generate_messages_cpp)

add_executable(configurationManager src/configurationManager.cpp)
target_link_libraries(configurationManager ${catkin_LIBRARIES})
add_dependencies(configurationManager icps_generate_messages_cpp)


## dummy nodes
add_executable(dummyTLV src/dummyTLV.cpp)
target_link_libraries(dummyTLV ${catkin_LIBRARIES})
add_dependencies(dummyTLV icps_generate_messages_cpp)

add_executable(dummySP src/dummySP.cpp)
target_link_libraries(dummySP ${catkin_LIBRARIES})
add_dependencies(dummySP icps_generate_messages_cpp)

add_executable(dummyOBU src/dummyOBU.cpp)
target_link_libraries(dummyOBU ${catkin_LIBRARIES})
add_dependencies(dummyOBU icps_generate_messages_cpp)

add_executable(dummyCodeTest src/dummyCodeTest.cpp)
target_link_libraries(dummyCodeTest ${catkin_LIBRARIES})
add_dependencies(dummyCodeTest icps_generate_messages_cpp)


## dummy lidar
add_executable(dummyLaptopForLidar src/dummyLaptopForLidar.cpp)
target_link_libraries(dummyLaptopForLidar ${catkin_LIBRARIES})
add_dependencies(dummyLaptopForLidar icps_generate_messages_cpp)

add_executable(dummyLidarNode src/dummyLidarNode.cpp)
target_link_libraries(dummyLidarNode ${catkin_LIBRARIES})
add_dependencies(dummyLidarNode icps_generate_messages_cpp)


## for lidar
#add_executable(laser_node src/lms5xx_node.cpp src/SickLMS5xx.cc src/SickLMS5xxBufferMonitor.cc src/SickLMS5xxMessage.cc src/Object_track_detect.cpp)
#add_dependencies(laser_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(laser_node ${catkin_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_LIBRARIES})
