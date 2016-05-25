cmake_minimum_required(VERSION 2.8.3)
project(cam_exploration)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs tf)
#find_package(Boost REQUIRED COMPONENTS pending/disjoint_sets)

catkin_package(
    INCLUDE_DIRS include
)



include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(cam_exploration src/cam_exploration.cpp
				src/MarkerPublisher.cpp
				src/RobotMotion.cpp
				src/MapServer.cpp
				src/frontier.cpp
				src/frontiersMap.cpp
				src/strategy/frontierValue.cpp
				src/strategy/goalSelector.cpp
				src/strategy/replan.cpp
				src/strategy/AStar.cpp)



target_link_libraries(cam_exploration ${catkin_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} )
#add_dependencies(talker beginner_tutorials_generate_messages_cpp)
