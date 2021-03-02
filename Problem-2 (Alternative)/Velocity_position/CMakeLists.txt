cmake_minimum_required(VERSION 2.8.3)
project(fares_packages)

#set(CMAKE_PREFIX_PATH "/usr/local/lib")

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  image_transport
  cv_bridge
tf


 )



include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} 
/usr/local/include/)

#
###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
    roscpp
    image_transport
    cv_bridge
	tf
)


###########
## Build ##
###########



add_executable(PC  src/Waypoints.cpp)
target_link_libraries(PC ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${OpenCV_LIBS} ${OpenCV_LIBRARIES} /usr/local/lib/libaruco.so )
add_dependencies(PC  PC)
#
#

install(TARGETS PC
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)



