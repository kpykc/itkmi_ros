cmake_minimum_required(VERSION 2.8.3)
project(itkmi)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/;/usr/lib/InsightToolkit/")
# set(ITK_DIR ${ITK_DIR} "/usr/lib/InsightToolkit/")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED COMPONENTS
    std_msgs geometry_msgs sensor_msgs
	roscpp rosbag message_filters image_geometry
	eigen_conversions cmake_modules cv_bridge
    rospy message_generation
)

find_package( cmake_modules REQUIRED)
find_package( Eigen3 REQUIRED )
find_package( PCL REQUIRED)
find_package( OpenCV REQUIRED core highgui imgproc features2d )
find_package( Boost 1.40 COMPONENTS program_options timer filesystem REQUIRED )

find_package(ZLIB REQUIRED)
find_package(MATIO)
if(MATIO_FOUND)
    message(STATUS "MATIO includes: ${MATIO_INCLUDE_DIRS}")
    # ADD_DEFINITIONS( "-DNICE_USELIB_MATIO")
    INCLUDE_DIRECTORIES(${MATIO_INCLUDE_DIRS})
    # set (external_deps ${external_deps} "MATIO")
    # link_directories(${MATIO_INCLUDE_DIRS})
else()
    message(FATAL_ERROR "MATIO not found")
endif()

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS} )


find_package(ITK REQUIRED
    NAMES ITK InsightToolkit
    CONFIGS ITKConfig.cmake
)

include(${ITK_USE_FILE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -D_REENTRANT --std=c++11")
# itkopencv bridge has problems with C++11

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -D_REENTRANT")


add_message_files(
  FILES
  MI.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
    DEPENDS system_lib
    CATKIN_DEPENDS
        std_msgs
        geometry_msgs
        sensor_msgs
        rosgraph_msgs
        roscpp
        rosbag
        message_filters
        image_geometry
		# pcl_ros
		eigen_conversions
		cmake_modules
		cv_bridge
        message_runtime
    INCLUDE_DIRS ${catkin_INCLUDE_DIRS}
    #LIBRARIES
)

add_executable(itkmi src/itkmi.cxx )
target_link_libraries(itkmi
    ${catkin_LIBRARIES}
    ${ITK_LIBRARIES}
    ${OpenCV_LIBS}
)

