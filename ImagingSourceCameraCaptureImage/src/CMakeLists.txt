cmake_minimum_required(VERSION 2.8.3)
project(camera_publish)
set(CMAKE_INCLUDE_CURRENT_DIR ON)


## Find catkin and any catkin packages
  
  
  
  
find_package(catkin REQUIRED COMPONENTS geometry_msgs dynamic_reconfigure cv_bridge image_transport roscpp rospy camera_info_manager std_msgs message_generation)
FIND_PACKAGE(OpenCV REQUIRED) # OpenCV 3
include_directories(${OpenCV_INCLUDE_DIRS})
generate_dynamic_reconfigure_options(cfg/Tutorials.cfg) 

## Declare ROS messages and services
#add_message_files(FILES Num.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs)



add_message_files(
   FILES
   my_msg.msg
#   Message2.msg
 )
 
  generate_messages(
   DEPENDENCIES
   geometry_msgs #std_msgs
 )
 
 
 ## Declare a catkin package
catkin_package(CATKIN_DEPENDS dynamic_reconfigure geometry_mesgs roscpp std_msgs message_runtime)



## Build talker and listener
#include_directories(include ${catkin_INCLUDE_DIRS} )


#set(EXECUTABLE_OUTPUT_PATH ../../bin) # this line is to put the executable file in another folder
#set(app_SOURCES talker_old ../../_libs/xiAPIplusOpenCV/xiApiPlusOcv)


set(THREADS_PREFER_PTHREAD_FLAG ON)
# CPP includes
include_directories(../../_libs/xiAPI)
include_directories(../../_libs/xiAPIplusOpenCV)
include_directories($(OPENCV_DIR)/../../include)
include_directories(../../_libs)
find_package(Eigen3 REQUIRED) 
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(OpenGL REQUIRED)
include_directories(OpenGL_INCLUDE_DIR)

find_package(Pangolin 0.4 REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})


if(UNIX)
#     link_libraries(m3api)
    link_libraries(opencv_core)
    link_libraries(opencv_highgui)
    link_libraries(opencv_imgproc)
else()
    link_libraries(xiapi64)
    link_libraries(opencv_world310d)
    link_directories(../../../bin)
endif()

link_directories($(OPENCV_DIR)/lib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -g -Wall")


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

find_package(PkgConfig REQUIRED)
find_package(X11 REQUIRED)
message(STATUS "X11_FOUND = ${X11_FOUND}")
message(STATUS "X11_INCLUDE_DIR = ${X11_INCLUDE_DIR}")
message(STATUS "X11_LIBRARIES = ${X11_LIBRARIES}")

include_directories(${X11_INCLUDE_DIR})
link_directories(${X11_LIBRARIES})

pkg_search_module(GLIB REQUIRED glib-2.0)
pkg_search_module(GTK REQUIRED gtk+-2.0)

pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0)
pkg_check_modules(TCAMLIB tcam)

# Setup CMake to use GTK+, tell the compiler where to look for headers
# and to the linker where to look for libraries
INCLUDE_DIRECTORIES(${GTK_INCLUDE_DIRS})
LINK_DIRECTORIES(${GTK_LIBRARY_DIRS})

# Add other flags to the compiler
ADD_DEFINITIONS(${GTK_CFLAGS_OTHER})

# these three libraries should be search separately, rather than put them under one shared name.
pkg_search_module(GSTER REQUIRED gstreamer-1.0>=1.4)
pkg_search_module(GSTVIDEO REQUIRED gstreamer-video-1.0>=1.4)
pkg_search_module(GSTAPP REQUIRED   gstreamer-app-1.0>=1.4)

find_package(OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIR})
include_directories( ${CMAKE_CURRENT_BINARY_DIR}  ${GSTREAMER_INCLUDE_DIRS} ${TCAM_INCLUDE_DIRS} )

add_definitions(${GSTREAMER_CFLAGS_OTHER})  
#add_executable(talker src/talker_use_old_images.cpp src/tcamimage.cpp src/tcamcamera.cpp )

add_executable(talker src/talker.cpp src/tcamimage.cpp src/tcamcamera.cpp )
add_executable(my_subscriber src/my_subscriber.cpp)
target_link_libraries(my_subscriber  ${catkin_LIBRARIES} )


target_include_directories(talker  PRIVATE ${GLIB_INCLUDE_DIRS} ${Pangolin_INCLUDE_DIRS} ${OPENGL_INCLUDE_DIRS} ${GTK_INCLUDE_DIRS} ${GSTER_INCLUDE_DIRS}  ${GSTVIDEO_INCLUDE_DIRS}  ${GSTAPP_INCLUDE_DIRS} ${X11_INCLUDE_DIRS})
target_link_libraries(talker ${Pangolin_LIBRARIES} ${OPENGL_LIBRARIES} ${OpenCV_LIBRARIES} ${GLIB_LDFLAGS}  ${GTK_LIBRARIES} ${GSTER_LIBRARIES} ${GSTVIDEO_LIBRARIES} ${GSTAPP_LIBRARIES} ${X11_LIBRARIES}  ${catkin_LIBRARIES})
#add_dependencies(talker camera_publish_generate_messages_cpp)
target_link_libraries(talker ${TCAMLIB_LIBRARIES} ${GSTREAMER_LIBRARIES} )

