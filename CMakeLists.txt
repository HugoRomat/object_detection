cmake_minimum_required(VERSION 2.8.3)
project(mypkg)



## Find catkin and any catkin packages


find_package(catkin REQUIRED COMPONENTS 
    cv_bridge roscpp rospy sensor_msgs std_msgs image_transport genmsg message_filters pcl_ros tf
   )

find_package(PCL 1.3 REQUIRED COMPONENTS common io)

FIND_PACKAGE(OpenCV REQUIRED) # tested on 2.3.1
FIND_PACKAGE(Qt4 COMPONENTS QtCore QtGui QtNetwork REQUIRED)


SET(INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
	${CMAKE_CURRENT_SOURCE_DIR}/../include
)

INCLUDE(${QT_USE_FILE})

SET(LIBRARIES
	${OpenCV_LIBS} 
	${QT_LIBRARIES} 
)

add_message_files(
  FILES
  Object.msg
  Object3D.msg
)

# Make sure the compiler can find include files from our library.
INCLUDE_DIRECTORIES(${INCLUDE_DIRS})
FIND_PACKAGE(Qt4 COMPONENTS QtCore QtGui QtNetwork REQUIRED)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

#add_executable(Suscriber_Video src/Suscriber_Video.cpp)
#target_link_libraries(Suscriber_Video ${catkin_LIBRARIES})


#add_executable(Suscriber_findobject3d src/Suscriber_findobject3d.cpp)
#target_link_libraries(Suscriber_findobject3d  ${catkin_LIBRARIES})

#add_executable(Object src/ColorTracking/Object.cpp)
#target_link_libraries(Object  ${catkin_LIBRARIES})

#add_executable(Tuto src/Tuto.cpp)
#target_link_libraries(Tuto  ${catkin_LIBRARIES})

#add_executable(TutoThreshold src/TutoThreshold.cpp)
#target_link_libraries(TutoThreshold  ${catkin_LIBRARIES})

#add_executable(multipleObjectTracking src/multipleObjectTracking.cpp)
#target_link_libraries(multipleObjectTracking  ${catkin_LIBRARIES})

add_executable(MultipleObjectTrackingKinect2 src/ColorTrackingV2/MultipleObjectTrackingKinect2.cpp)
target_link_libraries(MultipleObjectTrackingKinect2  ${catkin_LIBRARIES})

add_executable(ColorTrackingEssaiKinect  src/ColorTrackingEssaiKinect/MultipleObjectTrackingEssaiKinect.cpp)
target_link_libraries(ColorTrackingEssaiKinect ${catkin_LIBRARIES})

add_executable(ColorTrackingEssaiKinectGazebo  src/ColorTrackingEssaiKinectGazebo/MultipleObjectTrackingEssaiKinect.cpp)
target_link_libraries(ColorTrackingEssaiKinectGazebo ${catkin_LIBRARIES})
add_executable(BoundingBox  src/ColorTrackingEssaiKinectGazebo/CreateBox.cpp)
target_link_libraries(BoundingBox ${catkin_LIBRARIES})

add_executable(BoundingBoxRviz  src/ColorTrackingEssaiKinectGazebo/CreateBoxRviz.cpp)
target_link_libraries(BoundingBoxRviz ${catkin_LIBRARIES})

add_executable(BorderKinect  src/PointCloud/Border.cpp)
target_link_libraries(BorderKinect ${catkin_LIBRARIES})

add_executable(BorderTuto  src/PointCloud/BorderTuto.cpp)
target_link_libraries(BorderTuto ${catkin_LIBRARIES})

add_executable(Segmentation src/PointCloud/Segmentation.cpp)
target_link_libraries(Segmentation ${catkin_LIBRARIES})

add_executable(pcd_write src/PointCloud/pcd_write.cpp)
target_link_libraries(pcd_write ${catkin_LIBRARIES})

add_executable(smooth src/PointCloud/smooth.cpp)
target_link_libraries(smooth ${catkin_LIBRARIES})

add_executable(RemovingOutliners src/PointCloud/RemovingOutliners.cpp)
target_link_libraries(RemovingOutliners ${catkin_LIBRARIES})

add_executable(cluster src/PointCloud/cluster.cpp)
target_link_libraries(cluster ${catkin_LIBRARIES})

add_executable(ObjectRecognition src/PointCloud/ObjectRecognition.cpp)
target_link_libraries(ObjectRecognition ${catkin_LIBRARIES})

add_executable(colorCloud src/PointCloud/colorCloud.cpp)
target_link_libraries(colorCloud ${catkin_LIBRARIES})


add_executable(gravity_center src/PointCloud/gravity_center.cpp)
target_link_libraries(gravity_center ${catkin_LIBRARIES})

add_executable(MultipleObjectTrackingKinect src/Main.cpp
                                            src/MultipleObjectTrackingKinect.cpp
                                            src/Object.cpp
)
target_link_libraries(MultipleObjectTrackingKinect ${catkin_LIBRARIES})

#add_executable(TutoCannyThreshold src/TutoCannyThreshold.cpp)
#target_link_libraries(TutoCannyThreshold  ${catkin_LIBRARIES})

#target_link_libraries(Suscriber_findobject3d  ${QT_LIBRARIES})
