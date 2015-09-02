#include <sstream>
#include <string>
#include <iostream>
#include <vector>

#include "Object.h"
#include "MultipleObjectTrackingKinect.h"


#include "ros/ros.h"
#include "std_msgs/String.h"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>

#include <mypkg/Object.h>
#include <mypkg/Object3D.h>
#include <cmath>
#include <tf/transform_listener.h>
#include <string> 

using namespace std;

int main(int argc, char* argv[])
{
	//if we would like to calibrate our filter values, set to true.
	ros::init(argc, argv, "Essai");
	//ros::NodeHandle n;

	/*image_transport::ImageTransport depth(n);
	//image_transport::Subscriber SubDepth = depth.subscribe("/camera/depth_registered/image_raw",1,DepthCallback);
	image_transport::Subscriber SubDepth = depth.subscribe("/head_mount_kinect/depth/image_raw",1,DepthCallback);

	image_transport::ImageTransport it(n);
	ROS_INFO("Subscribe Depth");
	
	//image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_rect_color",1,callback);
	//image_transport::Subscriber sub = it.subscribe("/head_mount_kinect/rgb/image_rect_color",1,callback); // le vrai robot
	image_transport::Subscriber sub = it.subscribe("/head_mount_kinect/rgb/image_raw",1,callback);
	ROS_INFO("Subscribe Image");*/

	
	
    ros::spin();
	
	
	return 0;
}