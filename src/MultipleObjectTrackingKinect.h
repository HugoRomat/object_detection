#include <sstream>
#include <string>
#include <iostream>
#include <vector>
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

namespace enc = sensor_msgs::image_encodings;

class MultipleObjectTrackingKinect
{
	public:	

	MultipleObjectTrackingKinect();
	void CannyThreshold(int, void*);
	static void on_trackbar( int, void* );
	string intToString(int number);
	void DepthCallback(const sensor_msgs::ImageConstPtr &depth_image);
	void createTrackbars();
	void drawObject(vector<Object> &theObjects,Mat &frame, Mat &temp, vector< vector<Point> > &contours, vector<Vec4i> &hierarchy);
	void morphOps(Mat &thresh);
	void trackFilteredObject(Object &theObject,Mat &threshold,Mat &HSV, Mat &cameraFeed);
	void callback(const sensor_msgs::ImageConstPtr& original_image);

	private:

	int H_MIN;
	int H_MAX ;
	int S_MIN;
	int S_MAX;
	int V_MIN ;
	int V_MAX;
	
	int FRAME_WIDTH;
	int FRAME_HEIGHT;
	
	int MAX_NUM_OBJECTS;
	int MIN_OBJECT_AREA;
	int MAX_OBJECT_AREA ;
	
	string windowName;
	string windowName1;
	string windowName2;
	string windowName3;
	string trackbarWindowName;
	cv_bridge::CvImagePtr cv_ptr;
	bool Contour;
	
	//The following for canny edge detec
	Mat dst, detected_edges;
	Mat src, src_gray;
	int edgeThresh;
	int lowThreshold;
	int max_lowThreshold;
	int ratio;
	int kernel_size;
	const char* window_name;

	ros::Publisher MesObjects;
};
