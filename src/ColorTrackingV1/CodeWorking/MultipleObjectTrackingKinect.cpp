#include <sstream>
#include <string>
#include <iostream>
#include <vector>

#include "Object.h"
#include "Object.cpp"

#include "ros/ros.h"
#include "std_msgs/String.h"
//Use image_transport for publishing and subscribing to images in ROS
#include <cv_bridge/cv_bridge.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <sensor_msgs/image_encodings.h>
//Include headers for OpenCV Image processing
#include <opencv2/imgproc/imgproc.hpp>
//Include headers for OpenCV GUI handling
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>

#include <mypkg/Object.h>
#include <mypkg/Object3D.h>


//initial min and max HSV filter values.
//these will be changed using trackbars
int H_MIN = 0;
int H_MAX = 256;
int S_MIN = 0;
int S_MAX = 256;
int V_MIN = 0;
int V_MAX = 256;
//default capture width and height
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;
//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS=50;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 20*20;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;
//names that will appear at the top of each window
const string windowName = "Original Image";
const string windowName1 = "HSV Image";
const string windowName2 = "Thresholded Image";
const string windowName3 = "After Morphological Operations";
const string trackbarWindowName = "Trackbars";


//Image of depth of my world
cv_bridge::CvImagePtr cv_ptr;

//The following for canny edge detec
Mat dst, detected_edges;
Mat src, src_gray;
int edgeThresh = 1;
int lowThreshold;
int const max_lowThreshold = 50;
int ratio = 3;
int kernel_size = 3;
const char* window_name = "Edge Map";

namespace enc = sensor_msgs::image_encodings;

void CannyThreshold(int, void*)
{
     /// Reduce noise with a kernel 3x3
	blur( src_gray, detected_edges, Size(3,3) );

	     /// Canny detector\
	Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );

	     /// Using Canny's output as a mask, we display our result
	dst = Scalar::all(0);
	    
	src.copyTo( dst, detected_edges);
	imshow( window_name, dst );
}


void on_trackbar( int, void* )
{//This function gets called whenever a
	// trackbar position is changed

}

string intToString(int number){

	std::stringstream ss;
	ss << number;
	return ss.str();
}

void DepthCallback(const sensor_msgs::ImageConstPtr& depth_image)
{
	//ROS_INFO("DEPTH");
	//Mat MyDepthMat;

	//cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(depth_image, enc::TYPE_16UC1);

	//MyDepthMat = cv_ptr->image;

	//int depth = 0;

	//ros::Subscriber Depth2DSub = n.subscribe("Objets", 1000, 2DCallback);
		
			//mypkg::Object3D MessageObjet;
			//ros::NodeHandle j;
			//ROS_INFO("%d",i);
			//ROS_INFO(MyObjects.at(i).getType().c_str());
			//ros::Publisher MesObjects = j.advertise<mypkg::Object3D>("ObjetsDepths",100);

			

			//depth = cv_ptr->image.at<short int>(cv::Point(MyObjects.at(i).getXPos(),MyObjects.at(i).getYPos()));

			/*MessageObjet.x = MyObjects.at(i).getXPos();
			MessageObjet.y = MyObjects.at(i).getYPos();
			MessageObjet.z = depth;
			MessageObjet.type = MyObjects.at(i).getType();

			MesObjects.publish(MessageObjet);
			ros::spinOnce();
		
	

	//int depth = cv_ptr->image.at<short int>(cv::Point(253,438));//you can change 240,320 to your interested pixel

	//int X = MyObjects.at(1).getXPos();
	int u = MyObjects.size();*/
	//ROS_INFO("SIZE : %d ", u);
	//ROS_INFO("DEPTH : %d", X);
    //ROS_INFO("Depth: %d", depth);

	//cv::imshow("DEPTH", cv_ptr->image);
}
void createTrackbars(){
	//create window for trackbars
	namedWindow(trackbarWindowName,0);
	//create memory to store trackbar name on window
	char TrackbarName[50];
	sprintf( TrackbarName, "H_MIN", H_MIN);
	sprintf( TrackbarName, "H_MAX", H_MAX);
	sprintf( TrackbarName, "S_MIN", S_MIN);
	sprintf( TrackbarName, "S_MAX", S_MAX);
	sprintf( TrackbarName, "V_MIN", V_MIN);
	sprintf( TrackbarName, "V_MAX", V_MAX);
	//create trackbars and insert them into window
	//3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
	//the max value the trackbar can move (eg. H_HIGH), 
	//and the function that is called whenever the trackbar is moved(eg. on_trackbar)
	//                                  ---->    ---->     ---->      
	createTrackbar( "H_MIN", trackbarWindowName, &H_MIN, H_MAX, on_trackbar );
	createTrackbar( "H_MAX", trackbarWindowName, &H_MAX, H_MAX, on_trackbar );
	createTrackbar( "S_MIN", trackbarWindowName, &S_MIN, S_MAX, on_trackbar );
	createTrackbar( "S_MAX", trackbarWindowName, &S_MAX, S_MAX, on_trackbar );
	createTrackbar( "V_MIN", trackbarWindowName, &V_MIN, V_MAX, on_trackbar );
	createTrackbar( "V_MAX", trackbarWindowName, &V_MAX, V_MAX, on_trackbar );
}

void drawObject(vector<Object> theObjects,Mat &frame, Mat &temp, vector< vector<Point> > contours, vector<Vec4i> hierarchy){

	//MyObjects.push_back(theObjects);
	int depth = 0;

	for(int i =0; i<theObjects.size(); i++)
	{
		//Actualize object of scene
		//Put the depth of my world for the points
		depth = cv_ptr->image.at<short int>(cv::Point(theObjects.at(i).getXPos(),theObjects.at(i).getYPos()));

		cv::drawContours(frame,contours,i,theObjects.at(i).getColor(),3,8,hierarchy);
		cv::circle(frame,cv::Point(theObjects.at(i).getXPos(),theObjects.at(i).getYPos()),5,theObjects.at(i).getColor(),1);
		cv::putText(frame,intToString(theObjects.at(i).getXPos())+ " , " + intToString(theObjects.at(i).getYPos()),cv::Point(theObjects.at(i).getXPos(),theObjects.at(i).getYPos()+20),1,1,theObjects.at(i).getColor());
		cv::putText(frame,theObjects.at(i).getType(),cv::Point(theObjects.at(i).getXPos(),theObjects.at(i).getYPos()-20),1,2,theObjects.at(i).getColor());
	
		//Store the recognize object into my message
		mypkg::Object3D MessageObjet;
		ros::NodeHandle j;
		
		ros::Publisher MesObjects = j.advertise<mypkg::Object3D>("Objects", 10);

		MessageObjet.x = theObjects.at(i).getXPos();
		MessageObjet.y = theObjects.at(i).getYPos();
		MessageObjet.z = depth;
		MessageObjet.type = theObjects.at(i).getType();

		MesObjects.publish(MessageObjet);
		//ROS_INFO("Publish");
		ros::spinOnce();
	}


}

void morphOps(Mat &thresh){

	//create structuring element that will be used to "dilate" and "erode" image.
	//the element chosen here is a 3px by 3px rectangle
	Mat erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));
	//dilate with larger element so make sure object is nicely visible
	Mat dilateElement = getStructuringElement( MORPH_RECT,Size(8,8));

	erode(thresh,thresh,erodeElement);
	erode(thresh,thresh,erodeElement);

	dilate(thresh,thresh,dilateElement);
	dilate(thresh,thresh,dilateElement);
}

void trackFilteredObject(Object theObject,Mat threshold,Mat HSV, Mat &cameraFeed){

	vector <Object> objects;
	Mat temp;
	threshold.copyTo(temp);
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );
	//use moments method to find our filtered object
	double refArea = 0;
	bool objectFound = false;
	if (hierarchy.size() > 0) {
		int numObjects = hierarchy.size();
		//if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
		if(numObjects<MAX_NUM_OBJECTS){

			for (int index = 0; index >= 0; index = hierarchy[index][0]) {

				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;

		//if the area is less than 20 px by 20px then it is probably just noise
		//if the area is the same as the 3/2 of the image size, probably just a bad filter
		//we only want the object with the largest area so we safe a reference area each
				//iteration and compare it to the area in the next iteration.
				if(area>MIN_OBJECT_AREA){

					Object object;
					
					object.setXPos(moment.m10/area);
					object.setYPos(moment.m01/area);
					object.setType(theObject.getType());
					object.setColor(theObject.getColor());

					objects.push_back(object);

					objectFound = true;

				}
				else objectFound = false;
			}
			//let user know you found an object
			if(objectFound ==true){
				//draw object location on screen
				
				drawObject(objects,cameraFeed,temp,contours,hierarchy);
				
				

				
			}

		}
		else putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",Point(0,50),1,2,Scalar(0,0,255),2);
	}
}

void callback(const sensor_msgs::ImageConstPtr& original_image)
{
	Mat MyCameraFeed;
	Mat HSV;
	Mat threshold;
	cv_bridge::CvImagePtr cv_ptr;
	bool calibrationMode = false;

	cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
	MyCameraFeed = cv_ptr->image;

	src = MyCameraFeed;

	if(calibrationMode){
		//create slider bars for HSV filtering
		createTrackbars();
	}

	if(calibrationMode==true){

		//need to find the appropriate color range values
		// calibrationMode must be false
		Object blue("blue"), yellow("yellow"), red("red"), green("green");
		//if in calibration mode, we track objects based on the HSV slider values.
			cvtColor(MyCameraFeed,HSV,COLOR_BGR2HSV);
			inRange(HSV,Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX),threshold);
			morphOps(threshold);
			imshow(windowName2,threshold);
		
		//the folowing for canny edge detec	  		
			/// Create a matrix of the same type and size as src (for dst)
	  		dst.create( src.size(), src.type() );
	  		/// Convert the image to grayscale
	  		cvtColor( src, src_gray, CV_BGR2GRAY );
	  		/// Create a window
	  		namedWindow( window_name, CV_WINDOW_AUTOSIZE );
	  		/// Create a Trackbar for user to enter threshold
	  		//createTrackbar( "Min Threshold:", window_name, &lowThreshold, max_lowThreshold, CannyThreshold );
	  		createTrackbar( "Min Threshold:", window_name, &lowThreshold, max_lowThreshold, CannyThreshold );
	  		
	  		/// Show the image
			trackFilteredObject(yellow,threshold,HSV,MyCameraFeed);
		}
		else{
			//create some temp fruit objects so that
			//we can use their member functions/information
			Object blue("blue"), yellow("yellow"), red("red"), green("green");

			//first find blue objects
			/*cvtColor(MyCameraFeed,HSV,COLOR_BGR2HSV);
			inRange(HSV,blue.getHSVmin(),blue.getHSVmax(),threshold);
			morphOps(threshold);
			trackFilteredObject(blue,threshold,HSV,MyCameraFeed);*/
			//then yellows
			cvtColor(MyCameraFeed,HSV,COLOR_BGR2HSV);
			inRange(HSV,yellow.getHSVmin(),yellow.getHSVmax(),threshold);
			morphOps(threshold);
			trackFilteredObject(yellow,threshold,HSV,MyCameraFeed);
			//then reds
			cvtColor(MyCameraFeed,HSV,COLOR_BGR2HSV);
			inRange(HSV,red.getHSVmin(),red.getHSVmax(),threshold);
			morphOps(threshold);
			trackFilteredObject(red,threshold,HSV,MyCameraFeed);
			//then greens
			cvtColor(MyCameraFeed,HSV,COLOR_BGR2HSV);
			inRange(HSV,green.getHSVmin(),green.getHSVmax(),threshold);
			morphOps(threshold);
			trackFilteredObject(green,threshold,HSV,MyCameraFeed);

		}
		//ROS_INFO("Show Frame");
		//show frames 
		imshow(windowName2,threshold);

		imshow(windowName,MyCameraFeed);
		cv::waitKey(3);
		//imshow(windowName1,HSV);
		

}
int main(int argc, char* argv[])
{
	//if we would like to calibrate our filter values, set to true.
	

	ros::init(argc, argv, "Essai");
	ros::NodeHandle n;



	image_transport::ImageTransport depth(n);
	image_transport::Subscriber SubDepth = depth.subscribe("/camera/depth_registered/image_raw",1,DepthCallback);

	image_transport::ImageTransport it(n);
	ROS_INFO("Suscribe");
	
	image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_rect_color",1,callback);
	
	//ros::Duration(5).sleep(); 
	//ros::wait(20);



	
    ros::spin();
	
	
	return 0;
}
