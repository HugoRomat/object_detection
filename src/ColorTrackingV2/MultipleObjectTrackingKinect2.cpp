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
#include <cmath>
#include <tf/transform_listener.h>
#include <string> 
#include "std_msgs/String.h"
//#include <Transform.h>

#include "pyride_pr2/NodeStatus.h"

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

vector <Object> SceneObjects;

//Image of depth of my world
//cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage());
cv_bridge::CvImagePtr cv_Depth;
bool Contour = true;
//The following for canny edge detec
Mat dst, detected_edges;
Mat src, src_gray;
int edgeThresh = 1;
int lowThreshold;
int const max_lowThreshold = 50;
int ratio = 3;
int kernel_size = 3;
const char* window_name = "Edge Map";

ros::Publisher MesObjects;
ros::Publisher PyrideObjects;

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
	ROS_INFO("Acces a la profondeur");
	//cv_bridge::CvImagePtr cv_ptr;
	cv_Depth = cv_bridge::toCvCopy(depth_image, enc::TYPE_16UC1);
	//imshow(windowName2,cv_ptr->image);

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
	

	for(int i = 0; i<theObjects.size(); i++)
	{
		
		
		//Actualize object of scene
		//Resolution Kinect 640 x 480
		// A RAJOUTER
		cv::drawContours(frame,contours,i,theObjects.at(i).getColor(),3,8,hierarchy);
		// Fais le rond du centre de l'objet
		cv::circle(frame,cv::Point(theObjects.at(i).getXPos(),theObjects.at(i).getYPos()),5,theObjects.at(i).getColor(),1);
		
		cv::putText(frame,intToString(theObjects.at(i).getXPos())+ " , " + intToString(theObjects.at(i).getYPos()),cv::Point(theObjects.at(i).getXPos(),theObjects.at(i).getYPos()-20),1,1,theObjects.at(i).getColor());
		//cv::putText(frame,theObjects.at(i).getType(),cv::Point(theObjects.at(i).getXPos(),theObjects.at(i).getYPos()-20),1,2,theObjects.at(i).getColor());
		
		//Put Depth in the display
		//cv::putText(frame,intToString((theObjects.at(i).getXPos()-320)*-1)+ " , " + intToString((theObjects.at(i).getYPos()-240)*-1)+ " , " + intToString(depthObject),cv::Point(theObjects.at(i).getXPos(),theObjects.at(i).getYPos()+20),1,1,theObjects.at(i).getColor());
		

		
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
	findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE);
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
					//Je met mon objet trouve dans la liste des objets de la scene
					SceneObjects.push_back(object);

					objectFound = true;

				}
				else objectFound = false;
			}
			//let user know you found an object
			if(objectFound ==true){
				//draw object location on screen
				//ROS_INFO("%d",numObjects);
				drawObject(objects,cameraFeed,temp,contours,hierarchy);
				
				

				
			}

		}
		else putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",Point(0,50),1,2,Scalar(0,0,255),2);
	}
}
void CreatePublisherObject()
{
	double depthObject;
	double depth0;
	double VecteurNew;
	int focalLengthY = 500; //515
	int focalLengthZ = 500;//220 - 400;

	for(int i = 0; i<SceneObjects.size(); i++)
	{
		depthObject = cv_Depth->image.at<short int>(cv::Point(SceneObjects.at(i).getXPos(),SceneObjects.at(i).getYPos()));
		
		depth0 = cv_Depth->image.at<short int>(cv::Point(320,240));
		depth0 = (double)depth0/1000;
		
		//Resolution Kinect 640 x 480
		double VecteurX = (double)depthObject/1000;
		double VecteurY = ((double)SceneObjects.at(i).getXPos()-320)/-1000;
		double VecteurZ = ((double)SceneObjects.at(i).getYPos()-240)/-1000;


	    cout << "VecteurDebutX " << VecteurX << endl;
		cout << "VecteurDebutY " << VecteurY << endl;
		cout << "VecteurDebutZ " << VecteurZ << endl;

	 

		/*double VecteurX = (double)depthObject/1000;
		double VecteurY = -(((double)SceneObjects.at(i).getXPos()-320)*0.26458);
		double VecteurZ = -(((double)SceneObjects.at(i).getYPos()-240)*0.26458);*/

		/*cout << "VecteurDebutX " << VecteurX << endl;
		cout << "VecteurDebutY " << VecteurY << endl;
		cout << "VecteurDebutZ " << VecteurZ << endl;
		
		//tf::TransformListener listener;

		//geometry_msgs::PointStamped kinect_point;
		//kinect_point.header.frame_id = "/head_mount_link";

		

  		//ROS_INFO("Publish");

		/*double YbaseLink = (VecteurY*depthObject)/(double)focalLengthY;
		

		double ZbaseLink = (VecteurZ*depthObject)/(double)focalLengthZ;
		if (VecteurZ != 0)
		{  VecteurNew = (VecteurZ/abs(VecteurZ)) * sqrt(pow(ZbaseLink, 2) + pow(VecteurX-depth0,2)); }
		else
		{  VecteurNew = sqrt(pow(ZbaseLink, 2) + pow(VecteurX-depth0,2)); }
  		//VecteurNew = VecteurNew + 0.2;

		cout << "VecteurX " << VecteurX << endl;
		cout << "depth0 " << depth0 << endl;

  		cout << "Difference Z " << ZbaseLink << endl;
		cout << "Difference X " << VecteurX-depth0 << endl;
		cout << "== VecteurX " << VecteurX << endl;
		cout << "== VecteurY " << YbaseLink << endl;
  		cout << "== Vecteurz " << VecteurNew << endl;*/
  		


		
		/*cout << "VecteurApresY " << YbaseLink << endl;
		cout << "VecteurApresZ " << ZbaseLink << endl;*/

		/*cout << "== VecteurApresY " << YbaseLink << endl;
		cout << "== VecteurApresZ " << ZbaseLink << endl;


		//laser_point.header.stamp = ros::Time();
		kinect_point.point.x = VecteurX;
  		kinect_point.point.y =  VecteurY;
  		kinect_point.point.z = VecteurZ;

		geometry_msgs::PointStamped base_point;
	try{
	    listener.waitForTransform("/base_footprint","/head_mount_link",ros::Time(0), ros::Duration(2.0));
	    listener.transformPoint("/base_footprint", kinect_point, base_point);

	   }
	  catch(tf::TransformException& ex){
	    ROS_ERROR("Received an exception trying to transform a point from \"head_mount_kinect\" to \"base_link\": %s", ex.what());
	  }*/

	    tf::Vector3 pointKinect(VecteurX,VecteurY,VecteurZ);
		tf::TransformListener listener;
		tf::StampedTransform transform;
		try{
			listener.waitForTransform("/base_footprint","/head_mount_kinect_rgb_link",ros::Time(0), ros::Duration(2.0));
			listener.lookupTransform( "/base_footprint","/head_mount_kinect_rgb_link",ros::Time(0), transform);
		}
		 catch (tf::TransformException ex){
		 	ROS_ERROR("%s",ex.what());
		 	
		}
	    tf::Vector3 pointOdom = transform * pointKinect;
	    /*double X = pointOdom.getX()+0.14;
	    double Y = pointOdom.getY();
	    double Z = pointOdom.getZ()+0.14;*/
	    /*double TranslationMatrice[3][3];
	    TranslationMatrice[0][0] = 0.2;
	    TranslationMatrice[1][1] = 0;
	    TranslationMatrice[2][2] = -0.2;
	    TranslationMatrice[3][3] = 1;*/

	    
		/*ROS_INFO("Vecteur X Apres : [%lf]", transform.getOrigin().getX());
		ROS_INFO("Vecteur Y Apres : [%lf]", transform.getOrigin().getY());
		ROS_INFO("Vecteur Z Apres :  [%lf]",transform.getOrigin().getZ());*/

		/*double left_elbow_1_x = transform.getOrigin().x();  
    	double left_elbow_1_y = transform.getOrigin().y();
    	double left_elbow_1_z = transform.getOrigin().z();
		double right_elbow_1_x = transform.getRotation().x(); 
	    double right_elbow_1_y = transform.getRotation().y(); 
	    double right_elbow_1_z = transform.getRotation().z();
	    double right_elbow_1_w = transform.getRotation().w();
	    /*cout <<"right_elbow_1_x " <<right_elbow_1_x<<endl;
    	cout <<"right_elbow_1_y " <<right_elbow_1_y<<endl;
    	cout <<"right_elbow_1_z " <<right_elbow_1_z<<endl;
    	cout <<"right_elbow_1_w " <<right_elbow_1_w<<endl;*/
		//cout << "VecteurDebutZ " << VecteurZ << endl;

		//Rx = Sx * ( x + xo ) 
		//double MonX = VecteurX * 0.730696327 - (0.208751078 * pointOdom.getX());
		//double MonY = VecteurX * 0 + (2.698958031 * pointOdom.getY());
		//cout << "Rotation " << transform.getRotation() << endl;

		//ROS_INFO("PointOdomY:[%lf]",transform.getRotation());

	  int width = 0;

		
		int XMinWidth;
		int YMinWidth = SceneObjects.at(i).getYPos();
		int XMaxWidth;
		int YMaxWidth = SceneObjects.at(i).getYPos();

		int Xedge = SceneObjects.at(i).getXPos();

		int DepthMinWidth = 0;
		int DepthMaxWidth = 0;

		int tempWidth = 0;
		int height = 0;
		int tempHeight = 0;

		
			/*for(int k= 0; k < contours.size(); k++) // 1
			{
			   
			    for(int l= 0;l < contours[k].size();l++) // 26 // run until j < contours[i].size();
			    {

				    for(int m= l;m < contours[k].size();m++) // Iterate into : comparate 1 to 2, 1 to 3, 1 to 4
				    {
					        	
				    	// Next Point minus previous point
				    	tempWidth = sqrt((contours[k][m].x-contours[k][l].x)*(contours[k][m].x-contours[k][l].x));
				    	tempHeight = sqrt((contours[k][m].y-contours[k][l].y)*(contours[k][m].y-contours[k][l].y));
				    						   
					   if (tempWidth>width)
					   {
					   		width = tempWidth;
					   		//Get the position of min X and max X 
					   		XMinWidth = contours[k][l].x+7;
					   		
					   		XMaxWidth = contours[k][m].x-10;
					   		

					   }
					   if (tempHeight>height){height = tempHeight;}

					}

			        
			    }
			}*/



		

		// OLD PUBLISHER
		mypkg::Object3D MessageObjet;
		ros::NodeHandle j;

		//PUBLISHER FOR PYRIDE
		pyride_pr2::NodeStatus ObjectPyride;
		PyrideObjects = j.advertise<pyride_pr2::NodeStatus>("/pyride_status", 10);

		ObjectPyride.node_id = "Essai";
		ObjectPyride.for_console = false;
		ObjectPyride.priority = 1;
		std::stringstream ss;

        ss << pointOdom.getX() << "/" << pointOdom.getY() << "/" << pointOdom.getZ() << "/" << SceneObjects.at(i).getType() << "/" << height << "/" << height << "/" << i;
        ObjectPyride.status_text = ss.str();

		PyrideObjects.publish(ObjectPyride);
		//ROS_INFO("%s", ObjectPyride.status_text.c_str());

		//ROS_INFO("%d",SceneObjects.size());
	}

}
void callback(const sensor_msgs::ImageConstPtr& original_image)
{
	
	Mat MyCameraFeed;
	Mat HSV;
	Mat threshold;
	cv_bridge::CvImagePtr cv_ptr;
	bool calibrationMode = true;

	if (cv_Depth == NULL)
	{
		ROS_INFO("No Subscribe Depth");
		ros::Duration(2).sleep();
		return;
	}

	

	cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
	MyCameraFeed = cv_ptr->image;

	src = MyCameraFeed;

	if(calibrationMode){
		//create slider bars for HSV filtering
		createTrackbars();
	}

	if(calibrationMode){

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
			trackFilteredObject(blue,threshold,HSV,MyCameraFeed);
		}
		else{
			//create some temp fruit objects so that
			//we can use their member functions/information
			Object blue("blue"), yellow("yellow"), red("red"), green("green");
			
			//first find blue objects
			cvtColor(MyCameraFeed,HSV,COLOR_BGR2HSV);
			inRange(HSV,blue.getHSVmin(),blue.getHSVmax(),threshold);
			morphOps(threshold);
			//ROS_INFO("Begin track object");
			trackFilteredObject(blue,threshold,HSV,MyCameraFeed);
			//then yellows
			cvtColor(MyCameraFeed,HSV,COLOR_BGR2HSV);
			inRange(HSV,yellow.getHSVmin(),yellow.getHSVmax(),threshold);
			morphOps(threshold);
			trackFilteredObject(red,threshold,HSV,MyCameraFeed);
			//then reds
			/*cvtColor(MyCameraFeed,HSV,COLOR_BGR2HSV);
			inRange(HSV,red.getHSVmin(),red.getHSVmax(),threshold);
			morphOps(threshold);
			trackFilteredObject(red,threshold,HSV,MyCameraFeed);
			//then greens
			cvtColor(MyCameraFeed,HSV,COLOR_BGR2HSV);
			inRange(HSV,green.getHSVmin(),green.getHSVmax(),threshold);
			morphOps(threshold);
			trackFilteredObject(green,threshold,HSV,MyCameraFeed);*/

			//PUBLIE TOUT EN MEME TEMPS
			CreatePublisherObject();
			//Supprime les elements de mon tableau
			SceneObjects.clear();

		}
		//ROS_INFO("Show Frame");
		//show frames 
		//imshow(windowName2,threshold);

	   // imshow(windowName,MyCameraFeed);
		//cv::waitKey(3);
		//imshow(windowName1,HSV);
		

}
int main(int argc, char* argv[])
{
	//if we would like to calibrate our filter values, set to true.


	ros::init(argc, argv, "Essai");
	ros::NodeHandle n;

	image_transport::ImageTransport depth(n);
	//image_transport::Subscriber SubDepth = depth.subscribe("/camera/depth_registered/image_raw",1,DepthCallback);
	image_transport::Subscriber SubDepth = depth.subscribe("/head_mount_kinect/depth/image_raw",1,DepthCallback);

	
	//if (cv_ptr != NULL)
	//{
	//ROS_INFO("YOUHOU");
	//cv::waitKey(0);
	image_transport::ImageTransport it(n);
	//image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_rect_color",1,callback);

	image_transport::Subscriber sub = it.subscribe("/head_mount_kinect/rgb/image_raw",1,callback);
	//ROS_INFO("Subscribe Image");

	//}
	
	
    ros::spin();
	

	return 0;
}
