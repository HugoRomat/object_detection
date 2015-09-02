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


//Store all constants for image encodings in the enc namespace to be used later.

namespace enc = sensor_msgs::image_encodings;
static const char WINDOW[] = "Image Processed";
image_transport::Publisher pub;
 /**
  * This tutorial demonstrates simple receipt of messages over the ROS system.
  */
void callback(const sensor_msgs::ImageConstPtr& original_image)
{
	//Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
  ROS_INFO("Suscriber Launch");

  cv_bridge::CvImagePtr cv_ptr;
  try
    {
        //Always copy, returning a mutable CvImage
        //OpenCV expects color images to use BGR channel order.
        cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
    }
  catch (cv_bridge::Exception& e)
    {
        //if there is an error during conversion, display it
        ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }
 
    //Invert Image
    //Go through all the rows
    for(int i=0; i<cv_ptr->image.rows; i++)
    {
        //Go through all the columns
        for(int j=0; j<cv_ptr->image.cols; j++)
        {
            //Go through all the channels (b, g, r)
            for(int k=0; k<cv_ptr->image.channels(); k++)
            {
                //Invert the image by subtracting image data from 255               
                cv_ptr->image.data[i*cv_ptr->image.rows*4+j*3 + k] = 255-cv_ptr->image.data[i*cv_ptr->image.rows*4+j*3 + k];
            }
        }
    }
  cv::imshow(WINDOW, cv_ptr->image);
  cv::waitKey(3);
  pub.publish(cv_ptr->toImageMsg());
  ROS_INFO("I heard");
}
int main( int argc, char** argv )
{
    /*ros::init(argc, argv, "Essai");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/camera/rgb/image_color",1,callback);
    //ros::Subscriber sub = n.subscribe("chatter", 1000, callback);*/

    ros::init(argc, argv, "Essai");
	ros::NodeHandle n;
	image_transport::ImageTransport it(n);
	ROS_INFO("Suscribe");
	image_transport::Subscriber sub = it.subscribe("camera/rgb/image_color",1000,callback);
    cv::destroyWindow(WINDOW);
    pub = it.advertise("camera/image_processed", 1);
	ros::spin();

    return 0;
    ROS_INFO("WELL");

    /*if( argc != 2)
    {
     ROS_INFO(" Usage: display_image ImageToLoadAndDisplay");
     return -1;
    }

    cv::Mat image;
    image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);   // Read the file

    if(! image.data )                              // Check for invalid input
    {
        ROS_INFO("Could not open or find the image");
        return -1;
    }
    
    cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
    cv::imshow( "Display window", image );                   // Show our image inside it.

    cv::waitKey(0);   */                                       // Wait for a keystroke in the window
    return 0;
}