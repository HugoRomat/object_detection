#include <iostream>
#include "ros/ros.h"
#include <boost/thread/thread.hpp>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/registration/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <pcl/common/centroid.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
/*
pcl::PointCloud<pcl::PointXYZ> cloud;
pcl::PointCloud<pcl::PointXYZ>::ConstPtr MyCloud;
typedef pcl::PointXYZ PointT;

void callbackCloud(const sensor_msgs::PointCloud2& input){

	pcl::PCLPointCloud2 pcl_pc;
	pcl_conversions::toPCL(input, pcl_pc);
    pcl::fromPCLPointCloud2(pcl_pc, cloud);


	
}
int main(int argc, char* argv[])
{
	//if we would like to calibrate our filter values, set to true.

	ros::init(argc, argv, "CreateBound");
	ros::NodeHandle n;

	image_transport::ImageTransport it(n);
	ros::Subscriber subCloud = n.subscribe("/head_mount_kinect/depth_registered/points",100,callbackCloud);
	

    ros::spin();
	

	return 0;
}*/
/*#include <iostream>
#include <cstdlib>

// ROS
#include <ros/ros.h>

// PCL
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

boost::shared_ptr<pcl::visualization::PCLVisualizer> visor;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
sensor_msgs::PointCloud2 cloud;
typedef pcl::PointXYZ PointT;

boost::shared_ptr<pcl::visualization::PCLVisualizer> createVis ()
{
  // --------------------------------------------
  // -----Open 3D viewer and add properties-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return viewer;
}

void updateVis(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud){
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->removePointCloud();
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb);
  viewer->spinOnce();
  
  /*viewer->addCube (0, 2, 0, 2, 0, 2); 
  viewer->addCube (4, 6, 4, 6, 4, 6); 
  pcl::PointXYZ min_point_AABB;
  pcl::PointXYZ max_point_AABB;
  pcl::PointXYZ min_point_OBB;
  pcl::PointXYZ max_point_OBB;
  pcl::PointXYZ position_OBB;
  Eigen::Matrix3f rotational_matrix_OBB;
  float major_value, middle_value, minor_value;
  Eigen::Vector3f major_vector, middle_vector, minor_vector;
  Eigen::Vector3f mass_center;

  viewer->addCube (min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, "AABB");

  Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
  Eigen::Quaternionf quat (rotational_matrix_OBB);
  viewer->addCube (position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");

  pcl::PointXYZ center (mass_center (0), mass_center (1), mass_center (2));
  pcl::PointXYZ x_axis (major_vector (0) + mass_center (0), major_vector (1) + mass_center (1), major_vector (2) + mass_center (2));
  pcl::PointXYZ y_axis (middle_vector (0) + mass_center (0), middle_vector (1) + mass_center (1), middle_vector (2) + mass_center (2));
  pcl::PointXYZ z_axis (minor_vector (0) + mass_center (0), minor_vector (1) + mass_center (1), minor_vector (2) + mass_center (2));
  viewer->addLine (center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
  viewer->addLine (center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
  viewer->addLine (center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");*/




/*}

void showPCL(sensor_msgs::PointCloud2 points){
  cloud = points;
  pcl::fromROSMsg<pcl::PointXYZRGB>(cloud, *(ptCloud.get()));
  updateVis(visor, ptCloud);
}


int main(int argc, char **argv) {
  using namespace ros;

  visor = createVis();

  init(argc,argv,"kinQual");
  NodeHandle nh;
  Subscriber subDepth = nh.subscribe<>("camera/depth_registered/points",10,showPCL);
 
  while(!visor->wasStopped()){
   //sleep(1);
   spin();

  }

  return 0;
}*/
#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/console/parse.h>

typedef pcl::PointXYZ PointType;

// --------------------
// -----Parameters-----
// --------------------
float angular_resolution = 0.5f;
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
bool setUnseenToMaxRange = false;

boost::shared_ptr<pcl::visualization::PCLVisualizer> visor;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
sensor_msgs::PointCloud2 cloud;


boost::shared_ptr<pcl::visualization::PCLVisualizer> createVis()
{
  // --------------------------------------------
  // -----Open 3D viewer and add properties-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return viewer;
}
void printUsage (const char* progName)
{
  std::cout << "\n\nUsage: "<<progName<<" [options] <scene.pcd>\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-r <float>   angular resolution in degrees (default "<<angular_resolution<<")\n"
            << "-c <int>     coordinate frame (default "<< (int)coordinate_frame<<")\n"
            << "-m           Treat all unseen points to max range\n"
            << "-h           this help\n"
            << "\n\n";
}
void updateVis(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud){
 
 
  pcl::PointCloud<PointType>::Ptr point_cloud_ptr (new pcl::PointCloud<PointType>);
  pcl::copyPointCloud<pcl::PointXYZRGB,PointType>(*cloud, *point_cloud_ptr);
 
  
  pcl::PointCloud<PointType>& point_cloud = *point_cloud_ptr;

  pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
  Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());


  float noise_level = 0.0;
  float min_range = 0.0f;
  int border_size = 1;
  boost::shared_ptr<pcl::RangeImage> range_image_ptr (new pcl::RangeImage);

  pcl::RangeImage& range_image = *range_image_ptr;
  range_image.createFromPointCloud (point_cloud, angular_resolution, pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
  range_image.integrateFarRanges (far_ranges);
  if (setUnseenToMaxRange)
    range_image.setUnseenToMaxRange ();


  viewer->setBackgroundColor (1, 1, 1);
  viewer->addCoordinateSystem (1.0f);
  
  pcl::visualization::PointCloudColorHandlerCustom<PointType> point_cloud_color_handler (point_cloud_ptr, 0, 0, 0);

   visor->removePointCloud ("original point cloud");
   visor->removePointCloud ("border points");
   visor->removePointCloud ("veil points");
   visor->removePointCloud ("shadow points");
   
   viewer->addPointCloud (point_cloud_ptr, point_cloud_color_handler, "original point cloud");
viewer->spinOnce();
  


  
  // -------------------------
  // -----Extract borders-----
  // -------------------------
  pcl::RangeImageBorderExtractor border_extractor (&range_image);
  pcl::PointCloud<pcl::BorderDescription> border_descriptions;
  border_extractor.compute (border_descriptions);
  
  // ----------------------------------
  // -----Show points in 3D viewer-----
  // ----------------------------------
  pcl::PointCloud<pcl::PointWithRange>::Ptr border_points_ptr(new pcl::PointCloud<pcl::PointWithRange>),
                                            veil_points_ptr(new pcl::PointCloud<pcl::PointWithRange>),
                                            shadow_points_ptr(new pcl::PointCloud<pcl::PointWithRange>);
  pcl::PointCloud<pcl::PointWithRange>& border_points = *border_points_ptr,
                                      & veil_points = * veil_points_ptr,
                                      & shadow_points = *shadow_points_ptr;
  for (int y=0; y< (int)range_image.height; ++y)
  {
    for (int x=0; x< (int)range_image.width; ++x)
    {
      if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__OBSTACLE_BORDER])
        border_points.points.push_back (range_image.points[y*range_image.width + x]);
      if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__VEIL_POINT])
        veil_points.points.push_back (range_image.points[y*range_image.width + x]);
      if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__SHADOW_BORDER])
        shadow_points.points.push_back (range_image.points[y*range_image.width + x]);
    }
  }
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> border_points_color_handler (border_points_ptr, 0, 255, 0);
  viewer->addPointCloud<pcl::PointWithRange> (border_points_ptr, border_points_color_handler, "border points");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "border points");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> veil_points_color_handler (veil_points_ptr, 255, 0, 0);
  viewer->addPointCloud<pcl::PointWithRange> (veil_points_ptr, veil_points_color_handler, "veil points");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "veil points");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> shadow_points_color_handler (shadow_points_ptr, 0, 255, 255);
  viewer->addPointCloud<pcl::PointWithRange> (shadow_points_ptr, shadow_points_color_handler, "shadow points");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "shadow points");
  
  //-------------------------------------
  // -----Show points on range image-----
  // ------------------------------------
  pcl::visualization::RangeImageVisualizer* range_image_borders_widget = NULL;
  range_image_borders_widget =
    pcl::visualization::RangeImageVisualizer::getRangeImageBordersWidget (range_image, -std::numeric_limits<float>::infinity (), std::numeric_limits<float>::infinity (), false,
                                                                          border_descriptions, "Range image with borders");
  // -------------------------------------
  
  
  //--------------------
  // -----Main loop-----
  //--------------------
}
void showPCL(sensor_msgs::PointCloud2 points){
  cloud = points;
  pcl::fromROSMsg<pcl::PointXYZRGB>(cloud, *(ptCloud.get()));
  updateVis(visor, ptCloud);
}

int main(int argc, char **argv) {
  using namespace ros;

  visor = createVis();

  init(argc,argv,"kinQual");
  NodeHandle nh;
  Subscriber subDepth = nh.subscribe<>("camera/depth_registered/points",10,showPCL);
 
  while(!visor->wasStopped()){
   //sleep(1);
   spin();

  }

  return 0;
}