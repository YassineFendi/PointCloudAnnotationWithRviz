#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt64MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

// STL
#include <cstring>
#include <stdint.h>
#include <cmath>
#include <string>
#include <fstream>

// Eigen
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/publisher.h"
#include <pcl/search/search.h>
#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>

// boost
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/foreach.hpp>

// Markers
#include <interactive_markers/interactive_marker_server.h>


#include"point_types.h"


using namespace std;



class AnnotationClass {

public :
typedef pcl::PointXYZRGBNormal PointXYZRGBNormal;
typedef pcl::PointCloud<PointXYZRGBNormal> PointXYZRGBNormalCloud;
typedef pcl::Normal PointNormal;
typedef pcl::PointCloud<PointNormal> PointNormalCloud;
typedef pcl::PointXYZRGB PointXYZRGB;
typedef pcl::PointCloud<PointXYZRGB> PointXYZRGBCloud;
typedef pcl::PointXYZRGBL PointXYZRGBL;
typedef pcl::PointCloud<PointXYZRGBL> PointXYZRGBLCloud;
typedef sensor_msgs::PointCloud2 RosMsgCloud2;
typedef pcl::PointXYZRGBA PointXYZRGBA;
typedef pcl::PointCloud<PointXYZRGBA> PointXYZRGBACloud;
typedef pcl::PointXYZI PointXYZI;
typedef pcl::PointCloud<PointXYZI> PointXYZICloud;


typedef visualization_msgs::InteractiveMarker InteractiveMarker;
typedef visualization_msgs::Marker Marker;
typedef interactive_markers::InteractiveMarkerServer InteractiveMarkerServer;
typedef boost::shared_ptr<interactive_markers::InteractiveMarkerServer> InteractiveMarkerServerPtr;
typedef visualization_msgs::InteractiveMarkerFeedback InteractiveMarkerFeedback;
typedef visualization_msgs::InteractiveMarkerFeedbackPtr InteractiveMarkerFeedbackPtr;
typedef visualization_msgs::InteractiveMarkerFeedbackConstPtr InteractiveMarkerFeedbackConstPtr;
typedef uint64_t uint64;

typedef velodyne_pointcloud::Point8D Velodyne_Data;
typedef pcl::PointCloud<Velodyne_Data> VelodyneCloud;






AnnotationClass(ros::NodeHandle & nh);

RosMsgCloud2 LoadPcd(const std::string & filename);

VelodyneCloud AffectId(Velodyne_Data cloud);

InteractiveMarker TransformToMarker(const VelodyneCloud &cloud);

InteractiveMarker SelectCloudObject(const VelodyneCloud &cloud);

void SendCloudSelection();

void GetSelection(const InteractiveMarkerFeedbackConstPtr  &feedback);

void SendCloudMarker(const bool apply);

void SaveMe(VelodyneCloud cloud2);

PointXYZRGBCloud ColorCopy();

private :


Velodyne_Data cloud;
VelodyneCloud cloudm;
VelodyneCloud cloudout;
RosMsgCloud2 output;
InteractiveMarkerServer server;
PointXYZRGBNormalCloud::Ptr cloudym;
int IndexName = 0;

PointXYZICloud PointIntensity;
float m_point_size_multiplier = 10;
float m_point_size =0.005;

ros::NodeHandle & m_nh;

};

