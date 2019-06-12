#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <boost/foreach.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <math.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

boost::shared_ptr<pcl::visualization::PCLVisualizer>
simpleVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  //viewer->addCoordinateSystem (1.0, "global");
  viewer->initCameraParameters ();
  return (viewer);
}

void chatterCallback(const PointCloud::ConstPtr& msg)
{
 ROS_INFO("I heard: [%d] [%d]", msg->height, msg->width);
 boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
 std::vector<int> inliers;
 pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB> (msg));
 pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac(model_p);
 ransac.setDistanceThreshold(0.001);
 ransac.computeModel();
 ransac.getInliers(inliers);
 pcl::copyPointCloud<pcl::PointXYZRGB>(*msg, inliers, *cloud);
 viewer = simpleVis(cloud);
 while (!viewer->wasStopped ())
{
  viewer->spinOnce (100);
  boost::this_thread::sleep (boost::posix_time::microseconds (100000));
}
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "segment_plane");
  ros::NodeHandle n;  
  ros::Subscriber sub = n.subscribe("/voxel_cloud", 1, chatterCallback);
  ros::spin();
  return 0;
}
