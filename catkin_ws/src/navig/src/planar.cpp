#include <ros/ros.h>
#include <vector>
#include <math.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <boost/foreach.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/kdtree/kdtree.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <fstream> 
#include <string.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

boost::shared_ptr<pcl::visualization::PCLVisualizer>
simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::PointXYZ>::ConstPtr plane)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb1(cloud, 0, 0, 255);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb2(plane, 255, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, rgb1, "sample cloud1");
  viewer->addPointCloud<pcl::PointXYZ> (plane, rgb2, "sample cloud2");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud2");
  viewer->initCameraParameters ();
  return (viewer);
}

void callback(const PointCloud::ConstPtr& msg)
{
  ROS_INFO("I heard: [%d] [%d]",msg->width, msg->height);
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered (new pcl::PointCloud<pcl::PointXYZ>());
  vg.setInputCloud(msg);
  vg.setLeafSize(0.03f, 0.03f, 0.03f);
  vg.filter(*filtered);
  std::cout<<"filtering done"<<endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<int> inl;
  pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (filtered));
  pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
  ransac.setDistanceThreshold(0.05);
  ransac.computeModel();
  ransac.getInliers(inl);
  std::cout<<"ransac done"<<endl;
  pcl::copyPointCloud<pcl::PointXYZ>(*filtered, inl, *cloud);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ()), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  //seg.setMaxIterations (100);
  seg.setDistanceThreshold(0.01);
  int i = 0, nr_points = (int)filtered->points.size();
  std::cout<<"sac done"<<endl;
  while(filtered->points.size() > 0.6*nr_points)
  {
    seg.setInputCloud(filtered);
    seg.segment(*inliers, *coefficients);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(filtered);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter (*cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << nr_points << std::endl;
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *filtered = *cloud_f;    
  }
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (filtered);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.05); // 2cm
  ec.setMinClusterSize (50);
  ec.setMaxClusterSize (10000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (filtered);
  ec.extract (cluster_indices);  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
  int j = 0;
  std::vector<std::vector<float> > moi, xcor;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    j++;
    pcl::PointCloud<pcl::PointXYZ>::Ptr clusters (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    {
      clusters->points.push_back (filtered->points[*pit]); //*      
      cloud_cluster->points.push_back (filtered->points[*pit]); //*
      //cloud_cluster->width = 0;
      //cloud_cluster->height = 0;
    }
    pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud (clusters);
    feature_extractor.compute ();
    std::vector <float> moment_of_inertia;
    std::vector <float> eccentricity;
    pcl::PointXYZ min_point_AABB;
    pcl::PointXYZ max_point_AABB;
    pcl::PointXYZ min_point_OBB;
    pcl::PointXYZ max_point_OBB;
    pcl::PointXYZ position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    float major_value, middle_value, minor_value;
    Eigen::Vector3f major_vector, middle_vector, minor_vector;
    Eigen::Vector3f mass_center;
    feature_extractor.getMomentOfInertia (moment_of_inertia);
    feature_extractor.getEccentricity (eccentricity);
    feature_extractor.getAABB (min_point_AABB, max_point_AABB);
    feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    feature_extractor.getEigenValues (major_value, middle_value, minor_value);
    feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
    feature_extractor.getMassCenter (mass_center);  
    //for(int i=j-1; i<=j; j++)
    //{
    std::vector<float> row1;
    for(int k=0;k<2;k++)
      row1.push_back(mass_center[k]);
    moi.push_back(row1);
    //}
     //for(int i=j-1; i<=j; j++)
    //{
    std::vector<float> row2;
    row2.push_back(min_point_AABB.x);
    row2.push_back(max_point_AABB.x);
    row2.push_back(min_point_AABB.z);
    row2.push_back(max_point_AABB.z);
    row2.push_back(min_point_AABB.y);
    row2.push_back(max_point_AABB.y);
    xcor.push_back(row2);
    //}
  }
  std::cout<<endl<<j<<endl;
  //for(int i=0;i<j;i++)
  //std::cout<<moi[i][0]<<" "<<moi[i][1]<<endl;
  ofstream outfile;
  outfile.open("kinect.txt");
  for(int i=0; i<j; i++)
      {
        if(xcor[i][2]<2.0 && abs(xcor[i][5]-xcor[i][4]>0.15))
        {
          //std::cout<<moi[i][1]<<" dist = "<<xcor[i][2]<<endl;
          if(moi[i][0]>=0)
	  {
            std::cout<<"left\n";
	    outfile <<"left";
	    goto label;
	  }
          else //if(fabs(xcor[i][0])>fabs(xcor[i][1]))
	  {
            std::cout<<"right\n";
    	    outfile <<"right";
	    goto label;
	  }
        }
	//else
       }
	std::cout<<"straight\n";
      label:
	std::cout<<"";
	outfile.close();
  viewer = simpleVis(cloud_cluster, cloud);  
  while(!viewer->wasStopped())
  {
     viewer->spinOnce(100);
     boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "planar");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/camera/depth/color/points", 1, callback);
  ros::spin();
  return 0;
} 
