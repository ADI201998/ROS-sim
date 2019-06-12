    #include <ros/ros.h>
    #include <pcl_ros/point_cloud.h>
    #include <pcl/point_types.h>
    #include <boost/foreach.hpp>
    
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    
    void callback(const PointCloud::ConstPtr& msg)
    {
     printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
     BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
     {
       if(pt.y>-0.2&&pt.x<0.5)
          std::cout<<"left\n";
       else
          std::cout<<"right\n";
     }
   }
   
   int main(int argc, char** argv)
   {
     ros::init(argc, argv, "sub_pcl");
     ros::NodeHandle nh;
     ros::Subscriber sub = nh.subscribe<PointCloud>("/voxel_cloud", 1, callback);
     ros::spin();
   }
