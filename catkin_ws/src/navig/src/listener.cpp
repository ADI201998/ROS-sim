#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

void chatterCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
 ROS_INFO("I heard: [%d] [%d] [%d] [%d]", msg->height, msg->width,msg->point_step,msg->row_step);
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("camera/depth/points", 1, chatterCallback);
  ros::spin();
  return 0;
}
