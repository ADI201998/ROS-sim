# ROS-sim
Work done by me on ROS and simulation on Gazebo.

The catkin workspace (catkin-ws) has two packages
- my_bot
- navig

The package my_bot contains a urdf file which was developed from the [urdf tutorials](http://wiki.ros.org/urdf/Tutorials) 
which can be imported on Gazebo.

The navig package contains various subscribers and publishers which were tries and tested on simulation on Gazebo using the urdf developed in the my_bot package.
Thses codes include obstacle avoidance using [Point Cloud Library](http://www.pointclouds.org/documentation/) from the point cloud data obtained from the kinect sensor mounted on the bot
and obstacle avoidance using laserscan from the lidar mounted on the bot.
