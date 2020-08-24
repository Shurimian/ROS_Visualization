#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <Eigen/Core>
#include <iostream>

laser_geometry::LaserProjection projector;
tf::TransformListener listener(ros::Duration(1000));
//sensor_msgs::PointCloud cloud;
//sensor_msgs::PointCloud data;

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  if(!listener.waitForTransform(msg->header.frame_id, "base_link", msg->header.stamp + ros::Duration().fromSec(msg->ranges.size()*msg->time_increment), ros::Duration(1.0)))
  {
  } 

  projector.transformLaserScanToPointCloud("base_link", *msg, cloud, listener);

}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  for (int i = 0; i < sizeof(cloud.points); ++i)
  {
    cloud.points[i].x += msg->pose.pose.position.x;
    cloud.points[i].y += msg->pose.pose.position.y;
  }
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "convert_data");
  ros::NodeHandle n;

  ros::Subscriber laser_sub = n.subscribe("scan", 50, laserCallback);
  ros::Subscriber odom_sub = n.subscribe("odom", 50, odomCallback);

  ros::Publisher data_pub = n.advertise<sensor_msgs::PointCloud>("pointData", 50);

  data_pub.publish(data);
  ros::spin();

  return 0;
}