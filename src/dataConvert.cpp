#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <Eigen/Core>
#include <iostream>
#include <cstring>
#include <vector>

using namespace std;

//create two objects; first to allow for a laserscan -> pointcloud transform, second to describe the transform
laser_geometry::LaserProjection projector;
tf::TransformListener* listener = NULL;
sensor_msgs::PointCloud temp_data;

//create temporary variables to hold the converted pointcloud data. 
//done because NodeHandle has scoping issues with message variables being created before initialization.
//also, cannot create a callback function with multiple parameters because they may only have one message type at a time

int array_size = 0;
/*
vector<double> temp_points_x; //arbitrary array size to hold however many points may end up being required
vector<double> temp_points_y;
vector<double> temp_points_z;
*/

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  if(!listener->waitForTransform(msg->header.frame_id, "base_link", msg->header.stamp + ros::Duration().fromSec(msg->ranges.size()*msg->time_increment), ros::Duration(1.0)))
  {
  } 
  
  //create temporary pointcloud message to hold transformed data
  sensor_msgs::PointCloud cloud;

  projector.transformLaserScanToPointCloud("base_link", *msg, cloud, *listener);

  //create a variable representing size of array to resize vectors later
  array_size = sizeof(cloud.points)/sizeof(cloud.points[0]);

  //start storing cloud values in temporary values
  temp_data.header.frame_id = cloud.header.frame_id;
  temp_data.header.stamp = cloud.header.stamp;

  //resize array to match size of cloud array
  //temp_points_x.resize(array_size);
  //temp_points_y.resize(array_size);
  //temp_points_z.resize(array_size);
  temp_data.points.resize(array_size);

  //transfer all the values in array
  for (int i{}; i < array_size; ++i)
  {
    temp_data.points[i] = cloud.points[i];
  }
  
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  //add odometry data to the temporary array
  for (int i{}; i < array_size; ++i)
  {
    temp_data.points[i].x += msg->pose.pose.position.x;
    temp_data.points[i].y += msg->pose.pose.position.y;
    temp_data.points[i].z += msg->pose.pose.position.z;
  }
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "convert_data");
  ros::NodeHandle n;

  listener = new(tf::TransformListener);

  ros::Subscriber laser_sub = n.subscribe("scan", 50, laserCallback);
  ros::spinOnce();
  ros::Subscriber odom_sub = n.subscribe("odom", 50, odomCallback);
  ros::spinOnce();

  ros::Publisher data_pub = n.advertise<sensor_msgs::PointCloud>("pointData", 50);

  //create a proper pointcloud message and repopulate with temp data
  sensor_msgs::PointCloud data;
  data.header.frame_id = temp_data.header.frame_id;
  data.header.stamp = temp_data.header.stamp;

  data.points.resize(array_size);

  for (int i{}; i < array_size; ++i)
  {
    data.points[i].x = temp_data.points[i].x;
    data.points[i].y = temp_data.points[i].y;
    data.points[i].z = temp_data.points[i].z;
  }

  data_pub.publish(data);
  //ros::spin();

}