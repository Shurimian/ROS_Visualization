#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>

int main(int argc, char** argv)
{

  ros::init(argc, argv, "odom_data_pub");
  ros::NodeHandle n;

  //advertise to ROS_Master that this node is publishing to a topic called /odom
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

  //create basic "odom info" for fake data
  double x = 0;
  double y = 0;
  double Vx = 0;
  double Vy = 0;

  geometry_msgs::Quaternion quat;
  quat.x = 0;
  quat.y = 0;
  quat.z = 0;
  quat.w = 1;

  //controls frequency of data publication
  ros::Rate r(0.5);

  //run loop while node is active
  while(n.ok())
  {
    //populate the odometry message
    nav_msgs::Odometry odomData;
    odomData.header.stamp = ros::Time::now();
    odomData.header.frame_id = "base_link";
    odomData.pose.pose.position.x = x;
    odomData.pose.pose.position.y = y;
    odomData.pose.pose.position.z = 0;
    odomData.pose.pose.orientation = quat;
    odomData.child_frame_id = "base_link";
    odomData.twist.twist.linear.x = Vx;
    odomData.twist.twist.linear.y = Vy;
    odomData.twist.twist.angular.z = 0;

    //publish the msg
    odom_pub.publish(odomData);
    
    //gradually move the robot forwards
    ++x;

    //controls the frequency of the loop
    r.sleep();
  }
}