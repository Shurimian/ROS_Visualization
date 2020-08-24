#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

//this node publishes transform data. Converts from laser_frame -> base_frame
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(30);

  tf::TransformBroadcaster broadcaster;

  while(n.ok())
  {
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.5, 0.0, 0.3)),
        ros::Time::now(),"base_link", "laser_frame"));

    r.sleep();
  }
}