#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

//this node publishes transform data. Converts from laser_frame -> base_frame

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_publisher");
  ros::NodeHandle n;

  //sets the rate at which tranform data will be published
  ros::Rate r(30);

  //create a broadcaster object
  tf::TransformBroadcaster broadcaster;

  //continuously publish the transform data as the node is running
  while(n.ok())
  {
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.5, 0.0, 0.3)),
        ros::Time::now(),"base_link", "laser_frame"));
    
    //this controls the frequency of data publication. Kept at 30Hz
    r.sleep();
  }
}