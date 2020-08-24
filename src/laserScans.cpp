#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

int main(int argc, char** argv)
{

  ros::init(argc, argv, "laser_scan_publisher");
  ros::NodeHandle n;
  ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 50);

  //create basic "laser info" for fake data
  unsigned int num_readings = 100;
  double laser_frequency = 40;
  double ranges[num_readings];
  double intensities[num_readings];

  int dist = 0;
  ros::Rate r(0.2);

  //run loop while node is active
  while(n.ok())
  {

    //generate some fake data for our laser scan
    for(unsigned int i = 0; i < num_readings; ++i)
    {
      ranges[i] = dist;
      intensities[i] = 100 + dist;
    }
    //ros::Time scan_time = ros::Time::now();

    //populate the LaserScan message
    sensor_msgs::LaserScan scan;
    scan.header.stamp = ros::Time::now();
    scan.header.frame_id = "laser_frame";
    scan.angle_min = -1.57;
    scan.angle_max = 1.57;
    scan.angle_increment = 3.14 / num_readings;
    scan.time_increment = (1 / laser_frequency) / (num_readings);
    scan.range_min = 0.0;
    scan.range_max = 100.0;

    //resize array to be as big as # of readings
    scan.ranges.resize(num_readings);
    scan.intensities.resize(num_readings);

    //populate the range and intensity array of msg
    for(unsigned int i = 0; i < num_readings; ++i)
    {
      scan.ranges[i] = ranges[i];
      scan.intensities[i] = intensities[i];
    }

    //publish the msg
    scan_pub.publish(scan);
    
    ++dist;
    //controls the frequency of the loop
    r.sleep();
  }
}