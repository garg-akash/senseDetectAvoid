#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <math.h>
#include <time.h>

#include <sensor_msgs/Image.h>
#include <image_geometry/stereo_camera_model.h>

#include <std_msgs/String.h>
#include <stdlib.h>
#include <string>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <velocityobs/MyRoots.h>

using namespace sensor_msgs;
using namespace nav_msgs;
using namespace geometry_msgs;
using namespace ros;
using namespace std;

ros::Publisher vel_pub;
geometry_msgs::Twist twist;

float my_p[3];  
double vel = 0.1;

void initialCallback(const OdometryConstPtr& p)
{
  my_p[0] = p->pose.pose.position.x;
  my_p[1] = p->pose.pose.position.y;
  my_p[2] = p->pose.pose.position.z;
  if (my_p[0] < 6)
  {
    twist.linear.x = vel;
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.z = 0;
    vel_pub.publish(twist);
  }
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "move");
  ros::NodeHandle nh;
  ros::Subscriber my_pose; 
  vel_pub = nh.advertise<geometry_msgs::Twist>("/bebop/cmd_vel", 1);
  my_pose = nh.subscribe("/bebop/odom", 1, initialCallback);
  ros::spin();
  return 0;
}