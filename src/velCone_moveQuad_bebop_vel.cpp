#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <math.h>
#include <time.h>

#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <image_geometry/stereo_camera_model.h>

#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <stdlib.h>
#include <string>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <velocityobs/MyRoots.h>

using namespace sensor_msgs;
using namespace nav_msgs;
using namespace geometry_msgs;
using namespace message_filters;
using namespace cv;
using namespace ros;
using namespace std;

ros::Publisher vel_pub;
geometry_msgs::TwistStamped ts;

float my_p[3];  
float my_p_prev[3] = {0};      /*Quad Pose*/
double t1;
double t2 = 0;    
int cntr = 0;
double vel_avg = 0;
double vel;

void initialCallback(const OdometryConstPtr& p)
{
  my_p[0] = p->pose.pose.position.x;
  my_p[1] = p->pose.pose.position.y;
  my_p[2] = p->pose.pose.position.z;
  t1 = p->header.stamp.toSec();
  vel = ((my_p[0] - my_p_prev[0])/(t1-t2));
  if (cntr > 5)
  {
    ts.header.stamp = p->header.stamp;
    vel_avg = vel_avg/5;
    ts.twist.linear.x = vel_avg;
    ts.twist.linear.y = 0;
    ts.twist.linear.z = 0;
    ts.twist.angular.z = 0;
    vel_pub.publish(ts);
    cntr = 0;
    vel_avg = 0;
  }
  if (vel> 0.4)
  {
    vel = 0.4;
  }
  vel_avg = vel_avg + vel;
  cntr++;
  my_p_prev[0] = my_p[0];
  my_p_prev[1] = my_p[1];
  my_p_prev[2] = my_p[2];
  t2 = t1;
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "avgvel");
  ros::NodeHandle nh;
  ros::Subscriber my_pose; 
  vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/self/velX", 1);
  my_pose = nh.subscribe("/bebop/odom", 1, initialCallback);
  ros::spin();
  return 0;
}