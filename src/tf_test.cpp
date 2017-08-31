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
#include <tf/transform_datatypes.h>
#include <boost/bind.hpp>
#include <velocityobs/MyRoots.h>

using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace nav_msgs;
using namespace message_filters;
using namespace cv;
using namespace ros;
using namespace std;

float my_p[2], trans[4];
bool init_trans = false;
float prev;
double run_t1, run_t2;
double roll, pitch, yaw;


void initialCallback(ros::NodeHandle &node_handle, const OdometryConstPtr& p)
{
  // my_p[0] = p->pose.pose.position.x;
  // my_p[1] = p->pose.pose.position.y;
  // my_p[2] = p->pose.pose.position.z;
  if (!init_trans)
  {
  	run_t1 = p->header.stamp.toSec();
    tf::Quaternion q(p->pose.pose.orientation.x, p->pose.pose.orientation.y, p->pose.pose.orientation.z, p->pose.pose.orientation.w); // xyzw
    tf::Matrix3x3 m(q);  
    m.getRPY(roll, pitch, yaw);
    std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;
    std::cout << "Roll: " << roll * (180/M_PI)<< ", Pitch: " << pitch * (180/M_PI)<< ", Yaw: " << yaw * (180/M_PI)<< std::endl;
    node_handle.setParam("yaw_param", yaw);
    cout<<"Yaw pushed"<<"\n";
    // trans[0] = cos(yaw);
    // trans[1] = sin(yaw);
    // trans[2] = -sin(yaw);
    // trans[3] = cos(yaw);
    //init_trans = true;
  }
  //cout<<"Transformed X: "<<trans[0]*my_p[0] + trans[1]*my_p[1]<<"\tY: "<<trans[2]*my_p[0] + trans[3]*my_p[1]<<"\n";
  /*if (trans[0]*my_p[0] + trans[1]*my_p[1] == prev)
  {
  	run_t2 = p->header.stamp.toSec();
  	cout<<"run_t1: "<<run_t1<<"\trun_t2: "<<run_t2<<"\n";
    cout<<"Average Velocity for the trip: "<<prev/(run_t2 - run_t1)<<"\n";
  }
  prev = trans[0]*my_p[0] + trans[1]*my_p[1];*/

}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_ka_test");
  ros::NodeHandle nh;
  //ros::Subscriber my_pose = nh.subscribe("/bebop/odom", 1, initialCallback);
  ros::Subscriber my_pose = nh.subscribe<Odometry>("/bebop/odom", 1, boost::bind(&initialCallback, boost::ref(nh), _1));
  ros::spin();
  run_t2 = ros::Time::now().toSec();
  return 0;
}