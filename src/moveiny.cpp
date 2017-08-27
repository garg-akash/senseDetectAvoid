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
#include <tf/transform_datatypes.h>
#include <velocityobs/MyRoots.h>

using namespace sensor_msgs;
using namespace nav_msgs;
using namespace geometry_msgs;
using namespace ros;
using namespace std;

ros::Publisher vel_pub;
geometry_msgs::Twist twist;

float my_p[3];  
float vel = 0.04;
float trans[4];
bool init_trans = false;
float prev;
double run_t1, run_t2;
int num=0;
void initialCallback(const OdometryConstPtr& p)
{
  my_p[0] = p->pose.pose.position.x;
  my_p[1] = p->pose.pose.position.y;
  my_p[2] = p->pose.pose.position.z;
  if (!init_trans)
  {
    run_t1 = p->header.stamp.toSec();
    tf::Quaternion q(p->pose.pose.orientation.x, p->pose.pose.orientation.y, p->pose.pose.orientation.z, p->pose.pose.orientation.w); // xyzw
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;
    std::cout << "Roll: " << roll * (180/M_PI)<< ", Pitch: " << pitch * (180/M_PI)<< ", Yaw: " << yaw * (180/M_PI)<< std::endl;
    trans[0] = cos(yaw);
    trans[1] = sin(yaw);
    trans[2] = -sin(yaw);
    trans[3] = cos(yaw);
    init_trans = true;
    cout<<"Called Once"<<"\n";
  }
  //my_p[0] = trans[0]*my_p[0] + trans[1]*my_p[1];
  //my_p[1] = trans[2]*my_p[0] + trans[3]*my_p[1];
  cout<<"Transformed X: "<<trans[0]*my_p[0] + trans[1]*my_p[1]<<"\tY: "<<trans[2]*my_p[0] + trans[3]*my_p[1]<<"\n";

  // cout<<"Reached"<<"\n";
  //   twist.linear.x = 0;
  //   twist.linear.y = vel;
  //   twist.linear.z = 0;
  //   twist.angular.z = 0;
  float count = 0;
  double del_t = 0;
  double t = 0;
  if (num%2==0)
  {
      while (count < 1)
      {
    //t = clock();
        t = ros::Time::now().toSec();
        twist.linear.x = 0;
        twist.linear.y = vel;
        twist.linear.z = 0; /*no velocity should be imparted in z direction*/
        vel_pub.publish(twist);
    //usleep(500);
    //t = clock() - t;
        del_t = ros::Time::now().toSec() - t;
    //del_t = ((float)t)/CLOCKS_PER_SEC;
        count = count + del_t;
        cout<<"count: "<<count<<"\n";
      }
  }
  else
  {
      while (count < 1)
      {
    //t = clock();
        t = ros::Time::now().toSec();
        twist.linear.x = 0;
        twist.linear.y = -vel;
        twist.linear.z = 0; /*no velocity should be imparted in z direction*/
        vel_pub.publish(twist);
    //usleep(500);
    //t = clock() - t;
        del_t = ros::Time::now().toSec() - t;
    //del_t = ((float)t)/CLOCKS_PER_SEC;
        count = count + del_t;
        cout<<"count: "<<count<<"\n";
      }
  }
  num = num+1;

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