#include <ros/ros.h>
#include <velocityobs/MyRoots.h>
#include <iostream>
#include <stdio.h>

using namespace ros;
using namespace std;

int main(int argc, char** argv)
{
  float new_t;
  ros::init(argc, argv, "velObs");
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<velocityobs::MyRoots>("find_roots");
  velocityobs::MyRoots srv;
  srv.request.a = 1;
  srv.request.b = 2;
  srv.request.c = -41;
  srv.request.d = -42;
  srv.request.e = 360;
  srv.request.f = 0;
  if (client.call(srv))
  {
    new_t = srv.response.g;
  }
  else
  {
    ROS_ERROR("Failed to call service find_roots\n");
  }
  cout<<"New az: "<<new_t<<"\n";
  return 0;
}