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
#include <velocityobs/MyRoots.h>

using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace nav_msgs;
using namespace message_filters;
using namespace cv;
using namespace ros;
using namespace std;

ros::Publisher vel_pub;
ros::ServiceClient client;
geometry_msgs::Twist twist;

int Zinit = 1;
int Yinit = 0;
int Xinit = 0;
int Xfinal = 6;
float v_x = 0.1;                /*quad velocities*/
float v_y = 0;
float v_z = 0;
float safe_dist = 3;
float R = 0.75;           //Inflated radius  
float ob1_x = 6;        //Obstacle data to be found out
float ob1_y = 0;
float ob1_z = 1;
float ob1Vel_x = 0;
float ob1Vel_y = 0;
float ob1Vel_z = 0;

float my_p[3], del_p[3], L_vel[3], slant[3], d[3];  
float my_p_prev[3] = {0};      /*Quad Pose*/
double t1;
double t2 = 0;
float rX,rY,rZ;
float r_Prev = 0;     

int cntr = 0;
double vel_avg = 0;
double vel;
float r_Mag;
float trans[4];
bool init_trans = false;

void Divert(float v_xx, float v_yy, float v_zz, float d, float vqq)
{
  float count = 0;
  //float del_t;
  double del_t;
  //clock_t t;
  double t;
  while (count < d/vqq)
  {
    //t = clock();
    t = ros::Time::now().toSec();
    twist.linear.x = v_xx;
    twist.linear.y = v_yy;
    twist.linear.z = 0; /*no velocity should be imparted in z direction*/
    vel_pub.publish(twist);
    usleep(500);
    //t = clock() - t;
    del_t = ros::Time::now().toSec() - t;
    //del_t = ((float)t)/CLOCKS_PER_SEC;
    count = count + del_t;
    cout<<"count: "<<count<<"\n";
    cout<<"cmd_divert in X: "<<v_xx<<"\tcmd_divert in Y: "<<v_yy<<"\tcmd_divert in Z: "<<v_zz<<"\n";
  }
}

void initialCallback(const OdometryConstPtr& p)
{
  my_p[0] = p->pose.pose.position.x;
  my_p[1] = p->pose.pose.position.y;
  my_p[2] = p->pose.pose.position.z;
  if (!init_trans)
  {
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
  }
  cout<<"Actual Pose: "<<my_p[0]<<"\t"<<my_p[1]<<"\t"<<my_p[2]<<"\n";
  my_p[0] = trans[0]*my_p[0] + trans[1]*my_p[1];
  my_p[1] = trans[2]*my_p[0] + trans[3]*my_p[1];
  cout<<"Transformed X: "<<my_p[0]<<"\tY: "<<my_p[1]<<"\tZ: "<<my_p[2]<<"\n";
  t1 = p->header.stamp.toSec();
  vel = ((my_p[0] - my_p_prev[0])/(t1-t2));
  if (cntr > 5)
  {
    vel_avg = vel_avg/5;
    twist.linear.x = vel_avg;
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.z = 0;

    rX = ob1_x - my_p[0];
    rY = ob1_y - my_p[1];
    rZ = ob1_z - my_p[2];
    r_Mag = sqrt(pow(rX,2) + pow(rY,2) + pow(rZ,2));
    float vx = vel_avg;
    cout<<"Average vx = "<<vx<<"\n";
    cout<<"r_Mag: "<<r_Mag<<"\n";
    if (vx<0)
    {
      cout<<"vx sign adjusted\n";
      vx = -1*vx;
    }
    float vy = 0;           /*considering motion in x direction only*/
    float vz = 0;
    L_vel[0] = vx - ob1Vel_x;
    L_vel[1] = vy - ob1Vel_y;
    L_vel[2] = vz - ob1Vel_z;
    float L_vel_Mag = sqrt(pow(L_vel[0],2) + pow(L_vel[1],2) + pow(L_vel[2],2));
    float Vo1_Mag = sqrt(pow(ob1Vel_x,2) + pow(ob1Vel_y,2) + pow(ob1Vel_z,2));
    float dp = L_vel[0]*rX + L_vel[1]*rY + L_vel[2]*rZ;
    slant[0] = (dp/pow(L_vel_Mag,2))*L_vel[0];
    slant[1] = (dp/pow(L_vel_Mag,2))*L_vel[1];
    slant[2] = (dp/pow(L_vel_Mag,2))*L_vel[2];
    d[0] = slant[0] - rX;
    d[1] = slant[1] - rY;
    d[2] = slant[2] - rZ;
    float d_Mag = sqrt(pow(d[0],2) + pow(d[1],2) + pow(d[2],2));
    float r_dot = r_Mag - r_Prev;
    r_Prev = r_Mag;
    t2 = t1;
    cout<<"d_Mag: "<<d_Mag<<"\tr_dot: "<<r_dot<<"\n";

    if (d_Mag<R && r_dot<0)
    {
      cout<<"Collision "<<clock()<<"\n";
    }
    double Vq,az,el,new_t,Vq_new_Y,Vq_new_X,Vq_new_Z;
    float H,K,a1,a2,a3,a4,a5,b, p4,p3,p2,p1,p0;

    if (my_p[0] >= Xfinal)
    {
      twist.linear.x = 0;
      twist.linear.y = 0;
      twist.linear.z = 0;
      twist.angular.z = 0;
      vel_pub.publish(twist);
    }
    else 
    {
      if (r_Mag > safe_dist)
      {
        twist.linear.x = v_x;
        twist.linear.y = 0;
        twist.linear.z = 0;
        twist.angular.z = 0;
        vel_pub.publish(twist);
      }
      else
      {
        Vq = sqrt(pow(vx,2) + pow(vy,2) + pow(vz,2));  //Check once
        //el = acos(twist.linear.y/Vq);
        el = acos(sqrt(pow(vx,2) + pow(vy,2))/Vq);
        az = atan2(double(vy),double(vx));

        cout<<"Vq: "<<Vq<<"\tel: "<<el<<"\taz: "<<az<<"\n"; 
        H = (pow(r_Mag,2)-pow(R,2))/pow(Vq,2);
        K = (rX*ob1Vel_x + rY*ob1Vel_y + rZ*ob1Vel_z)/Vq;
        a1 = pow(rX,2)*pow(cos(el),2);
        a2 = pow(rY,2)*pow(cos(el),2);
        a3 = 2*rX*rY*pow(cos(el),2);
        a4 = 2*H*Vq*(ob1Vel_x)*cos(el)-2*rX*rZ*cos(el)*sin(el)-2*K*rX*cos(el);
        a5 = 2*H*Vq*(ob1Vel_y)*cos(el)-2*rY*rZ*cos(el)*sin(el)-2*K*rY*cos(el);
        b = H*(pow(Vq,2)+pow(Vo1_Mag,2))+2*H*Vq*ob1Vel_z*sin(el)-pow(rZ,2)*pow(sin(el),2)-2*K*rZ*sin(el)-pow(K,2);

        p4 = a1-a4-b;
        p3 = 2*(a5-a3);
        p2 = 2*(2*a2-a1-b);
        p1 = 2*(a3+a5);
        p0 = a1+a4-b;

        velocityobs::MyRoots srv;
        srv.request.a = p4;
        srv.request.b = p3;
        srv.request.c = p2;
        srv.request.d = p1;
        srv.request.e = p0;
        srv.request.f = az;
        if (client.call(srv))
        {
          new_t = srv.response.g;
        }
        else
        {
          ROS_ERROR("Failed to call service find_roots\n");
        }
        cout<<"New az: "<<new_t<<"\n";

        Vq_new_X = Vq*cos(el)*cos(new_t);
        Vq_new_Y = Vq*cos(el)*sin(new_t);
        Vq_new_Z = Vq*sin(el);

        cout<<"Vq_new_X: "<<Vq_new_X<<"\tVq_new_Y: "<<Vq_new_Y<<"\tVq_new_Z: "<<Vq_new_Z<<"\n";
        float v_xx = 0.249744*pow(Vq_new_X,3) - 0.2286*pow(Vq_new_X,2) + 0.1863*Vq_new_X + 0.0042; //mapping from velocity to cmd_vel value
        float v_yy = 0.249744*pow(Vq_new_Y,3) - 0.2286*pow(Vq_new_Y,2) + 0.1863*Vq_new_Y + 0.0042;
        cout<<"cmd_new_X: "<<v_xx<<"\tcmd_new_Y: "<<v_yy<<"\tcmd_new_Z: "<<Vq_new_Z<<"\n";
        usleep(1000);
        if (v_xx<v_x)
        {
          Divert(v_xx, v_yy, 0, safe_dist, (Vq + Vo1_Mag));
          //double del_Vq_new_X = vx - Vq_new_X;
          float v_yyy = -v_yy;

          // cout<<"Vq_new_XX: "<<Vq_new_X<<"\tVq_new_YY: "<<vy + del_Vq_new_Y<<"\tVq_new_ZZ: "<<Vq_new_Z<<"\n";

          Divert(v_xx, v_yyy, 0, safe_dist, (Vq + Vo1_Mag));
        }
        else
        {
          cout<<"Generated velocities exceed limit\n";
        }
      }
    }
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
  ros::init(argc, argv, "newvel");
  ros::NodeHandle nh;
  client = nh.serviceClient<velocityobs::MyRoots>("find_roots");
  vel_pub = nh.advertise<geometry_msgs::Twist>("/self/cmd_vel", 1);
  ros::Subscriber my_pose = nh.subscribe("/bebop/odom", 1, initialCallback);
  ros::spin();
  return 0;
}
