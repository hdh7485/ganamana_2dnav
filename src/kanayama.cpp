#include "ros/ros.h"

#include "std_msgs/Float64.h"
#include "std_msgs/Header.h"

#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"

#include <iostream>
#include <cmath>

#define PI 3.14159256
#define GR 1
#define l  0.275

nav_msgs::Path = reference_path;

float getRadian(double theta){
  theta = theta*PI / 180;
  return theta;
}

void pathCallback(const nav_msgs::Path::ConstPtr& path_data){
  reference_path = *path_data;
  std::cout << reference_path << std::endl;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "kanayama_control");
  ros::NodeHandle nh;
  ros::Publisher v = nh.advertise<std_msgs::Float64>("V", 100);
  ros::Publisher angle = nh.advertise<std_msgs::Float64>("angle", 100);

  ros::Rate loop_rate(0.5);

  float xc, yc, thetac; //현재 차량좌표 변수지정필요 아마 0으로 시작하면 되지 않을까 합니다
  float xr, yr, thetar; //목표 좌표 변수지정필요
  float xe, ye, thetae; //오차 변수지정필요 아마 0으로 하면 되지 않을까 생각합니다.
  float vv, vr, w, wr; //현재 속도와 각속도 ,목표 속도와 각속도(정해야한다)
  float Kx = 1;//못정함
  float Ky = 1;//못정함
  float ktheta = 1;//못정함

  float V=0;
  float Angle=1;

  while (ros::ok()){
    std_msgs::Float64 msg;
    std_msgs::Float64 msgs;
    thetac = getRadian(thetac);
    thetae = getRadian(thetae);
    thetar = getRadian(thetar);
	thetae = thetar - thetac;
	ye = -sin(thetac)*(xr-xc)+cos(thetac)*(yr-yc);
    
    V = vr*cos(thetae)//보정된 속도
    w = vr*(Ky*ye + ktheta*sin(thetae));//보정된 각속도
	Angle = GR*atan(w*l / vc);

    msg.data=V;
    msgs.data=Angle;
    v.publish(msg);
    angle.publish(msgs);
    loop_rate.sleep();
    V++;
  }
  return 0;
}

