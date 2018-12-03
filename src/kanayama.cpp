#include "ros/ros.h"

#include "std_msgs/Float64.h"
#include "std_msgs/Header.h"

#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"

#include "marvelmind_nav/hedge_pos_ang.h"

#include <iostream>
#include <cmath>

#define PI 3.14159256
#define GR 1
#define l  0.275

class Point{
public:
  float x;
  float y;
  Point(){}
  Point(float _x, float _y): x(_x), y(_y){}

  Point& operator = (const Point &point){
    x = point.x;
    y = point.y;
    return *this;
  }

  static float getEuclideDistance(Point p1, Point p2){
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
  }
};

class Kanayama{
private:
  ros::NodeHandle nh;
  ros::Publisher v;
  ros::Publisher angle;
  ros::Publisher twist_pub;
  ros::Subscriber gps_sub;
  ros::Subscriber path_sub;

  nav_msgs::Path reference_path;
  int path_index;

  Point pre_point;
  Point current_point;
  Point target_point;

  float K_v;
  float K_theta;
  float K_steer;

  std_msgs::Float64 msg;
  std_msgs::Float64 msgs;
  geometry_msgs::Twist twist_msg;

public:
  Kanayama():nh() {
    int path_index = 0;
    float K_v = 1;
    float K_theta = 1;
    float K_steer = 1;

    v = nh.advertise<std_msgs::Float64>("V", 100);
    angle = nh.advertise<std_msgs::Float64>("angle", 100);
    twist_pub = nh.advertise<geometry_msgs::Twist>("twist_msg", 100);

    gps_sub = nh.subscribe("/hedge_pos_ang", 1000, &Kanayama::gpsCallback, this);
    path_sub = nh.subscribe("/gps_path", 1000, &Kanayama::pathCallback, this);
  }

  float getRadian(double theta) {
    theta = theta*PI / 180.0;
    return theta;
  }

  void pathCallback(const nav_msgs::Path::ConstPtr& path_data) {
    reference_path = *path_data;
    std::cout << reference_path << std::endl;
  }
  
  void gpsCallback(const marvelmind_nav::hedge_pos_ang::ConstPtr& gps_data) {
    Point current_point(gps_data->x_m, gps_data->y_m);
  
    while (Point::getEuclideDistance(current_point, target_point) < 0.1) {
      path_index += 1;
      target_point = Point(reference_path.poses[path_index].pose.position.x, reference_path.poses[path_index].pose.position.y);
    }
  
    float current_heading = atan2(current_point.x - pre_point.x, current_point.y - pre_point.y);
  
    float x_error = cos(current_heading)*(target_point.x - current_point.x) + sin(current_heading)*(target_point.y - current_point.y);
    float y_error = -sin(current_heading)*(target_point.x - current_point.x) + cos(current_heading)*(target_point.y - current_point.y);
  
    //V = vr*cos(thetae) + Kx*xe;
    //w = wr + vr*(Ky*ye + ktheta*sin(thetae));
    float velocity = K_v / y_error;
    float steer = K_steer * y_error;
  
    if(steer > 30) steer = 30;
    if(steer < -30) steer = -30;
  
    //Angle = GR*atan(w*l / vc);
    
    msg.data = velocity;
    msgs.data = steer;
  
    twist_msg.linear.x = velocity;
    twist_msg.angular.z = steer;
    
    v.publish(msg);
    angle.publish(msgs);
    twist_pub.publish(twist_msg);
  
    pre_point = current_point;
  }
};

int main(int argc, char **argv){
  ros::init(argc, argv, "kanayama_control");

  Kanayama kanayama;
  ros::spin();

  return 0;
/*
  ros::Rate loop_rate(10);

  float xc, yc, thetac; //현재 차량좌표 변수지정필요 아마 0으로 시작하면 되지 않을까 합니다
  float xr, yr, thetar; //목표 좌표 변수지정필요
  float xe, ye, thetae; //오차 변수지정필요 아마 0으로 하면 되지 않을까 생각합니다.
  float vc, vr, w, wr; //현재 속도와 각속도 ,목표 속도와 각속도(정해야한다)
  float Kx = 1;//못정함
  float Ky = 1;//못정함
  float ktheta = 1;//못정함

  float V=0;
  float Angle=1;

  std_msgs::Float64 msg;
  std_msgs::Float64 msgs;
  geometry_msgs::Twist twist_msg;

  while (ros::ok()){
    thetac = getRadian(thetac);
    thetae = getRadian(thetae);
    thetar = getRadian(thetar);
	thetae = thetar - thetac;
	xe = cos(thetac)*(xr - xc) + sin(thetac)*(yr - yc);
	ye = -sin(thetac)*(xr - xc) + cos(thetac)*(yr - yc);
    
    //V = vr*cos(thetae);//보정된 속도
    //w = vr*(Ky*ye + ktheta*sin(thetae));//보정된 각속도
    V = vr*cos(thetae);
    w = vr*(Ky*ye + ktheta*sin(thetae));
	Angle = GR*atan(w*l / vc);

    msg.data=V;
    msgs.data=Angle;
    twist_msg.linear.x = V;
    twist_msg.angular.z = Angle;

    v.publish(msg);
    angle.publish(msgs);
    twist_pub.publish(twist_msg);

    loop_rate.sleep();
    //V++;
    ros::spinOnce();
  }
  return 0;
*/
}

