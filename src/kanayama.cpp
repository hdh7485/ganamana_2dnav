#include "ros/ros.h"

#include "std_msgs/Float64.h"
#include "std_msgs/Header.h"

#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include <visualization_msgs/Marker.h>

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
  ros::Publisher marker_pub;
  ros::Subscriber gps_sub;
  ros::Subscriber path_sub;

  nav_msgs::Path reference_path;
  int path_index;
  int first_check;

  float current_heading;
  float pre_heading;

  Point pre_point;
  Point current_point;
  Point target_point;

  float K_v;
  float K_theta;
  float K_steer;

  std_msgs::Float64 msg;
  std_msgs::Float64 msgs;
  geometry_msgs::Twist twist_msg;

  visualization_msgs::Marker marker;

public:
  Kanayama():nh() {
    path_index = 0;
    K_v = 1;
    K_theta = 1;
    K_steer = 15;
    first_check = 0;

    v = nh.advertise<std_msgs::Float64>("V", 100);
    angle = nh.advertise<std_msgs::Float64>("angle", 100);
    twist_pub = nh.advertise<geometry_msgs::Twist>("twist_msg", 100);
    marker_pub = nh.advertise<visualization_msgs::Marker>("modem_position", 10);

    gps_sub = nh.subscribe("/hedge_pos_ang", 1000, &Kanayama::gpsCallback, this);
    path_sub = nh.subscribe("/LPF_path", 1000, &Kanayama::pathCallback, this);
  }

  float getRadian(double theta) {
    theta = theta*PI / 180.0;
    return theta;
  }

  void pathCallback(const nav_msgs::Path::ConstPtr& path_data) {
    reference_path = *path_data;
    if (first_check == 0){
      target_point = Point(reference_path.poses[0].pose.position.x, reference_path.poses[0].pose.position.y);
      first_check = 1;
    }
    //std::cout << reference_path << std::endl;
  }

  void drawMarker(std_msgs::Header header, Point point, int id=0){
    marker.header = header;

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = id;

    // Set the marker type
    marker.type = visualization_msgs::Marker::CUBE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = point.x;
    marker.pose.position.y = point.y;
    marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.15; 
    marker.scale.y = 0.15;
    marker.scale.z = 0.06;
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration(5);
     
    marker_pub.publish(marker);
  }
  
  void gpsCallback(const marvelmind_nav::hedge_pos_ang::ConstPtr& gps_data) {
    //ROS_INFO("%f %f", gps_data->x_m, gps_data->y_m);
    Point current_point(gps_data->x_m, gps_data->y_m);
    std_msgs::Header car_header;
    car_header.frame_id = "/gps_path";
    car_header.stamp = ros::Time::now();
    drawMarker(car_header, current_point);
  
    while (Point::getEuclideDistance(current_point, target_point) < 0.25) {
      path_index += 1;
      target_point = Point(reference_path.poses[path_index].pose.position.x, reference_path.poses[path_index].pose.position.y);
    }
    drawMarker(car_header, target_point, 1);
  
    if (Point::getEuclideDistance(current_point, pre_point) > 0.05)
      //current_heading = atan2(current_point.x - pre_point.x, current_point.y - pre_point.y);
      current_heading = atan2(current_point.y - pre_point.y, current_point.x - pre_point.x);
    else
      current_heading = pre_heading;
  
    float x_error = cos(current_heading)*(target_point.x - current_point.x) + sin(current_heading)*(target_point.y - current_point.y);
    float y_error = -sin(current_heading)*(target_point.x - current_point.x) + cos(current_heading)*(target_point.y - current_point.y);
  
    ROS_INFO("path_index:%d", path_index);
    ROS_INFO("current_heading:%f, pre_heading:%f", current_heading/M_PI*180, pre_heading);
    ROS_INFO("target:%f, %f, current:%f, %f", target_point.x, target_point.y, current_point.x, current_point.y);
    ROS_INFO("xerror:%f, yerror:%f", x_error, y_error);

    //V = vr*cos(thetae) + Kx*xe;
    //w = wr + vr*(Ky*ye + ktheta*sin(thetae));
    //float velocity = K_v / y_error;
    float velocity = 0.3;
    float steer = K_steer * y_error;
  
    if(steer > 30) steer = 30;
    if(steer < -30) steer = -30;
  
    //Angle = GR*atan(w*l / vc);
    
    msg.data = velocity;
    msgs.data = steer;
  
    twist_msg.linear.x = velocity;
    twist_msg.angular.z = steer;

    // Set our initial shape type to be a cube
    v.publish(msg);
    angle.publish(msgs);
    twist_pub.publish(twist_msg);
  
    pre_point = current_point;
    pre_heading = current_heading;
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

