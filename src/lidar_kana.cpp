#include "ros/ros.h"
#include <tf/transform_listener.h>

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

  Point& operator + (const Point &point){
    Point temp(x+point.x, y+point.y);
    return temp;
  }

  Point& operator - (const Point &point){
    Point temp(x-point.x, y-point.y);
    return temp;
  }

  Point& operator / (const float &num){
    Point temp(x/num, y/num);
    return temp;
  }

  Point& operator = (const Point &point){
    x = point.x;
    y = point.y;
    return *this;
  }

  static float getEuclideDistance(Point p1, Point p2){
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
  }
};

class Filter{
private:
  float x;
  float pre_x;
  float y;
  float pre_y;
  Point current_point;
  Point before_point;
  Point before2_point;

public:
  Filter(): x(0), pre_x(0), y(0), pre_y(0){
  }

  Filter(float x_, float pre_x_, float y_, float pre_y_): x(x_), pre_x(pre_x_), y(y_), pre_y(pre_y_){
  }

  void set_data(float x_, float pre_x_, float y_, float pre_y_, Point point_){
    x = x_;
    pre_x = pre_x_;
    y = y_;
    pre_y = pre_y_;
    before2_point = before_point;
    before_point = current_point;
    current_point = point_;
  }

  float LPFilter(float x_, float tau_, float ts_){
    y = (tau_ * pre_y + ts_ * x_ )/(tau_ + ts_); 
    pre_x = x_;
    pre_y = y;
    return y;
  }

  Point distanceFilter(Point _current_point, float distance){
    if (Point::getEuclideDistance(_current_point, before_point) > distance){
      ROS_INFO("%f, %f", _current_point.x, _current_point.y);
      ROS_INFO("%f, %f", before_point.x, before_point.y);
      Point result_point = Point(before_point.x + (before_point.x - before2_point.x)/1.1, before_point.y + (before_point.y - before2_point.y)/1.1);
      before2_point = before_point;
      before_point = result_point;;
      return result_point;
    }
    before2_point = before_point;
    before_point = _current_point;
    return _current_point;
  }
};

class Kanayama_lidar{
private:
  ros::NodeHandle nh;
  ros::Publisher v;
  ros::Publisher angle;
  ros::Publisher twist_pub;
  ros::Publisher marker_pub;
  ros::Subscriber path_sub;

  nav_msgs::Path reference_path;
  int path_index;

  int path_first_check;
  int gps_first_check;

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

  Filter x_low_pass_filter;
  Filter y_low_pass_filter;
  Filter point_filter;

public:
  Kanayama_lidar():nh() {
    path_index = 0;
    K_v = 1;
    K_theta = 1;
    K_steer = 40;
    path_first_check = 0;

    v = nh.advertise<std_msgs::Float64>("V", 100);
    angle = nh.advertise<std_msgs::Float64>("angle", 100);
    twist_pub = nh.advertise<geometry_msgs::Twist>("twist_msg", 100);
    marker_pub = nh.advertise<visualization_msgs::Marker>("modem_position", 10);

    path_sub = nh.subscribe("/LPF_path", 1000, &Kanayama_lidar::pathCallback, this);
    lidarKanayama();
  }

  float getRadian(double theta) {
    theta = theta*PI / 180.0;
    return theta;
  }

  void pathCallback(const nav_msgs::Path::ConstPtr& path_data) {
    reference_path = *path_data;
    if (path_first_check == 0){
      path_first_check = 1;
      target_point = Point(reference_path.poses[0].pose.position.x, reference_path.poses[0].pose.position.y);
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
  
  void lidarKanayama(){
    tf::TransformListener listener;
    while (nh.ok()){
      tf::StampedTransform transform;

      try{
        listener.lookupTransform("/map", "/base_link",  
            		  ros::Time::now(), transform);
      }
      catch (tf::TransformException ex){
              ROS_ERROR("%s",ex.what());
              ros::Duration(1.0).sleep();
      }

    Point raw_point(transform.getOrigin().x(), transform.getOrigin().y());
    current_point = raw_point;

    //point_filter.set_data(gps_data->y_m, gps_data->y_m, gps_data->y_m, gps_data->y_m, raw_point);

    /*
    if (gps_first_check < 3) {
      gps_first_check++;
      current_point = Point(gps_data->x_m, gps_data->y_m);
      x_low_pass_filter.set_data(gps_data->x_m, gps_data->x_m, gps_data->x_m, gps_data->x_m, current_point);
      y_low_pass_filter.set_data(gps_data->y_m, gps_data->y_m, gps_data->y_m, gps_data->y_m, current_point);
      point_filter.set_data(gps_data->y_m, gps_data->y_m, gps_data->y_m, gps_data->y_m, current_point);
    }
    else {
      Point raw_point2 = point_filter.distanceFilter(raw_point, 0.4);
      float current_x = x_low_pass_filter.LPFilter(raw_point2.x, 5.0, 1.0);
      float current_y = y_low_pass_filter.LPFilter(raw_point2.y, 5.0, 1.0);
      current_point = Point(current_x, current_y);
    }
    */
    std_msgs::Header car_header;
    car_header.frame_id = "/map";
    car_header.stamp = ros::Time::now();
    drawMarker(car_header, current_point);
  
    while (Point::getEuclideDistance(current_point, target_point) < 0.80) {
      path_index += 1;
      target_point = Point(reference_path.poses[path_index].pose.position.x, reference_path.poses[path_index].pose.position.y);
    }
    drawMarker(car_header, target_point, 1);
  
    if (Point::getEuclideDistance(current_point, pre_point) > 0.05)
      current_heading = atan2(current_point.y - pre_point.y, current_point.x - pre_point.x);
    else
      current_heading = pre_heading;
  
    float x_error = cos(current_heading)*(target_point.x - current_point.x) + sin(current_heading)*(target_point.y - current_point.y);
    float y_error = -sin(current_heading)*(target_point.x - current_point.x) + cos(current_heading)*(target_point.y - current_point.y);
  
    ROS_INFO("path_index:%d", path_index);
    ROS_INFO("current_heading:%f, pre_heading:%f", current_heading/M_PI*180, pre_heading);
    ROS_INFO("target:%f, %f, current:%f, %f", target_point.x, target_point.y, current_point.x, current_point.y);
    ROS_INFO("xerror:%f, yerror:%f", x_error, y_error);
    //
    //V = vr*cos(thetae) + Kx*xe;
    //w = wr + vr*(Ky*ye + ktheta*sin(thetae));
    //float velocity = K_v / y_error;
    float velocity = 0.25;
    float steer = K_steer * y_error;
  
    if(steer > 30) steer = 30;
    if(steer < -30) steer = -30;
  
    //Angle = GR*atan(w*l / vc);
    
    msg.data = velocity;
    msgs.data = steer;
  
    twist_msg.linear.x = velocity;
    twist_msg.angular.z = steer;
    ROS_INFO("steer:%f, speed:%f", steer, velocity);

    // Set our initial shape type to be a cube
    v.publish(msg);
    angle.publish(msgs);
    twist_pub.publish(twist_msg);
  
    pre_point = current_point;
    pre_heading = current_heading;
    }
  }
};

int main(int argc, char **argv){
  ros::init(argc, argv, "kanayama_control");

  Kanayama_lidar kanayama_lidar;
  ros::spin();

  return 0;
}

