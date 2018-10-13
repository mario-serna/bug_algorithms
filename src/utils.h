#ifndef UTILS_H
#define UTILS_H

#include "ros/ros.h"
#include "iostream"
#include "math.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"
#include "sstream"
#include "iomanip"

using namespace std;

const double PI = 3.1415926;

// 1 = upward | -1 = downward
static int robot_direction = 1;

template <typename T>
std::string to_string_with_precision(const T a_value, const int n = 6)
{
  // Code from: https://stackoverflow.com/questions/16605967/set-precision-of-stdto-string-when-converting-floating-point-values

    std::ostringstream out;
    out << std::setprecision(n) << a_value;
    return out.str();
}

double degrees2radians(double angle_in_degrees){
  return angle_in_degrees * PI / 180.0;
}

double radians2degrees(double angle_in_radians){
  return angle_in_radians * 180.0 / PI;
}

int mapRange(float y, float y_min, float y_max, float x_min, float x_max){
  return (y-y_min)/(y_max-y_min)*(x_max-x_min)+x_min;
}

float mapRangeFloat(float y, float y_min, float y_max, float x_min, float x_max){
  return (y-y_min)/(y_max-y_min)*(x_max-x_min)+x_min;
}

double normCyclic(double val, double min, double max){
  if(val >= min){
    return min + fmod((val - min), (max - min));
  } else{
    return max - fmod((min - val), (max - min));
  }
}

double normDeg360(double angle){
  return normCyclic(angle, 0 ,360);
}

double normalizeAngle(double angle){
  if(abs(angle) > PI){
    angle = angle - (2*PI*angle) / (abs(angle));
  }

  return angle;
}

double round(double var, int presition)
{
  // 37.66666 * 100 =3766.66
  // 3766.66 + .5 =37.6716    for rounding off value
  // then type cast to int so value is 3766
  // then divided by 100 so the value converted into 37.66
  double value = (int)(var * pow(10, presition) + .5);
  return (double)value / pow(10, presition);
}

geometry_msgs::Point pose2DToPoint(geometry_msgs::Pose2D pose){
  geometry_msgs::Point p;
  p.x = pose.x;
  p.y = pose.y;
  return p;
}

double getDistance(double x1, double y1, double x2, double y2){
  return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

double getDistance(geometry_msgs::Point a, geometry_msgs::Point b){
  return sqrt(pow(b.x - a.x, 2) + pow(b.y - a.y, 2));
}

double getPendant(geometry_msgs::Point a, geometry_msgs::Point b){
  double m;

  a.x = round(a.x, 2);
  a.y = round(a.y, 2);
  b.x = round(b.x, 2);
  b.y = round(b.y, 2);

  if((b.x - a.x) != 0){
    if((b.y - a.y) != 0)
      m = (b.y - a.y) / (b.x - a.x);
    //m = round((b.y - a.y) / (b.x - a.x), 1);
    else
      b.x - a.x < 0 ? m = -99999 : m = 0 ;
  } else{
    m = 99999;
  }
  //  cout << "b.x: "<<b.x << " | a.x: "<<a.x << endl;
  //  cout << "b.y: "<<b.y << " | a.y: "<<a.y << endl;
  //  cout<<"M: "<< m <<endl;
  //  cout<<"Msign: "<< copysign(1, m) <<endl;

  return m;
}

double getAlpha(geometry_msgs::Point a, geometry_msgs::Point b){
  double alpha;
  double m;

  robot_direction = 1;

  //TODO: poner condicion de tolerancia al ingresar una posicion muy cercana al robot para evitar que gire.
  // La pendiente debe ser diferente de 0

  m = getPendant(a, b);

  //ROS_INFO("M: %f", m);

  if(m != 99999){
    if((b.y - a.y) < 0){
      robot_direction = -1;
    }

    if(robot_direction == 1){
      alpha = radians2degrees(atan(m));

      if(copysign(1, m) == -1){
        m == -99999 ? m = 0 : m;
        alpha = 180 + radians2degrees(atan(m));
      }

    } else{
      if(copysign(1, m) == -1){
        m == -99999 ? m = 0 : m;
        alpha = radians2degrees(atan(m));
      } else{
        alpha = robot_direction * (180 - radians2degrees(atan(m)));
      }
    }
  } else{
    alpha = 90;
    if((b.y - a.y) < 0){
      alpha = -90;
      robot_direction = -1;
    }
  }

  return alpha;

}

bool isOnPendant(geometry_msgs::Point initial, geometry_msgs::Point goal, geometry_msgs::Point point){
  //Way 1: Less expensive but more unstable
  double m1, m2 = 0;

  m1 = round(getPendant(initial, goal), 2);
  m2 = round(getPendant(point, goal), 2);

  m1 < -50 || m1 > 50 ? m1 = 99999: m1;
  m2 < -50 || m2 > 50 ? m2 = 99999: m2;

  cout << "M1: " << m1 << " | M2: " << m2 << endl;

  return abs(m1 - m2) <= 0.05;
}

bool isOnPointRange(geometry_msgs::Point robot, geometry_msgs::Point point,float tolerance){
  if((abs(robot.x - point.x) < tolerance) &&
     (abs(robot.y - point.y) < tolerance)){
    //ROS_INFO("Robot is in the hit point range!");
    return true;
  }
  //ROS_INFO("Robot is out of range from the hit point!");
  return false;
}

// Function that checks whether the value is not a number or it has a value of infinite
// To avoid problems with the laser value presition
float processRay(float value, float maxRange){
  return isnan(value) || isinf(value) ? maxRange : (value > maxRange ? maxRange : (value < 0.15 ? maxRange : value));
}

// Function useful for detecting discontinuity points
float processRaySimple(float value, float maxRange){
  return isnan(value) || isinf(value) ? maxRange : value;
}

// Function that compares two numbers, it's used for checking the laser ray
// 4 is the max laser range
bool myfn(float i, float j) {
  i = processRay(i, 4);
  j = processRay(j, 4);
  return i<j;
}

visualization_msgs::Marker createMarker(string frame_id, string ns, int id, int type, int action, geometry_msgs::Point p, geometry_msgs::Vector3 scale, std_msgs::ColorRGBA color){
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time();
  marker.ns = ns;
  marker.id = id;
  marker.type = type;
  marker.action = action;
  marker.pose.position.x = p.x;
  marker.pose.position.y = p.y;
  marker.pose.position.z = p.z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = scale.x;
  marker.scale.y = scale.y;
  marker.scale.z = scale.z;
  marker.color.a = color.a; // Don't forget to set the alpha!
  marker.color.r = color.r;
  marker.color.g = color.g;
  marker.color.b = color.b;

  return marker;
}

#endif // UTILS_H
