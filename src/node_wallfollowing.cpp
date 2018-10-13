#include "../include/node_wallfollowing.h"
#include <math.h>
#define PI 3.141592

NodeWallFollowing::NodeWallFollowing(ros::Publisher pub, double wallDist, double maxSp, int dir, double pr, double di, double an)
{
  wallDistance = wallDist;
  maxSpeed = maxSp;
  direction = dir;
  P = pr;
  D = di;
  angleCoef = an;
  e = 0;
  angleMin = 0;  //angle, at which was measured the shortest distance
  pubMessage = pub;
}

NodeWallFollowing::~NodeWallFollowing()
{
}

//Publisher
void NodeWallFollowing::publishMessage()
{
  //preparing message
  geometry_msgs::Twist msg;

  msg.angular.z = direction*(P*e + D*diffE) + angleCoef * (angleMin - PI*direction/2);    //PD controller

  if (distFront < wallDistance){
    msg.linear.x = 0;
  }
  else if (distFront < wallDistance * 2){
    msg.linear.x = 0.5*maxSpeed;
  }
  else if (fabs(angleMin)>1.75){
    msg.linear.x = 0.4*maxSpeed;
  }
  else {
    msg.linear.x = maxSpeed;
  }

  //publishing message
  pubMessage.publish(msg);
}

//Subscriber
void NodeWallFollowing::messageCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  int size = msg->ranges.size();

  //Variables whith index of highest and lowest value in array.
  int minIndex = size*(direction+1)/4;
  int maxIndex = size*(direction+3)/4;

  //This cycle goes through array and finds minimum
  for(int i = minIndex; i < maxIndex; i++)
  {
    if (msg->ranges[i] < msg->ranges[minIndex] && msg->ranges[i] > 0.0){
      minIndex = i;
    }
  }

  //Calculation of angles from indexes and storing data to class variables.
  angleMin = (minIndex-size/2)*msg->angle_increment;
  double distMin;
  distMin = msg->ranges[minIndex];
  distFront = msg->ranges[size/2];
  diffE = (distMin - wallDistance) - e;
  e = distMin - wallDistance;

  //Invoking method for publishing message
  publishMessage();
}
