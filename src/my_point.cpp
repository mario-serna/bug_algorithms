#include "../include/my_point.h"

MyPoint::MyPoint(double xPos, double yPos, double alpha, ros::Time t)
{
  x = xPos;
  y = yPos;
  angle = alpha;
  time = t;
}

MyPoint::MyPoint(double xPos, double yPos)
{
  x = xPos;
  y = yPos;
}

MyPoint::~MyPoint()
{
}

double MyPoint::getAngle(MyPoint* target)
{
  return atan2((target->y - y),(target->x - x));
}

double MyPoint::getDistance(MyPoint* target)
{
  return sqrt((pow(target->y - y,2.0)) + (pow(target->x - x,2.0)));
}

MyPoint MyPoint::operator+(const MyPoint &other) const
{
   MyPoint result = *(new MyPoint(x+other.x, y+other.y));
   return result;
}

MyPoint MyPoint::operator-(const MyPoint &other) const
{
   MyPoint result = *(new MyPoint(x-other.x, y-other.y));
   return result;
}

MyPoint MyPoint::times(double r){
  return *(new MyPoint(r*x, r*y));
}

double MyPoint::getAbs()
{
  return sqrt((pow(y,2.0)) + (pow(x,2.0)));
}
