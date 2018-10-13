#ifndef MY_POINT_H
#define MY_POINT_H

#include <math.h>
#include "ros/ros.h"

class MyPoint
{
public:

  /* Constructor:
   * xPos   Position in x.
   * yPos   Position in y.
   * alpha  Robot rotation.
   * t      Time of measurement
   */
  MyPoint(double xPos, double yPos, double alpha, ros::Time t);
  MyPoint(double xPos, double yPos);

  ~MyPoint();

  /* Returns angle between this and target point.
   */
  double getAngle(MyPoint* target);

  /* Returns distance between this and target point.
   */
  double getDistance(MyPoint* target);

  /* Sum of vectors
    */
  MyPoint operator+(const MyPoint &other) const;

  /* Difference of vectors
    */
  MyPoint operator-(const MyPoint &other) const;

  /* Multiplication of vector by real number
    */
  MyPoint times(double r);

  /* Absolute value of vector.
    */
  double getAbs();

  //variables
  double x;       // Position in x.
  double y;       // Position in y.
  ros::Time time; // Time, when the position was measured.
  double angle;   // Robot's angle.
};

#endif // MY_POINT_H
