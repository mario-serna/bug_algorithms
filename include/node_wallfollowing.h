#ifndef NODE_WALLFOLLOWING_H
#define NODE_WALLFOLLOWING_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

class NodeWallFollowing
{
public:

  /* Constructor:
   * pub   Publisher, which can send commands to robot.
   * wallDist Desired distance from the wall.
   * maxSp Maximum speed, that robot can go.
   * dir   1 for wall on the left side of the robot (-1 for the right side).
   * pr    P constant for PD controller.
   * di    D constant for PD controller.
   * an    Angle coefficient for P controller.
   */
  NodeWallFollowing(ros::Publisher pub, double wallDist, double maxSp, int dir, double pr, double di, double an);

  ~NodeWallFollowing();

  /* This method is generating commands for robot by
   * using data in variables (e, diffE, angleMin).
   */

  void publishMessage();

  /* This method finds minimum distance in data from
   * sensor and process these data into variables
   * (e, diffE, angleMin).
   * msg  Message, which came from robot and contains data from
   * laser scan.
   */

  void messageCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

  //variables
  double wallDistance; // Desired distance from the wall.
  double e;            // Difference between desired distance from the wall and actual distance.
  double diffE;     // Derivative element for PD controller;
  double maxSpeed;     // Maximum speed of robot.
  double P;            // k_P Constant for PD controller.
  double D;            // k_D Constant for PD controller.
  double angleCoef;    // Coefficient for P controller.
  int direction;      // 1 for wall on the right side of the robot (-1 for the left one).
  double angleMin;     // Angle, at which was measured the shortest distance.
  double distFront;    // Distance, measured by ranger in front of robot.
  ros::Publisher pubMessage;  // Object for publishing messages.
};

#endif // NODE_WALLFOLLOWING_H
