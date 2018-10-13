#include "ros/ros.h"
#include <iostream>
#include "std_msgs/String.h"
#include "tf/tf.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Pose2D.h"
#include "vector"
#include "algorithm"
#include "bug_algorithms/bugSwitch.h"
#include "utils.h"
#include "../include/node_pid.h"
#include "../include/node_wallfollowing.h"

static int rateHz = 20;
static int algorithm = -1;
static bool isSimulation = true;
static bool isPoseReady = false;
static bool isLaserReady = false;
static bool isNodeInit = false;
static float linear_vel_, angular_vel_;

enum NodeStates {Waiting, Initializing, Executing, Pause, Stopping};
static int node_state_ = Waiting;

enum States {GoToPoint, AvoidObstacle, FollowBoundary, Done};
static int state_ = GoToPoint;


static map<string, float> regions_ = {
  {"right",0},
  {"front_right",0},
  {"front",0},
  {"front_left",0},
  {"left",0}
};

static map<int, string> node_state_desc = {
  {Waiting, "Waiting"},
  {Initializing, "Initializing"},
  {Executing, "Executing"},
  {Pause, "Pause"},
  {Stopping, "Stopping"}
};

static map<int, string> state_desc_ = {
  {GoToPoint, "Going to point"},
  {AvoidObstacle, "Avoiding obstacle"},
  {FollowBoundary, "Following obstacle boundary"},
  {Done, "Done"}
};

geometry_msgs::Twist twist_msg;
ros::Publisher vel_pub;
ros::Subscriber odom_sub;
ros::Subscriber laser_sub;
ros::Subscriber target_point_sub;

bool behaviorSwitch(bug_algorithms::bugSwitchRequest& request, bug_algorithms::bugSwitchResponse& response){
  node_state_ = request.state;
  cout << "Go to point node state: " << node_state_ << " | " << node_state_desc[node_state_] << endl;

  return true;
}

void changeState(int state){
  state_ = state;
  string s = state_desc_[state_];
  ROS_INFO("Navigate - State changed to: %s", s.c_str());
}

void waitPose(){
  /*This prevents the robot's pose from being 0*/

  bool wait = true;
  ROS_INFO("Waiting for pose...");

  while(!isPoseReady && node_state_ == Initializing){
    ros::spinOnce();
  }
  ROS_INFO("Robot: %f, %f", position_.x, position_.y);

}

void waitLaser(){
  /*This prevents the robot's pose from being 0*/

  bool wait = true;
  ROS_INFO("Waiting for laser...");

  while(!isLaserReady && node_state_ == Initializing){
    ros::spinOnce();
  }
  ROS_INFO("Laser ok!");
}

void stop(){
  twist_msg = geometry_msgs::Twist();
  twist_msg.linear.x = 0.0;
  twist_msg.angular.z = 0.0;
  vel_pub.publish(twist_msg);
  cout << "Go to point stop!\n";
}

void shutDownSubscribers(){
  odom_sub.shutdown();
  laser_sub.shutdown();
}

void initNode(ros::NodeHandle& nh){
  string vel_topic, odom_topic, laser_topic;
  nh.getParam("/algorithm", algorithm);
  nh.getParam("/simulation", isSimulation);
  nh.getParam("/desired_x", desired_position_.x);
  nh.getParam("/desired_y", desired_position_.y);
  nh.getParam("/velocity", linear_vel_);

  ROS_INFO("Desired position: %f | %f", desired_position_.x, desired_position_.y);
  target_position_ = desired_position_;

  angular_vel_ = linear_vel_+0.1;
  isPoseReady = false;

  if(isSimulation){
    cout << "Go_to_point: Using GazeboSim topics\n";
    vel_topic = "cmd_vel";
    odom_topic = "odom";
    laser_topic = "p3dx/laser/scan";
  } else{
    cout << "Go_to_point: Using RosAria topics\n";
    vel_topic = "RosAria/cmd_vel";
    odom_topic = "RosAria/pose";
    laser_topic = "scan";
  }

  vel_pub = nh.advertise<geometry_msgs::Twist>(vel_topic, rateHz);
  odom_sub = nh.subscribe(odom_topic, rateHz, odomCallback);
  laser_sub = nh.subscribe(laser_topic, rateHz, laserCallback);
  target_point_sub = nh.subscribe("bugServer/targetPoint", rateHz, targetPointCallback);

  waitPose();
  waitLaser();

  // Check if the current state is Initializing for the triggered Stopping case
  if(node_state_ == Initializing){
    // Changing node state to Executing
    node_state_ = Executing;
    // Initialize state
    isNodeInit = true;

    changeState(Align);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "navigate_to_point");
  ros::NodeHandle nh;
  bool pauseBand = true;

  ros::ServiceServer service = nh.advertiseService("navigateToPointSwitch", behaviorSwitch);

  ros::Rate rate(rateHz);

  while(!ros::isShuttingDown()){
    if(node_state_ == Waiting){
      ROS_INFO_ONCE("Navigate to point inactive");

    } else if(node_state_ == Initializing){
      initNode(nh);

    } else if(node_state_ == Executing){
      pauseBand = true;

      if(state_ == GoToPoint)
        align(target_position_);
      else if(state_ == AvoidObstacle)
        fixYaw(target_position_,3);
      else if(state_ == FollowBoundary)
        goStraightAhead(target_position_);
      else if(state_ == Done){
        stop();
        node_state_ = Waiting;
      }
      else
        ROS_INFO("Unknown state!");

    } else if(node_state_ == Pause){
      if(pauseBand){
        if(isNodeInit)
          stop();
        pauseBand = false;
      }
    } else if(node_state_ == Stopping){
      if(isNodeInit){
        stop();
        shutDownSubscribers();
        isNodeInit = false;
      }
      node_state_ = Waiting;
    }

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
