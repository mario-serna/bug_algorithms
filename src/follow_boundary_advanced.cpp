#include "ros/ros.h"
#include <iostream>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
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

static int rate_hz = 20;
static int algorithm = -1;
static bool isSimulation = true;
static bool isPoseReady = false;
static bool isLaserReady = false;
static bool isNodeInit = false;
static float linear_vel_, angular_vel_;

enum NodeStates {Waiting, Initializing, Executing, Pause, Stopping};
static int node_state_ = Waiting;

enum States {FindBoundary, TurnLeft, FollowBoundary, TurnLeftOnly};
static int state_ = TurnLeftOnly;
static int count_state_time = 0; // Seconds the robot is in a state
static int count_loop = 0;
static float dist_detection = 0.4;

static float max_laser_range = 4.0;
static float yaw_ = 0;
static float yaw_error_allowed = 5 * (PI/180); // 5 degrees
static geometry_msgs::Point position_ = geometry_msgs::Point();
static geometry_msgs::Point desired_position_ = geometry_msgs::Point();

static bool isLeft = true;
static bool isReverseActive = false;
// This variable is used for making sure the robot is close enough to the obstacle for first time
static bool isCloseEnough = false;
static bool reverseCriterion;
static bool chooseCriterion = true;

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
  {FindBoundary, "find the boundary"},
  {TurnLeft, "turn left"},
  {TurnLeftOnly, "turn left only"},
  {FollowBoundary, "follow the boundary"}
};

geometry_msgs::Twist twist_msg;
ros::Publisher vel_pub;
ros::Publisher lost_obstacle_pub;
ros::Publisher reverse_status_pub;
ros::Subscriber laser_sub;
ros::Subscriber laser2_sub;
ros::Subscriber odom_sub;

bool followBoundarySwitch(bug_algorithms::bugSwitchRequest& request, bug_algorithms::bugSwitchResponse& response){
  node_state_ = request.state;
  cout << "Follow node state: " << node_state_ << " | " << node_state_desc[node_state_] << endl;

  return true;
}

void waitPose(){
  /*This prevents the robot's pose from being 0*/

  bool wait = true;
  ROS_INFO("Waiting for pose...");

  while(!isPoseReady && node_state_ == Initializing){
    ros::spinOnce();
  }
  //ROS_INFO("Robot: %f, %f", position_.x, position_.y);

}

void waitLaser(){
  /*This prevents the robot's pose from being 0*/

  bool wait = true;
  ROS_INFO("Waiting for laser...");

  while(!isLaserReady && node_state_ == Initializing){
    ros::spinOnce();
  }
}

void changeState(int state){
  if(state != state_){
    count_state_time = 0;
    state_ = state;
    string s = state_desc_[state_];
    //ROS_INFO("Follow Boundary - State changed to: %s", s.c_str());
  } else{
    if(node_state_ == Executing){
      std_msgs::Bool isLost;
      if(state_ == FindBoundary && count_state_time > 200){
        isLost.data = true;
        lost_obstacle_pub.publish(isLost);
      } else{
        isLost.data = false;
        lost_obstacle_pub.publish(isLost);
        count_state_time++;
      }
    }
  }
}

void takeActionSimple(){
  float r1,r2,r3,rCrit;

  if(reverseCriterion){
    if(!isReverseActive){
      float desired_yaw = atan2(desired_position_.y - position_.y, desired_position_.x - position_.x);
      float err_yaw = normalizeAngle(desired_yaw - yaw_);

      if(abs(err_yaw) > ((PI/180)*160)){
        isLeft = !isLeft;
        isReverseActive = true;
        cout << "Reverse criterion active!" << endl;
      }
    } else{
      rCrit = isLeft ? regions_["right"]: regions_["left"];
      // Turn left only until the side region is close to the obstacle
      if(rCrit > dist_detection){
        changeState(TurnLeftOnly);
      } else{
        reverseCriterion = false;
        isReverseActive = false;
      }
    }
  }

  if(isLeft){
    r1 = regions_["front_right"];
    r2 = regions_["front_left"];
    r3 = regions_["right"];
  } else{
    r1 = regions_["front_left"];
    r2 = regions_["front_right"];
    r3 = regions_["left"];
  }

  if(!isReverseActive){
    if(!isCloseEnough){
      if(r3 < dist_detection+0.1){
        isCloseEnough = true;
      } else{
        changeState(TurnLeftOnly);
      }
    } else{
      if((r1 < dist_detection) ||
         (regions_["front"] <= dist_detection) ||
         (r2 < dist_detection-0.1)){
        if((regions_["front"] > dist_detection+0.1) && (r3 > 0.3)){
          changeState(TurnLeft);
        } else{
          changeState(TurnLeftOnly);
        }
      } else if(r3 > dist_detection){
        changeState(FindBoundary);
      } else if(r3 > 0.3 && r3 < dist_detection+0.05){
        changeState(FollowBoundary);
      } else{
        changeState(TurnLeftOnly);
      }
    }
  }
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  position_ = msg->pose.pose.position;

  tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  yaw_ = yaw;

  isPoseReady = true;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
  // Check the utils.h file for the functions processRay and myfn
  regions_["right"] = *min_element(begin(msg->ranges), begin(msg->ranges)+200, myfn);
  regions_["right"] = processRay(regions_["right"], max_laser_range);
  regions_["front_right"] = *min_element(begin(msg->ranges)+200, begin(msg->ranges)+310, myfn);
  regions_["front_right"] = processRay(regions_["front_right"], max_laser_range);
  regions_["front"] = *min_element(begin(msg->ranges)+310, begin(msg->ranges)+420, myfn);
  regions_["front"] = processRay(regions_["front"], max_laser_range);
  regions_["front_left"] = *min_element(begin(msg->ranges)+420, begin(msg->ranges)+530, myfn);
  regions_["front_left"] = processRay(regions_["front_left"], max_laser_range);
  regions_["left"] = *min_element(begin(msg->ranges)+530, end(msg->ranges), myfn);
  regions_["left"] = processRay(regions_["left"], max_laser_range);

  isLaserReady = true;

  takeActionSimple();

  //ROS_INFO("IMP\nRight: %f \nFront_right: %f \nFront: %f \nFront_left: %f \nLeft: %f",
  //         regions_["right"], regions_["front_right"], regions_["front"], regions_["front_left"], regions_["left"]);
}

void chooseDirection(){
  if(chooseCriterion){
    if((regions_["left"] < 1 || regions_["right"] < 1)){
      if(regions_["left"] < regions_["right"]){
        isLeft = false;
        cout << "Choose direction: Right ( " << regions_["left"] << " | " << regions_["right"] << " )" << endl;
      } else{
        isLeft = true;
        cout << "Choose direction: Left ( " << regions_["left"] << " | " << regions_["right"] << " )" << endl;
      }

    } else if((regions_["left"]+regions_["front_left"]) < (regions_["right"]+regions_["front_right"])){
      isLeft = false;
      cout << "Choose direction: Right ( " << (regions_["left"]+regions_["front_left"]) << " | " << (regions_["right"]+regions_["front_right"]) << " )" << endl;
    } else{
      isLeft = true;
      cout << "Choose direction: Left ( " << (regions_["left"]+regions_["front_left"]) << " | " << (regions_["right"]+regions_["front_right"]) << " )" << endl;
    }
  } else{
    isLeft = true;
    cout << "Default: Left" << endl;
  }
}

void findBoundary(){
  twist_msg = geometry_msgs::Twist();

  if(angular_vel_ > 0.2){
    twist_msg.linear.x = 0.1;
    twist_msg.angular.z = -0.6;
  }
  else{
    twist_msg.linear.x = linear_vel_*0.5;
    twist_msg.angular.z = -(angular_vel_+(linear_vel_*0.5));
  }

  twist_msg.angular.z = (isLeft ? twist_msg.angular.z : -twist_msg.angular.z);
  vel_pub.publish(twist_msg);
}

void turnLeft(){
  twist_msg = geometry_msgs::Twist();
  twist_msg.linear.x = 0.1;
  if(angular_vel_ > 0.2)
    twist_msg.angular.z = 0.3;
  else
    twist_msg.angular.z = angular_vel_;
  twist_msg.angular.z = (isLeft ? twist_msg.angular.z : -twist_msg.angular.z);
  vel_pub.publish(twist_msg);
}

void turnLeftOnly(){
  twist_msg = geometry_msgs::Twist();
  twist_msg.linear.x = 0.0;
  twist_msg.angular.z = angular_vel_;
  twist_msg.angular.z = (isLeft ? twist_msg.angular.z : -twist_msg.angular.z);
  vel_pub.publish(twist_msg);
}

void followBoundary(){
  twist_msg = geometry_msgs::Twist();
  if(linear_vel_ > 0.2)
    twist_msg.linear.x = 0.3;
  else
    twist_msg.linear.x = linear_vel_;
  twist_msg.angular.z = 0.0;
  vel_pub.publish(twist_msg);
}

void stop(){
  twist_msg = geometry_msgs::Twist();
  twist_msg.linear.x = 0.0;
  twist_msg.angular.z = 0.0;
  vel_pub.publish(twist_msg);
  cout << "Follow boundary stop!\n";
}

void shutDownSubscribers(){
  laser_sub.shutdown();
  odom_sub.shutdown();
  isPoseReady = false;
  isLaserReady = false;
}

void initNode(ros::NodeHandle& nh){
  ros::spinOnce();
  string vel_topic, odom_topic, laser_topic;

  nh.getParam("/algorithm", algorithm);
  nh.getParam("/simulation", isSimulation);
  nh.getParam("/desired_x", desired_position_.x);
  nh.getParam("/desired_y", desired_position_.y);
  nh.getParam("/velocity", linear_vel_);
  nh.getParam("/reverse", reverseCriterion);
  nh.getParam("/choose", chooseCriterion);

  angular_vel_ = linear_vel_+0.1;
  isPoseReady = false;
  isLaserReady = false;
  isCloseEnough = false;

  if(isSimulation){
    cout << "Follow boundary: Using GazeboSim topics\n";
    vel_topic = "cmd_vel";
    odom_topic = "odom";
    laser_topic = "p3dx/laser/scan";
  } else{
    cout << "Follow boundary: Using RosAria topics\n";
    vel_topic = "RosAria/cmd_vel";
    odom_topic = "RosAria/pose";
    laser_topic = "scan";
  }

  vel_pub = nh.advertise<geometry_msgs::Twist>(vel_topic, rate_hz);
  lost_obstacle_pub = nh.advertise<std_msgs::Bool>("lost_obstacle", rate_hz);
  //reverse_status_pub = nh.advertise<std_msgs::Bool>("reverse_status", rate_hz);
  laser_sub = nh.subscribe(laser_topic, rate_hz, laserCallback);
  odom_sub = nh.subscribe(odom_topic, rate_hz, odomCallback);

  waitLaser();
  waitPose();

  // Check if the current state is Initializing for the triggered Stopping case
  if(node_state_ == Initializing){
    // Changing node state to Executing
    node_state_ = Executing;
    // Initialize state
    isNodeInit = true;

    cout << "Simulation? " << isSimulation << endl;
    cout << "Choose criterion? " << chooseCriterion << endl;
    chooseDirection();

    // Initialize state
    changeState(TurnLeftOnly);
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "follow_boundary_advanced");
  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService("followBoundaryAdvancedSwitch", followBoundarySwitch);
  bool pauseBand = true;

  ros::Rate rate(rate_hz);

  while(!ros::isShuttingDown()){
    if(node_state_ == Waiting){
      ROS_INFO_ONCE("Follow inactive");

    } else if(node_state_ == Initializing){
      initNode(nh);

    } else if(node_state_ == Executing){
      pauseBand = true;
      if(state_ == FindBoundary)
        findBoundary();
      else if(state_ == TurnLeft)
        turnLeft();
      else if(state_ == TurnLeftOnly)
        turnLeftOnly();
      else if(state_ == FollowBoundary)
        followBoundary();
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
