#include "ros/ros.h"
#include <iostream>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"
#include "tf/tf.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Pose2D.h"
#include "vector"
#include "algorithm"
#include "std_srvs/SetBool.h"
#include "std_srvs/Empty.h"
#include "utils.h"
#include "bug_algorithms/bugSwitch.h"
#include "bug_algorithms/nodeState.h"
#include "bug_algorithms/algorithmState.h"
#include <ros/callback_queue.h>
#include "visualization_msgs/Marker.h"

static int rate_hz = 20;
static bool isSimulation = true;
static bool isPoseReady = false;
static bool isLaserReady = false;
static bool areSubsDown = true;
static float linear_vel_, angular_vel_;
static bug_algorithms::nodeState nodeStateGlobal;
static string node_name = "tangentBug";
static int algorithm_id = 6;
static float node_state_time = 0;
static float state_time = 0;
static ros::Time previous_time;

enum NodeStates {Waiting, Initializing, Executing, Pause, Stopping};
static int node_state_ = Waiting;

enum States {GoToPoint, GoToPointFollowing, FollowBoundary, Success, Fail};
static int state_ = GoToPoint;
static int count_state_time = 0; // Seconds the robot is in a state
static int count_loop = 0;
static int count_tolerance = 0; // Seconds the robot must wait
static int laser_align_index;
static int laser_samples = 727;
static int angle_increment;

static float laser_angle = PI/1.5; // +-120 degrees
static float max_laser_range = 4.0;
static float yaw_ = 0;
static float yaw_error_allowed = 5 * (PI/180); // 5 degrees
static geometry_msgs::Point position_ = geometry_msgs::Point();
static geometry_msgs::Point laser_position_ = geometry_msgs::Point();
static geometry_msgs::Point initial_position_ = geometry_msgs::Point();
static geometry_msgs::Point desired_position_ = geometry_msgs::Point();
static geometry_msgs::Point closest_point = geometry_msgs::Point();
static geometry_msgs::Point leave_point = geometry_msgs::Point();
static geometry_msgs::Point sensed_closest_point = geometry_msgs::Point();
// M is the point on the sensed obstacle which has the shorted distance to the goal
static geometry_msgs::Point m_point = geometry_msgs::Point();

static bool isSensedClosestPointInit = false;
static bool isClosestPointInit = false;
static bool isHitPointInit = false;
static bool leaveCondition = false;
static int checkCondition = 0;
static int count_same_point = 0;
static float yaw_precision_ = PI/90; // 2 degrees
static float desired_yaw = 0.0;
static float err_yaw = 0.0;
static float dist_precision_ = 0.3;
static float dist_detection = 0.4;
static float initial_to_goal_distance = 0.0;
static float current_to_goal_distance = 0.0;
static float free_distance = 0.0;
static float best_distance = 0.0;
// Followed distance is the shortest distance between the sensed boundary and the goal
static float followed_distance = 0.0;
// Reach_distance is the shortest distance between blocking obstacle and goal
// (or the current distance to goal if no blocking obstacle visible)
static float reach_distance = 0.0;
static float leave_distance = 0.0;
static bool lockedPoint = false;
static bool isInitAlign = true;
geometry_msgs::Point target = geometry_msgs::Point();

static bool reverseCriterion;

static bool isPreviousReady = false;
static geometry_msgs::Point previous_position_ = geometry_msgs::Point();
static float path_length = 0.0;

// 0: None, 1: Left, 2: Right
static int hand_tracking = 0;
static float left_hand_angle;
static float right_hand_angle;

// Marker variables
static bool pubMarker = false;
static bool isPathMarkerReady = false;
static visualization_msgs::Marker start_text_marker = visualization_msgs::Marker();
static visualization_msgs::Marker goal_text_marker = visualization_msgs::Marker();
static visualization_msgs::Marker total_path_text_marker = visualization_msgs::Marker();
static visualization_msgs::Marker start_marker = visualization_msgs::Marker();
static visualization_msgs::Marker goal_marker = visualization_msgs::Marker();
static visualization_msgs::Marker path_marker = visualization_msgs::Marker();
static visualization_msgs::Marker path_marker_go_to = visualization_msgs::Marker();
static visualization_msgs::Marker path_marker_follow = visualization_msgs::Marker();
static visualization_msgs::Marker target_marker = visualization_msgs::Marker();
static visualization_msgs::Marker disPoint_marker = visualization_msgs::Marker();
static visualization_msgs::Marker m_marker = visualization_msgs::Marker();

static map<string, float> regions_ = {
  {"right",0},
  {"front_right",0},
  {"front",0},
  {"front_left",0},
  {"left",0}
};

static map<string, float> hand = {
  {"right",0},
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
  {GoToPoint, "Go to the point"},
  {GoToPointFollowing, "Go to the discontinuity point"},
  {FollowBoundary, "Follow the obstacle boundary"},
  {Success, "Goal reached"},
  {Fail, "Fail - Goal not reachable"}
};

struct discontinuity_point
{
  int index;
  float value;
  // 0 = left | 1 = right | 2 = ignore
  int direction;
  bool is_t_point;
};

geometry_msgs::Twist twist_msg;
ros::Publisher node_state_pub;
ros::Publisher target_point_pub;
ros::Publisher algorithm_state_pub;
ros::Publisher marker_pub;
ros::Subscriber node_state_sub;
ros::Subscriber odom_sub;
ros::Subscriber laser_sub;
ros::Subscriber laser_align_sub;
ros::Subscriber laser_detect_discont_sub;
ros::Subscriber lost_obstacle_sub;
ros::ServiceClient srv_client_go_to_point;
ros::ServiceClient srv_client_follow_boundary;
ros::ServiceClient srv_client_change_hand;

ros::CallbackQueue custom_queue;

void subscriberNotify(string m){
  cout << "Subscriber active: " << m << endl;
}

int isFreePath(geometry_msgs::Point desired, float tol);

bool bugSwitch(bug_algorithms::bugSwitchRequest& request, bug_algorithms::bugSwitchResponse& response){
  string message;

  response.success = true;
  message.append(node_state_desc[node_state_]);
  message.append("->");
  message.append(node_state_desc[request.state]);

  if(node_state_ == Waiting && request.state == Initializing){
    node_state_ = request.state;
  } else if(node_state_ == Initializing && request.state == Stopping){
    node_state_ = request.state;
  } else if(node_state_ == Executing && (request.state == Pause || request.state == Stopping)){
    node_state_ = request.state;
  } else if(node_state_ == Pause && (request.state == Executing || request.state == Stopping)){
    node_state_ = request.state;
  } else{
    response.success = false;
    message = "";
    message.append("Current state: ");
    message.append(node_state_desc[node_state_]);
    message.append(" | State not allowed: ");
    message.append(node_state_desc[request.state]);
  }

  response.message = message;

  return true;
}

void publishNodeState(){

  bug_algorithms::nodeState n = bug_algorithms::nodeState();
  n.algorithm = algorithm_id;
  n.node_state = node_state_;
  n.node_state_desc = node_state_desc[node_state_];
  n.node_state_time = node_state_time;
  n.bug_state = state_;
  n.bug_state_desc = state_desc_[state_];
  n.bug_state_time = state_time;

  node_state_pub.publish(n);
}

void nodeStateCallback(const bug_algorithms::nodeState::ConstPtr& msg){
  nodeStateGlobal.algorithm = msg->algorithm;
  nodeStateGlobal.bug_state = msg->bug_state;
  nodeStateGlobal.bug_state_desc = msg->bug_state_desc;
  nodeStateGlobal.bug_state_time = msg->bug_state_time;
  nodeStateGlobal.node_state = msg->node_state;
  nodeStateGlobal.node_state_desc = msg->node_state_desc;
  nodeStateGlobal.node_state_time = msg->node_state_time;
}

void publishPathMarkers(){
  if(isPathMarkerReady){
    marker_pub.publish(start_text_marker);
    marker_pub.publish(goal_text_marker);
    total_path_text_marker.text = "P(" + to_string_with_precision(path_length, 3) + "m)";
    marker_pub.publish(total_path_text_marker);
    marker_pub.publish(start_marker);
    marker_pub.publish(goal_marker);
    target_marker.pose.position.x = target.x;
    target_marker.pose.position.y = target.y;
    target_marker.pose.position.z = 0.8;
    marker_pub.publish(target_marker);
    geometry_msgs::Point p;
    if(m_point.z < sensed_closest_point.z){
      p = m_point;
    } else{
      p = sensed_closest_point;
    }
    m_marker.pose.position.x = p.x;
    m_marker.pose.position.y = p.y;
    m_marker.pose.position.z = 0.8;
    marker_pub.publish(m_marker);
    marker_pub.publish(disPoint_marker);
    marker_pub.publish(path_marker);
    marker_pub.publish(path_marker_go_to);
    marker_pub.publish(path_marker_follow);
  }
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
  //subscriberNotify(node_name);
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

  // Laser pose stimation
  laser_position_.x  = position_.x + round(0.12*cos(yaw), 2);
  laser_position_.y  = position_.y + round(0.12*sin(yaw), 2);

  isPoseReady = true;

  if(isPreviousReady){
    float a_x,a_y,b_x,b_y;
    a_x = round(previous_position_.x, 1);
    a_y = round(previous_position_.y, 1);
    b_x = round(position_.x, 1);
    b_y = round(position_.y, 1);
    path_length = path_length + getDistance(a_x,a_y,b_x,b_y);
  }
  else{
    isPreviousReady = true;
    path_length = 0;
  }
  previous_position_ = position_;

  bug_algorithms::algorithmState a_state;
  a_state.algorithm = algorithm_id;
  a_state.name = node_name;
  a_state.pose_x = position_.x;
  a_state.pose_y = position_.y;
  a_state.yaw = yaw_;
  a_state.initial_to_goal_distance = initial_to_goal_distance;
  a_state.current_to_goal_distance = current_to_goal_distance;
  a_state.best_distance = best_distance;
  a_state.path_length = path_length;
  if(node_state_ == Initializing)
    a_state.time = 0;
  else
    a_state.time = ros::Time::now().toSec() - previous_time.toSec();
  algorithm_state_pub.publish(a_state);

  if(pubMarker && isPathMarkerReady){
    pubMarker = false;
    // Publishing path marker
    path_marker.points.push_back(position_);
    marker_pub.publish(path_marker);
    if(state_ == GoToPoint || state_ == GoToPointFollowing){
      path_marker_go_to.points.push_back(position_);
      marker_pub.publish(path_marker_go_to);
    } else{
      path_marker_follow.points.push_back(position_);
      marker_pub.publish(path_marker_follow);
    }
  }
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
  laser_samples = msg->ranges.size();
  angle_increment = msg->angle_increment;
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

  // (0° - 18°) 121 - 55 = 66 || (0° + 18°) 121 + 55 = 176
  auto i = min_element(begin(msg->ranges)+66, begin(msg->ranges)+176, myfn);
  hand["right"] = *i;
  hand["right"] = processRay(hand["right"], max_laser_range);
  right_hand_angle = i - begin(msg->ranges);

  i = min_element(end(msg->ranges)-176, end(msg->ranges)-66, myfn);
  hand["left"] = *i;
  hand["left"] = processRay(hand["left"], max_laser_range);
  left_hand_angle = i - begin(msg->ranges);

  right_hand_angle = (right_hand_angle-laser_samples/2)*angle_increment;
  left_hand_angle = (left_hand_angle-laser_samples/2)*angle_increment;

  isLaserReady = true;

  //cout << "Hand right: " << hand["right"] << " | left: " << hand["left"] << endl;
  //cout << "Hand right: " << right_hand_angle << " | left: " << left_hand_angle << endl;

  //ROS_INFO("IMP\nRight: %f \nFront_right: %f \nFront: %f \nFront_left: %f \nLeft: %f",
  //         regions_["right"], regions_["front_right"], regions_["front"], regions_["front_left"], regions_["left"]);
}

void laserAlignCallback(const sensor_msgs::LaserScan::ConstPtr& msg){

  float d_yaw = atan2(desired_position_.y - laser_position_.y, desired_position_.x - laser_position_.x);
  float e_yaw = normalizeAngle(d_yaw - yaw_);
  float free_temp;

  if(abs(e_yaw) < PI/1.5){
    laser_align_index = mapRange(e_yaw, -laser_angle, laser_angle, 0, msg->ranges.size()-1);
    free_temp = processRaySimple(msg->ranges[laser_align_index], max_laser_range);
    // Avoiding laser error
    if(free_temp > 0.15)
      free_distance = free_temp;
    /*if(isnan(msg->ranges[laser_align_index])){
      free_distance = 4;
    } else{
      free_distance = msg->ranges[laser_align_index] > 4 ? 4 : msg->ranges[laser_align_index];
    }*/
    //cout << "Err Yaw: " << e_yaw << " | Des Yaw: " << d_yaw << " | Cur Yaw: " << yaw_ << endl;
    //cout << "N: " << laser_align_index << endl;
    //cout << "Laser: " << free_distance << endl;
  } else{
    free_distance = 0;
    laser_align_index = -1;
  }
}

void laserDetectDiscontinuityCallback(const sensor_msgs::LaserScan::ConstPtr& msg){

  enum direction{Left, Right, Ignore};
  int n;
  int size = msg->ranges.size();
  int gap_num = 0;
  float best_path_distance = 9999.0;
  float closest_point_distance = 9999.0;
  float best_point_yaw = 9999.0;
  float path_distance, current_to_point_distance, point_to_goal_distance;
  float a,b;
  geometry_msgs::Point p = geometry_msgs::Point();
  // Collision Avoidance Point
  geometry_msgs::Point cap = geometry_msgs::Point();
  int count_discontinuity_points = 0;
  vector<discontinuity_point> dis_points = vector<discontinuity_point>();
  bool temp_lock = false;

  desired_yaw = atan2(desired_position_.y - position_.y, desired_position_.x - position_.x);
  err_yaw = normalizeAngle(desired_yaw - yaw_);

  int temp_size = 0;
  // Detecting discontinuity points
  for(int i = 2; i < size-2; i++){

    discontinuity_point a,b;

    a.is_t_point = b.is_t_point = false;

    temp_size = dis_points.size();
    a.index = i;
    b.index = i+1;
    a.value = isnan(msg->ranges[a.index]) ? 4.5 : (msg->ranges[a.index] > 4 ? 4.5 : msg->ranges[a.index]); // Right
    b.value = isnan(msg->ranges[b.index]) ? 4.5 : (msg->ranges[b.index] > 4 ? 4.5 : msg->ranges[b.index]); // Left

    if(a.value > 0.15 && b.value > 0.15){
      if(abs(a.value - b.value) > 0.4){
        if(a.value < 4 && b.value < 4){ // Both are within the range
          // Check if the discontinuity points are between 0 and 180 degrees
          (i > 121 && i < 605) ? count_discontinuity_points = count_discontinuity_points + 2 : false;

          // If the discontinuity point is at right then 1 else it is at left then 0
          if(a.value < b.value){
            a.direction = Ignore;
            b.direction = Ignore;

          } else{
            a.direction = Ignore;
            b.direction = Ignore;

          }

          dis_points.push_back(a);
          dis_points.push_back(b);
          i++;

        } else if(a.value < b.value){ // a is within the range
          (i > 121 && i < 605) ? ++count_discontinuity_points : false;

          a.direction = Right;
          dis_points.push_back(a);

        } else{ // b is within the range
          if(temp_size > 0){
            // Checking for a gap
            // Example: *-----* (* = disPoint | - = count for the gap)
            // i-disPointIndex[last] must be greater than gap_num
            if(abs(i - dis_points[temp_size-1].index) > gap_num){
              (i > 121 && i < 605) ? ++count_discontinuity_points : false;
              b.direction = Left;
              dis_points.push_back(b);
              i++;

            } else{ // Remove that disPoint
              (i > 121 && i < 605) ? --count_discontinuity_points : false;
              dis_points.pop_back();

            }
          } else{
            (i > 121 && i < 605) ? ++count_discontinuity_points : false;
            b.direction = Left;
            dis_points.push_back(b);
            i++;
          }
        }
      }
    }
  }

  //cout << "Middle region count: " << count_discontinuity_points << endl;

  if( free_distance == max_laser_range || current_to_goal_distance - free_distance < 0){
    discontinuity_point t_point;
    t_point.index = laser_align_index;

    if(current_to_goal_distance - free_distance < 0){
      t_point.value = current_to_goal_distance;
    } else{
      t_point.value = free_distance;
    }

    t_point.direction = Ignore;
    t_point.is_t_point = true;
    dis_points.push_back(t_point);
    count_discontinuity_points++;
  }

  if(count_discontinuity_points == 0 && (abs(err_yaw) < PI/36) && state_ != FollowBoundary){
    discontinuity_point a,b;
    a.is_t_point = b.is_t_point = false;
    // Check discontinuity for right side
    a.index = 0;
    b.index = 1;
    a.value = isnan(msg->ranges[a.index]) ? 4 : (msg->ranges[a.index] > 4 ? 4 : msg->ranges[a.index]);
    b.value = isnan(msg->ranges[b.index]) ? 4 : (msg->ranges[b.index] > 4 ? 4 : msg->ranges[b.index]);

    if(a.value > 0.6 && a.value < 4 && b.value > 0.6 && b.value < 4){
      if(abs(a.value - b.value) < 0.2){
        a.direction = Ignore;
        dis_points.push_back(a);
      }
    }

    // Check discontinuity for left side
    a.index = size-1;
    b.index = size-2;
    a.value = isnan(msg->ranges[a.index]) ? 4 : (msg->ranges[a.index] > 4 ? 4 : msg->ranges[a.index]);
    b.value = isnan(msg->ranges[b.index]) ? 4 : (msg->ranges[b.index] > 4 ? 4 : msg->ranges[b.index]);

    if(a.value > 0.6 && a.value < 4 && b.value > 0.6 && b.value < 4){
      if(abs(a.value - b.value) < 0.2){
        a.direction = Ignore;
        dis_points.push_back(a);
      }
    }

    if(dis_points.size() > 0){
      temp_lock = true;
    }
  }

  disPoint_marker.points.clear();

  if(!lockedPoint){

    // Compute and select best discontinuity point
    if(dis_points.size() > dist_detection && !isInitAlign){
      float c_distance;

      for(int i = 0; i < dis_points.size(); i++){

        if(dis_points[i].value > dist_detection || dis_points[i].is_t_point){

          float val = mapRangeFloat(dis_points[i].index, 0, size-1, -laser_angle, laser_angle);
          //float val = (dis_points[i].index - laser_samples/2)*angle_increment;
          // Discontinuity point pose stimation
          float angle = yaw_ + val;
          float temp_angle = 0;
          float temp_dist = 0;
          if(dis_points[i].direction == Right){
            temp_angle = (PI/2);
            temp_dist = 0.5;
          } else if(dis_points[i].direction == Left) {
            temp_angle = -(PI/2);
            temp_dist = 0.5;
          }
          p.x  = laser_position_.x + (dis_points[i].value*cos(angle));
          p.y  = laser_position_.y + (dis_points[i].value*sin(angle));

          cap.x = p.x + (temp_dist*cos(angle + temp_angle));
          cap.y = p.y + (temp_dist*sin(angle + temp_angle));

          disPoint_marker.points.push_back(p);

          current_to_point_distance = getDistance(position_, p);
          point_to_goal_distance = getDistance(p, desired_position_);

          c_distance = 0;

          if(isSensedClosestPointInit){
            c_distance = !dis_points[i].is_t_point ? getDistance(p, sensed_closest_point) : 0;
            if(point_to_goal_distance <= sensed_closest_point.z){
              sensed_closest_point.x = p.x;
              sensed_closest_point.y = p.y;
              sensed_closest_point.z = point_to_goal_distance;
            }
          }
          // Total distance = current position to point + point position to goal + point to closest point distance
          path_distance = current_to_point_distance + point_to_goal_distance + c_distance;

          if(temp_lock){

            if(abs(val) < best_point_yaw){
              closest_point_distance = getDistance(p, desired_position_);
              best_path_distance = path_distance;
              best_point_yaw = abs(val);
              n = dis_points[i].index;
              target = p;
              target.z = point_to_goal_distance;
              //cout << "Temp lock dis point" << endl;
            }

          } else{
            if(path_distance < best_path_distance && point_to_goal_distance < current_to_goal_distance){
              closest_point_distance = getDistance(p, desired_position_);
              best_path_distance = path_distance;
              best_point_yaw = abs(angle);
              n = dis_points[i].index;
              target = cap;
              target.z = point_to_goal_distance;
              //cout << "Go to DisPoint" << endl;
            } /*else{
              target = sensed_closest_point;
              //cout << "Go to Sensed point" << endl;
            }*/
          }
        }
      }

      if(!isSensedClosestPointInit){
        sensed_closest_point = m_point;
        isSensedClosestPointInit = true;
      }

      if(temp_lock && (best_path_distance < 999)){ //&& (closest_point_distance > current_to_goal_distance) && (abs(err_yaw) < PI/6)){
        cout << "Point Locked!!" << endl;
        cout << "X: " << target.x << " | Y: " << target.y << endl;
        cout << "Best point distance: " << best_path_distance << endl;
        cout << "Current goal distance: " << current_to_goal_distance << endl;
        lockedPoint = true;
      }
    }
  } else{
    cout << "Point Locked!!" << endl;
    cout << "X: " << target.x << " | Y: " << target.y << endl;
  }

  if(isInitAlign){
    // Make sure the robot is align
    if(abs(err_yaw) < PI/6)
      isInitAlign = false;

  } else{
    target_point_pub.publish(target);
  }
  //cout << "Discontinuity points: " << disPointIndex.size() << endl;
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

  // If the node is stopped then finish
  while(!isLaserReady && node_state_ == Initializing){
    ros::spinOnce();
  }
  ROS_INFO("Laser ok!");
}

void changeState(int state){
  bug_algorithms::bugSwitch srv;

  count_state_time = 0;
  state_ = state;
  string s = state_desc_[state_];
  state_time = ros::Time::now().toSec() - previous_time.toSec();
  node_state_time = state_time;
  cout << node_name << " - State changed to: " << s << endl;
  //ROS_INFO("%s - State changed to: %s", node_name.c_str(), s.c_str());

  if(state_ == GoToPoint){
    //ROS_INFO("Calling go to point!");
    srv.request.state = Initializing;
    srv_client_go_to_point.call(srv);
    srv.request.state = Stopping;
    srv_client_follow_boundary.call(srv);
  } else if(state_ == GoToPointFollowing){
    //ROS_INFO("Calling follow boundary!");
    srv.request.state = Stopping;
    srv_client_go_to_point.call(srv);
    srv.request.state = Initializing;
    srv_client_follow_boundary.call(srv);
  } else if(state_ == FollowBoundary){
    //ROS_INFO("Calling follow boundary!");
    srv.request.state = Stopping;
    srv_client_go_to_point.call(srv);
    srv.request.state = Initializing;
    srv_client_follow_boundary.call(srv);
  } else if(state_ == Success || state_ == Fail){
    // Publishing the current node state before changing to waiting state
    publishNodeState();
    // Stopping all services
    srv.request.state = Stopping;
    srv_client_go_to_point.call(srv);
    srv.request.state = Stopping;
    srv_client_follow_boundary.call(srv);
    node_state_ = Waiting;
  }
  publishNodeState();
}

void lostObstacleCallback(const std_msgs::Bool::ConstPtr& msg){
  //subscriberNotify(node_name);
  if(state_ == FollowBoundary && msg->data == true){
    ROS_INFO("Obstacle lost!");
    changeState(GoToPoint);
  }
}

void initMarkers(){
  string frame_id = "odom" ;

  // Publishing initial and goal markers
  geometry_msgs::Vector3 scale = geometry_msgs::Vector3();
  std_msgs::ColorRGBA color = std_msgs::ColorRGBA();

  // Only scale.z specifies the height of an uppercase 'A'
  scale.z = 0.5;
  color.a = 1.0;
  color.r = 0.0;
  color.g = 0.0;
  color.b = 0.0;

  start_text_marker = createMarker(frame_id, node_name, 1, visualization_msgs::Marker::TEXT_VIEW_FACING, visualization_msgs::Marker::ADD, initial_position_, scale, color);
  start_text_marker.text = "S(" + to_string_with_precision(round(initial_position_.x, 1), 0) + " , " + to_string_with_precision(round(initial_position_.y, 1), 0) + ")";
  start_text_marker.pose.position.y = start_text_marker.pose.position.y+0.5;
  start_text_marker.pose.position.z = 0.5;
  marker_pub.publish(start_text_marker);
  goal_text_marker = createMarker(frame_id, node_name, 2, visualization_msgs::Marker::TEXT_VIEW_FACING, visualization_msgs::Marker::ADD, desired_position_, scale, color);
  goal_text_marker.text = "G(" + to_string_with_precision(desired_position_.x, 1) + " , " + to_string_with_precision(desired_position_.y, 1) + ")";
  goal_text_marker.pose.position.y = goal_text_marker.pose.position.y+0.5;
  goal_text_marker.pose.position.z = 0.5;
  marker_pub.publish(goal_text_marker);
  total_path_text_marker = createMarker(frame_id, node_name, 8, visualization_msgs::Marker::TEXT_VIEW_FACING, visualization_msgs::Marker::ADD, initial_position_, scale, color);
  total_path_text_marker.text = "P(" + to_string_with_precision(path_length, 3) + ")";
  total_path_text_marker.pose.position.y = start_text_marker.pose.position.y+1;
  total_path_text_marker.pose.position.z = 0.5;
  marker_pub.publish(total_path_text_marker);

  scale.x = 0.3;
  scale.y = 0.3;
  scale.z = 0.1;
  color.a = 1.0;
  color.r = 0.0;
  color.g = 0.0;
  color.b = 1.0;
  // Deleting existing markers
  //visualization_msgs::Marker m = createMarker(frame_id, node_name, 0, visualization_msgs::Marker::CUBE, visualization_msgs::Marker::DELETEALL, initial_position_, scale, color);
  //marker_pub.publish(m);
  start_marker = createMarker(frame_id, node_name, 3, visualization_msgs::Marker::CUBE, visualization_msgs::Marker::ADD, initial_position_, scale, color);
  marker_pub.publish(start_marker);
  color.r = 1.0;
  color.g = 0.0;
  color.b = 0.0;
  goal_marker = createMarker(frame_id, node_name, 4, visualization_msgs::Marker::CUBE, visualization_msgs::Marker::ADD, desired_position_, scale, color);
  marker_pub.publish(goal_marker);

  color.r = 0.5;
  color.g = 1.0;
  color.b = 0.0;
  target_marker = createMarker(frame_id, node_name, 9, visualization_msgs::Marker::CUBE, visualization_msgs::Marker::ADD, desired_position_, scale, color);
  marker_pub.publish(target_marker);

  color.r = 0.8;
  color.g = 0.0;
  color.b = 0.6;
  m_marker = createMarker(frame_id, node_name, 11, visualization_msgs::Marker::CUBE, visualization_msgs::Marker::ADD, desired_position_, scale, color);
  marker_pub.publish(m_marker);

  // For line strip only scale.x is used and it controls the width of the line segments
  scale.x = 0.05;
  color.a = 0.7;
  color.r = 0.0;
  color.g = 1.0;
  color.b = 0.0;
  path_marker = createMarker(frame_id, node_name, 5, visualization_msgs::Marker::LINE_STRIP, visualization_msgs::Marker::ADD, geometry_msgs::Point(), scale, color);
  // Point of reference
  geometry_msgs::Point p = geometry_msgs::Point();
  p.x = 0;
  p.y = 0;
  p.z = 0.05;
  scale.x = 0.1;
  scale.y = 0.1;
  scale.z = 0.1;
  color.a = 1.0;
  color.r = 0.0;
  color.g = 0.0;
  color.b = 1.0;
  path_marker_go_to = createMarker(frame_id, node_name, 6, visualization_msgs::Marker::POINTS, visualization_msgs::Marker::ADD, p, scale, color);
  marker_pub.publish(path_marker_go_to);
  color.r = 1.0;
  color.g = 0.0;
  color.b = 0.0;
  path_marker_follow = createMarker(frame_id, node_name, 7, visualization_msgs::Marker::POINTS, visualization_msgs::Marker::ADD, p, scale, color);
  marker_pub.publish(path_marker_follow);

  p.x = 0;
  p.y = 0;
  p.z = 0.9;
  scale.x = 0.2;
  scale.y = 0.2;
  scale.z = 0.2;
  color.a = 1.0;
  color.r = 1.0;
  color.g = 0.5;
  color.b = 0.0;
  disPoint_marker = createMarker(frame_id, node_name, 10, visualization_msgs::Marker::POINTS, visualization_msgs::Marker::ADD, p, scale, color);

  isPathMarkerReady = true;
}

void initBug(ros::NodeHandle& nh){
  ROS_INFO("Initializing %s Node", node_name.c_str());

  string vel_topic, odom_topic, laser_topic;

  nh.getParam("/simulation", isSimulation);
  nh.getParam("/desired_x", desired_position_.x);
  nh.getParam("/desired_y", desired_position_.y);
  nh.getParam("/velocity", linear_vel_);
  nh.getParam("/reverse", reverseCriterion);

  angular_vel_ = linear_vel_+0.1;

  // Restating the static variables
  leave_point = geometry_msgs::Point();
  target = geometry_msgs::Point();
  target.x = desired_position_.x;
  target.y = desired_position_.y;

  isPoseReady = false;
  isLaserReady = false;
  isHitPointInit = false;
  leaveCondition = false;
  lockedPoint = false;
  checkCondition = 0;
  isSensedClosestPointInit = false;

  // Path length variables
  isPreviousReady = false;
  isInitAlign = true;
  path_length = 0;

  // Marker
  isPathMarkerReady = false;

  if(isSimulation){
    cout << node_name << " | Using GazeboSim topics\n";
    vel_topic = "cmd_vel";
    odom_topic = "odom";
    laser_topic = "p3dx/laser/scan";
  } else{
    cout << node_name << " | Using RosAria topics\n";
    vel_topic = "RosAria/cmd_vel";
    odom_topic = "RosAria/pose";
    laser_topic = "scan";
  }

  // create options for subscriber and pass pointer to our custom queue
  ros::SubscribeOptions ops =
      ros::SubscribeOptions::create<sensor_msgs::LaserScan>(
        laser_topic, // topic name
        1, // queue length
        laserDetectDiscontinuityCallback, // callback
        ros::VoidPtr(), // tracked object, we don't need one thus NULL
        &custom_queue // pointer to callback queue object
        );
  laser_detect_discont_sub = nh.subscribe(ops);

  laser_sub = nh.subscribe(laser_topic, rate_hz, laserCallback);
  laser_align_sub = nh.subscribe(laser_topic, rate_hz, laserAlignCallback);
  lost_obstacle_sub = nh.subscribe("lost_obstacle", rate_hz, lostObstacleCallback);
  odom_sub = nh.subscribe(odom_topic, rate_hz, odomCallback);
  node_state_sub = nh.subscribe("bugServer/bugNodeState", rate_hz, nodeStateCallback);

  ROS_INFO("Waiting for goToPoint service");
  ros::service::waitForService("goToPointAdvancedSwitch");
  ROS_INFO("Waiting for followBoundaryAdvancedSwitch service");
  ros::service::waitForService("followBoundaryAdvancedSwitch");
  ROS_INFO("Waiting for change hand service");
  ros::service::waitForService("changeHand");

  srv_client_go_to_point = nh.serviceClient<bug_algorithms::bugSwitch>("goToPointAdvancedSwitch", true);
  srv_client_follow_boundary = nh.serviceClient<bug_algorithms::bugSwitch>("followBoundaryAdvancedSwitch", true);
  srv_client_change_hand = nh.serviceClient<std_srvs::SetBool>("changeHand", true);

  waitPose();
  waitLaser();

  initial_position_ = position_;
  closest_point = position_;
  sensed_closest_point = geometry_msgs::Point();
  m_point = position_;
  m_point.z = 0;

  initial_to_goal_distance = getDistance(initial_position_, desired_position_);
  current_to_goal_distance = initial_to_goal_distance;
  best_distance = current_to_goal_distance;
  reach_distance = current_to_goal_distance;
  followed_distance = current_to_goal_distance;

  target.z = current_to_goal_distance;

  initMarkers();

  // Subscribers are ready
  areSubsDown = false;

  // Check if the current state is Initializing for the triggered Stopping case
  if(node_state_ == Initializing){
    // Changing node state to Executing
    node_state_ = Executing;

    // Initialize going to the point
    previous_time = ros::Time::now();
    node_state_time = 0;
    changeState(GoToPoint);
  }
}

void pauseBug(bool isPause){
  bug_algorithms::bugSwitch srv;
  publishNodeState();
  // Stopping all services
  if(isPause){
    srv.request.state = Pause;
    srv_client_go_to_point.call(srv);
    srv_client_follow_boundary.call(srv);
  } else{
    if(state_ == GoToPoint){
      srv.request.state = Executing;
      srv_client_go_to_point.call(srv);
    } else{
      srv.request.state = Executing;
      srv_client_follow_boundary.call(srv);
    }
  }
}

int isFreePath(geometry_msgs::Point desired, float tol){
  float desired_yaw = atan2(desired.y - position_.y, desired.x - position_.x);
  float err_yaw = normalizeAngle(desired_yaw - yaw_);

  //ROS_INFO("Err_yaw: %f",radians2degrees(err_yaw));
  //ROS_INFO("X: %f | Y: %f", desired.x, desired.y);

  if(abs(err_yaw) < (PI/1.5)){
    // Check if there is a free path
    if((abs(err_yaw) < (PI/36)) && regions_["front_left"] > dist_detection+tol
       && regions_["front"] > dist_detection+(tol-0.05) && regions_["front_right"] > dist_detection+tol){
      //cout << "Less than 5 deg" << endl;
      return 1;

    } else if(err_yaw > 0 && (abs(err_yaw) > (PI/36)) && (abs(err_yaw) < (PI/2))
              && regions_["left"] > dist_detection+0.1 && regions_["front_left"] > dist_detection+tol){
      //cout << "Between 10 and 90 - to the left" << endl;
      return 1;

    } else if(err_yaw < 0 && (abs(err_yaw) > (PI/36)) && (abs(err_yaw) < (PI/2))
              && regions_["right"] > dist_detection+0.1 && regions_["front_right"] > dist_detection+tol){
      //cout << "Between 10 and 90 - to the right" << endl;
      return 1;

    } else if((abs(err_yaw) > (PI/2)) && (abs(err_yaw) < (PI/1.5))){
      if(err_yaw > 0 && regions_["left"] > dist_detection+0.2){
        //ROS_INFO("Between 90 and 120 - to the left");
        return 1;

      } else if(err_yaw < 0 && regions_["right"] > dist_detection+0.2){
        //ROS_INFO("Between 90 and 120 - to the left");
        return 1;
      }
    }
  } else{
    // Ignore
    return 2;
  }

  // There is not a free path
  return 0;
}

void chooseDirection(){
  if((regions_["left"] < 1 || regions_["right"] < 1)){
    if(regions_["left"] < regions_["right"]){
      hand_tracking = 1; // Robot is using its left hand
      cout << "Choose direction: Right ( " << regions_["left"] << " | " << regions_["right"] << " )" << endl;
    } else{
      hand_tracking = 2; // Robot is using its right hand
      cout << "Choose direction: Left ( " << regions_["left"] << " | " << regions_["right"] << " )" << endl;
    }

  } else if((regions_["front_left"]) < (regions_["front_right"])){
    hand_tracking = 1; // Robot is using its left hand
    cout << "Choose direction: Right ( " << (regions_["left"]+regions_["front_left"]) << " | " << (regions_["right"]+regions_["front_right"]) << " )" << endl;
  } else{
    hand_tracking = 2; // Robot is using its right hand
    cout << "Choose direction: Left ( " << (regions_["left"]+regions_["front_left"]) << " | " << (regions_["right"]+regions_["front_right"]) << " )" << endl;
  }
}

void bugConditions(){

  bool isCurrentClose = false;
  bool isReachDistanceClose = false;
  geometry_msgs::Point p = geometry_msgs::Point();

  // Updating tangent bug distance
  if(free_distance > 0){
    float val = mapRangeFloat(laser_align_index, 0, laser_samples-1, -laser_angle, laser_angle);
    //float val = (laser_align_index - laser_samples/2)*angle_increment;
    // Pose stimation
    float temp_free_dist = current_to_goal_distance - free_distance < 0 ? current_to_goal_distance : free_distance;
    p.x  = laser_position_.x + (temp_free_dist*cos(yaw_ + val));
    p.y  = laser_position_.y + (temp_free_dist*sin(yaw_ + val));
    reach_distance = getDistance(p, desired_position_);

    if(reach_distance+0.2 < followed_distance){
      // if the free distance is less than 0 then set 0 to followed_distance
      cout << "Reach: " << reach_distance << " | Followed: " << followed_distance <<  endl;
      m_point = p;
      m_point.z = reach_distance;
      m_marker.pose.position.x = p.x;
      m_marker.pose.position.y = p.y;
      m_marker.pose.position.z = 0.8;
      followed_distance = reach_distance;
      isReachDistanceClose = true;
    }
  } else {
    // If any laser ray is pointing to goal then change the current checkCondition to 0,
    // this allows to check the condition 2 (current_distance > best_distance)
    checkCondition = 0;
  }

  current_to_goal_distance = getDistance(position_, desired_position_);
  desired_yaw = atan2(desired_position_.y - position_.y, desired_position_.x - position_.x);
  err_yaw = normalizeAngle(desired_yaw - yaw_);

  float t_desired_yaw = atan2(target.y - position_.y, target.x - position_.x);
  float t_err_yaw = normalizeAngle(t_desired_yaw - yaw_);

  if(current_to_goal_distance+0.1 < best_distance){
    closest_point = position_;
    best_distance = current_to_goal_distance;
    isCurrentClose = true;
    count_same_point = 0;

    /*if(state_ == FollowBoundary){
      count_state_time = 11;
    }*/

  }

  if(state_ == GoToPoint || state_ == GoToPointFollowing){
    if(state_ == GoToPoint){
      if(isFreePath(target, 0) == 0 && abs(t_err_yaw) < PI/30){
        if(count_state_time > 2){
          chooseDirection();
          changeState(GoToPointFollowing);
          return;
        }
      }
    }

    if(state_ == GoToPointFollowing){

      if(count_state_time > 1){
        if(!leaveCondition){

          if(!lockedPoint && regions_["front"] > dist_detection+0.2 && hand["right"] < 1 && hand["left"] < 1 &&
             ((laser_align_index > 66 && laser_align_index < 176) ||
              (laser_align_index > laser_samples-176 && laser_align_index < laser_samples-66))){
            int opt_hand;
            // virtual hands points stimation
            geometry_msgs::Point p_right, p_left = laser_position_;
            p_right.x  = laser_position_.x + round(hand["right"]*cos(yaw_ + right_hand_angle), 2);
            p_right.y  = laser_position_.y + round(hand["right"]*sin(yaw_ + right_hand_angle), 2);
            p_left.x  = laser_position_.x + round(hand["left"]*cos(yaw_ + left_hand_angle), 2);
            p_left.y  = laser_position_.y + round(hand["left"]*sin(yaw_ + left_hand_angle), 2);

            if(err_yaw > 0){
              cout << "Using left hand" << endl;
              opt_hand = 1;
            } else{
              cout << "Using right hand" << endl;
              opt_hand = 2;
            }

            /*if(getDistance(p_left, desired_position_) < getDistance(p_right, desired_position_)){
              cout << "Using left hand" << endl;
              opt_hand = 1;
            } else{
              cout << "Using right hand" << endl;
              opt_hand = 2;
            }*/

            if(hand_tracking == 0 || hand_tracking != opt_hand){
              hand_tracking = opt_hand;
              cout << "Using: " << hand_tracking << endl;
              std_srvs::SetBool r = std_srvs::SetBool();
              // if hand_tracking is 1 this means that robot is using its left and therefore following direction is right
              // on the oder hand, robot is using its right hand and following direction is left
              r.request.data = hand_tracking == 1 ? true : false;
              srv_client_change_hand.call(r);
            }
          }


          if(free_distance > 0.5 && (sensed_closest_point.z < current_to_goal_distance) ){//&& (abs(err_yaw) < PI/2)
            //cout << "Leave condition 2: " << free_distance << endl;
            leaveCondition = true;
            count_tolerance = 0;
            return;
          } else if((current_to_goal_distance - free_distance) <= 0){
            cout << "Leave condition 1: " << current_to_goal_distance - free_distance << endl;
            cout << "Free: " << free_distance << endl;
            leaveCondition = true;
            count_tolerance = 0;
            return;
          }

        } else{
          leaveCondition = false;
          if(isFreePath(target, 0.3) == 1){
            //cout << "The problem is here" << endl;
            changeState(GoToPoint);
            return;
          } else{
            //cout << "No free path" << endl;
          }
        }

        if(lockedPoint){
          if(isFreePath(target, 0) == 1){
            //cout << "The problem is here" << endl;
            changeState(GoToPoint);
            return;
          } else{
            //cout << "No free path" << endl;
          }
        }

      }
    }

    if(regions_["front_left"] < dist_detection  || regions_["front_right"] < dist_detection
       || regions_["left"] < dist_detection+0.1 || regions_["right"] < dist_detection+0.1){
      if(lockedPoint && isOnPointRange(position_, target, 0.5)){
        cout << "Condition 1" << endl;
        lockedPoint = false;
        leaveCondition = false;
        changeState(FollowBoundary);
        return;
      }

      // For this condition is necessary that the current distance to goal is greater than the best distance,
      // when the reached distance is closest to goal, then ignore this condition
      if(current_to_goal_distance-0.2 > best_distance && checkCondition != 2 && checkCondition != 3){
        if(!lockedPoint && reach_distance > followed_distance+0.2 && followed_distance > 0.1){
          cout << "Condition 3" << endl;
          cout << "Reach: " << reach_distance << " | Followed: " << followed_distance <<  endl;
          lockedPoint = false;
          leaveCondition = false;
          changeState(FollowBoundary);
          return;
        } else if(!lockedPoint){
          cout << "Condition 2" << endl;
          lockedPoint = false;
          leaveCondition = false;
          changeState(FollowBoundary);
          return;
        }
      }
    }

    /*if(state_ == GoToPointFollowing && lockedPoint && !isOnPointRange(position_, target, 0.4)){
      if(abs(err_yaw) < PI/2){
        cout << "Here is the problem lock point" << endl;
        changeState(GoToPoint);
        return;
      } else{
        changeState(FollowBoundary);
        return;
      }
    }*/

    if(getDistance(position_, desired_position_) < 0.3){
      changeState(Success);
      return;
    }
  } else if(state_ == FollowBoundary){
    lockedPoint = false;

    if(!leaveCondition) {
      leaveCondition = false;

      if(isCurrentClose && isReachDistanceClose){
        checkCondition = 1;
        cout << "C-1 ok" << endl;
      } else if(isReachDistanceClose){
        checkCondition = 2;
        cout << "C-2 ok" << endl;
      } else if((current_to_goal_distance < 4) && (followed_distance < 0.5) && isFreePath(desired_position_, 0.1) == 1){
        checkCondition = 3;
        cout << "C-3 ok" << endl;
      } else {
        checkCondition = 0;
        //cout << "C-0 Not ok : " << followed_distance << endl;
      }

      if(checkCondition != 0){
        cout << "Follow checking conditions..." << endl;
        if(free_distance > 0.5){
          cout << "Leave condition 2: " << current_to_goal_distance - free_distance << endl;
          cout << "Free: " << free_distance << endl;
          leaveCondition = true;
        }
      } else if((current_to_goal_distance - free_distance) <= 0){
        if(isFreePath(desired_position_, 0.1) == 1){
          cout << "Leave condition 1: " << current_to_goal_distance - free_distance << endl;
          cout << "Free: " << free_distance << endl;
          leaveCondition = true;
        }
      }

    } else{
      leaveCondition = false;
      if(isFreePath(desired_position_, 0.1) == 0){
        changeState(GoToPointFollowing);
      } else{
        changeState(GoToPoint);
      }
    }

    // If the free distance from the current position to goal is gratter than rangeMax (4m) then set freeDistance = rangeMax

    if(count_state_time > 10 || isCurrentClose){

      if(isCurrentClose && isReachDistanceClose){
        changeState(GoToPointFollowing);

      } else if(getDistance(position_, closest_point) < 0.3){
        if(reverseCriterion && count_same_point == 2){
          cout << "Count for hit point is: " << count_same_point << endl;
          changeState(Fail);
          return;
        } else if(!reverseCriterion && count_same_point == 1){
          cout << "Count for hit point is: " << count_same_point << endl;
          changeState(Fail);
          return;
        }
        //cout << "Count for hit point is: " << count_same_point << endl;
        count_state_time = 0;
        count_same_point++;

      }
    }

    if(getDistance(position_, desired_position_) < 0.3){
      changeState(Success);
      return;
    }

  }
}

void shutDownSubscribers(){
  cout << node_name << ": shutting down subscribers" << endl;
  laser_sub.shutdown();
  laser_align_sub.shutdown();
  laser_detect_discont_sub.shutdown();
  odom_sub.shutdown();
  lost_obstacle_sub.shutdown();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh;
  // While the node is in Pause, it only will call the pauseBug function once.
  bool pauseBand = true;
  int count = 0;

  nodeStateGlobal.algorithm = algorithm_id;

  node_state_pub = nh.advertise<bug_algorithms::nodeState>("bugServer/bugNodeStateInternal", rate_hz);
  target_point_pub = nh.advertise<geometry_msgs::Point>("bugServer/targetPoint", rate_hz);
  algorithm_state_pub = nh.advertise<bug_algorithms::algorithmState>("bugServer/algorithmState", rate_hz);
  marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", rate_hz);

  ros::ServiceServer service = nh.advertiseService("bugServer/tangentBugSwitch", bugSwitch);

  ros::Rate rate(rate_hz);
  while(!ros::isShuttingDown()){
    if(node_state_ == Waiting){
      if(!areSubsDown){
        shutDownSubscribers();
        areSubsDown = true;
      }

    } else if(node_state_ == Initializing){
      initBug(nh);

    } else if(node_state_ == Executing){
      if(!pauseBand){
        pauseBug(false);
      }
      pauseBand = true;
      bugConditions();

    } else if(node_state_ == Pause){
      if(pauseBand){
        ROS_INFO("%s paused", node_name.c_str());
        pauseBug(true);
        pauseBand = false;
      }

    } else if(node_state_ == Stopping){
      changeState(Fail);
    }
    //publishNodeState();

    ros::spinOnce();
    rate.sleep();

    ++count_loop;

    if(count_loop == 10 || count_loop == rate_hz){
      custom_queue.callAvailable();
      count = 0;
    }

    if(count_loop == rate_hz){
      count_state_time++;
      count_tolerance++;
      count_loop = 0;
      pubMarker = true;
      publishPathMarkers();
      /*++count;
      if(count > 0){
        custom_queue.callAvailable();
        count = 0;
      }*/
    }
  }

  return 0;
}
