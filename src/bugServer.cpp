#include "ros/ros.h"
#include <iostream>
#include "vector"
#include "utils.h"
#include "tf/tf.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_srvs/SetBool.h"
#include "bug_algorithms/bugService.h"
#include "bug_algorithms/bugSwitch.h"
#include "bug_algorithms/nodeState.h"

enum BugAlgorithms {Bug0, Bug1, Bug2, DistBug, IntelligentBug, IBug, TangentBug, PointBug};
enum NodeStates {Waiting, Initializing, Executing, Pause, Stopping};

static bug_algorithms::nodeState nodeGlobalState;

static map<int, string> node_state_desc = {
  {Waiting, "Waiting"},
  {Initializing, "Initializing"},
  {Executing, "Executing"},
  {Pause, "Pause"},
  {Stopping, "Stopping"}
};

static map<int, string> algorithm = {
  {Bug0, "bug0Switch"},
  {Bug1, "bug1Switch"},
  {Bug2, "bug2Switch"},
  {DistBug, "distBugSwitch"},
  {IntelligentBug, "intelligentBugSwitch"},
  {IBug, "iBugSwitch"},
  {TangentBug, "tangentBugSwitch"},
  {PointBug, "pointBugSwitch"},

};

ros::Publisher node_state_pub;
ros::Subscriber node_state_sub;

void setDefaultNodeGlobalState(){
  nodeGlobalState.algorithm = -1;
  nodeGlobalState.bug_state = -1;
  nodeGlobalState.bug_state_desc = "";
  nodeGlobalState.node_state = Waiting;
  nodeGlobalState.node_state_desc = node_state_desc[Waiting];
}

void nodeStateCallback(const bug_algorithms::nodeState::ConstPtr& msg)
{
  nodeGlobalState.algorithm = msg->algorithm;
  nodeGlobalState.bug_state = msg->bug_state;
  nodeGlobalState.bug_state_desc = msg->bug_state_desc;
  nodeGlobalState.bug_state_time = msg->bug_state_time;
  nodeGlobalState.node_state = msg->node_state;
  nodeGlobalState.node_state_desc = msg->node_state_desc;
  nodeGlobalState.node_state_time = msg->node_state_time;

  cout << "Node state: " << nodeGlobalState.node_state_desc << endl;
}

bool bugExecSwitch(bug_algorithms::bugSwitchRequest& request, bug_algorithms::bugSwitchResponse& response){
  bug_algorithms::bugSwitch srv;
  srv.request = request;

  string s = "/bugServer/" + algorithm[request.algorithm];
  string desc = node_state_desc[request.state];

  if(ros::service::call(s, srv)){
    ROS_INFO("%s: %s", s.c_str(), desc.c_str());
    response = srv.response;
  } else{
    ROS_INFO("Error while calling service");
  }

  return true;
}

bool initializeBugAlgorithm(bug_algorithms::bugServiceRequest& request, bug_algorithms::bugServiceResponse& response){

  // Setting global parameters
  ros::param::set("/algorithm", request.algorithm);
  ros::param::set("/velocity", request.velocity);
  ros::param::set("/initial_x", request.initial_x);
  ros::param::set("/initial_y", request.initial_y);
  ros::param::set("/desired_x", request.desired_x);
  ros::param::set("/desired_y", request.desired_y);
  ros::param::set("/simulation", bool(request.simulation));
  ros::param::set("/reverse", bool(request.reverse));
  ros::param::set("/choose", bool(request.choose));

  ROS_INFO("Algorith: %i \nVelocity: %f \nInitial position: %f | %f "
           "\nDesired position: %f | %f \nSimulation: %i "
           "\nReverse: %i \nChoose: %i",
           request.algorithm,
           request.velocity,
           request.initial_x,
           request.initial_y,
           request.desired_x,
           request.desired_y,
           request.simulation,
           request.reverse,
           request.choose);

  bug_algorithms::bugSwitch srv;
  srv.request.state = Initializing;
  string s = "/bugServer/" + algorithm[request.algorithm];

  if(ros::service::call(s, srv)){
    ROS_INFO("Calling: %s", s.c_str());
    srv.response.message = "Success on calling " + s;
  } else {
    ROS_INFO("Error calling service");
    srv.response.message = "A problem has ocurred on the server!";
  }

  if(srv.response.success){
    nodeGlobalState.algorithm = request.algorithm;
    nodeGlobalState.node_state = Initializing;
    nodeGlobalState.node_state_desc = node_state_desc[Initializing];
    nodeGlobalState.node_state_time = 0;
    nodeGlobalState.bug_state = 0;
    nodeGlobalState.bug_state_desc = "";
    nodeGlobalState.bug_state_time = 0;

    node_state_pub.publish(nodeGlobalState);
    ros::spinOnce();
  }

  s = srv.response.message;
  ROS_INFO("Message: %s", s.c_str());

  response.message = srv.response.message;
  response.success = srv.response.success;

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "bugServer");

  ros::NodeHandle nh("bugServer");

  int rate_hz = 5;

  node_state_pub = nh.advertise<bug_algorithms::nodeState>("bugNodeState", rate_hz);
  node_state_sub = nh.subscribe("bugNodeStateInternal", 20, nodeStateCallback);

  ros::ServiceServer service = nh.advertiseService("initBugAlg", initializeBugAlgorithm);
  ros::ServiceServer service2 = nh.advertiseService("bugNodeStateSwitch", bugExecSwitch);

  // Init node global state
  setDefaultNodeGlobalState();
  node_state_pub.publish(nodeGlobalState);

  ros::Rate rate(rate_hz);
  while(!ros::isShuttingDown()){

    node_state_pub.publish(nodeGlobalState);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
