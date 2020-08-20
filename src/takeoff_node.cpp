#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
// #include <hector_uav_msgs/EnableMotors.h>
#include <actionlib/client/simple_action_client.h>
#include <cmath>
#include "two_drones/hover_node.h"
// #include <hector_uav_msgs/PoseAction.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

// double roll, pitch, yaw;
// double dist_x, dist_y, dist_theta;
// ros::Publisher vel;
// ros::Subscriber subOdom;

std::string POSE_NAME = "action/pose";
// Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state,
            const hector_uav_msgs::PoseResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
//   ROS_INFO("Answer: %s", result->status);
  ros::shutdown();
}

// Called once when the goal becomes active
void activeCb()
{
  ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal
void feedbackCb(const hector_uav_msgs::PoseFeedbackConstPtr& feedback)
{
  ROS_INFO("Got Feedback of action %f %f %f", feedback->current_pose.pose.position.x, feedback->current_pose.pose.position.y, feedback->current_pose.pose.position.z);
}

// void odom_callback(const nav_msgs::OdometryConstPtr& msg) {
//   nav_msgs::Odometry odom = *msg;
  // tf::Quaternion q(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
  // tf::Matrix3x3 m(q);
  // m.getRPY(roll, pitch, yaw);
  // dist_x = odom.pose.pose.position.x - 208599.559107;
  // dist_y = odom.pose.pose.position.y - 960441.484417;
  // dist_theta = atan2(dist_y, dist_x);
// }

int main(int argc, char **argv)
{
    // %Tag(INIT)%
    ros::init(argc, argv, "takeoff_node");
    // %EndTag(INIT)%

    std::string ns = ros::this_node::getNamespace();
    if(ns.length() > 1) {
      ns = ns.substr(2, ns.length());
      ROS_INFO("%s", ns.c_str());
      ns += "/";
    } else {
      ns="";
    }
    ROS_INFO("Pose name: %s", (ns + POSE_NAME).c_str());
    float Y_POS = 1.0;
    float X_POS = 0.0;
    float Z_POS = 4.0;
    switch(argc) {
      case 1:
        break;
      case 2:
        X_POS = atof(argv[1]);
        break;
      case 3:
        X_POS = atof(argv[1]);
        Y_POS = atof(argv[2]);
        break;
      case 4:
        X_POS = atof(argv[1]);
        Y_POS = atof(argv[2]);
        Z_POS = atof(argv[3]);
        break;
      default:
        ROS_INFO("Invalid entry");
        break;
    }

    // %Tag(NODEHANDLE)%
    ros::NodeHandle n;
    // %EndTag(NODEHANDLE)%

    // create the action client
    // true causes the client to spin its own thread
    // actionlib::SimpleActionClient<hector_uav_msgs::TakeoffAction> takeoff_drone1_("action/takeoff", true);
    actionlib::SimpleActionClient<hector_uav_msgs::PoseAction> pose_("/" + ns + POSE_NAME, true);
    // subOdom = n.subscribe<nav_msgs::Odometry>("geonav_odom", 10, &odom_callback);
    // vel = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    hector_uav_msgs::PoseActionFeedback pose_feedback;
    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    pose_.waitForServer();
    ROS_INFO("Action server started, sending goal.");
    // send a goal to the action
    hector_uav_msgs::PoseGoal pose_goal;
    ROS_INFO("Current pose %f", pose_feedback.feedback.current_pose.pose.position.z);
    pose_goal.target_pose = pose_feedback.feedback.current_pose;
    pose_goal.target_pose.pose.position.x = X_POS;
    pose_goal.target_pose.pose.position.y = Y_POS;
    pose_goal.target_pose.pose.position.z = Z_POS;
    ROS_INFO("%s", (ns + std::string("world")).c_str());
    pose_goal.target_pose.header.frame_id = ns + "world";
    pose_.sendGoal(pose_goal, &doneCb, &activeCb, &feedbackCb);
    pose_.waitForResult(ros::Duration(30.0));

    if (pose_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      actionlib::SimpleClientGoalState state = pose_.getState();
      ROS_INFO("Action finished: %s",state.toString().c_str());
      
    }
    else
        ROS_INFO("Action did not finish before the time out."); 

    // geometry_msgs::Twist speed;
    // if(abs(dist_theta - yaw) > 0.1) {
    //   speed.linear.x = 0.0;
    //   speed.angular.z = 0.3;
    //   vel.publish(speed);
    // }
  
    return 0;
}
