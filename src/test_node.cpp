#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Joy.h>
// #include <geometry_msgs/Twist.h>
#include <sstream>
// #include <geodesy/utm.h>
// #include <hector_uav_msgs/EnableMotors.h>
#include <geographic_msgs/GeoPoint.h>
#include <geographic_msgs/GeoPose.h>
#include <cmath>
#include "two_drones/hover_node.h"
#include <geometry_msgs/Vector3.h>
#include <tf/transform_datatypes.h>
#include <airsim_ros_pkgs/VelCmd.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

Drone_Mission drone1;

geometry_msgs::Vector3 alt_output1;
// ros::Publisher altitude1;

geometry_msgs::Vector3 waypoint;

int main(int argc, char **argv)
{
  waypoint.x = -102894.25;
  waypoint.y = -4562699.25;
  waypoint.z = 0.0;

// %Tag(INIT)%
  ros::init(argc, argv, "test_node");
// %EndTag(INIT)%

// %Tag(NODEHANDLE)%
  ros::NodeHandle n;
// %EndTag(NODEHANDLE)%

drone1 = Drone_Mission("drone1", n);

drone1.fix = n.subscribe("airsim_node/drone1/global_gps", 10, &drone1_gps_callback);

drone1.imu = n.subscribe("/airsim_node/drone1/imu/Imu", 10, &Drone_Mission::imuCallback, &drone1);

drone1.magnetic = n.subscribe("/airsim_node/drone1/magnetometer/Magnetometer", 10, &Drone_Mission::magneticCallback, &drone1);

drone1.odom = n.subscribe("/airsim_node/drone1/odom", 10, &Drone_Mission::odomCallback, &drone1);

// %Tag(PUBLISHER)%
drone1.move_drone = n.advertise<airsim_ros_pkgs::VelCmd>("/airsim_node/drone1/vel_cmd_body_frame", 1);

// drone1.move_drone2 = n.advertise<airsim_ros_pkgs::VelCmd>("/airsim_node/SimpleFlight/vel_cmd_world_frame", 1);

// altitude1 = n.advertise<geometry_msgs::Vector3>("drone1/altitude", 1);

// %Tag(LOOP_RATE)%
  ros::Rate loop_rate(10);
// %EndTag(LOOP_RATE)%

  drone1.time = ros::Time::now().toSec();

// %Tag(ROS_OK)%
  static int count = 0;
  while (ros::ok())
  {
    ROS_INFO("Current state: %d Current time: %f", drone1.state, (ros::Time::now().toSec() - drone1.time));
    if((ros::Time::now().toSec() - drone1.time) > 10 && (ros::Time::now().toSec() - drone1.time) < 30 && count < 2) {
      drone1.state = 1;
      ++count;
      ROS_INFO("Trying to control now");
    }  

// %Tag(SPINONCE)%
    ros::spinOnce();
// %EndTag(SPINONCE)%

// %Tag(RATE_SLEEP)%
    loop_rate.sleep();
// %EndTag(RATE_SLEEP)%
  }

  return 0;
}


double get_yaw_from_quat_msg(const geometry_msgs::Quaternion& quat_msg)
{
  tf2::Quaternion quat_tf;
  double roll, pitch, yaw;
  tf2::fromMsg(quat_msg, quat_tf);
  tf2::Matrix3x3(quat_tf).getRPY(roll, pitch, yaw);
  return yaw;
}


void step(Drone_Mission &drone) {
  // ros::Rate r(0.2);
    // r.sleep();
  if(drone.state == 1 && drone.finished != true) {
    if(drone.current_gps.altitude - drone.mean_start_gps < 3.98) {
      drone.motor_msg.twist.linear.z = -0.3;
      drone.motor_msg.twist.angular.z = 0.0;
      drone.move_drone.publish(drone.motor_msg);
    } else if(drone.current_gps.altitude - drone.mean_start_gps > 4.02){
      drone.motor_msg.twist.linear.z = 0.2;
      drone.motor_msg.twist.angular.z = 0.0;
      drone.move_drone.publish(drone.motor_msg);  
    }
    // if(drone.subOdom.pose.pose.position.x - drone.mean_start_eas < drone.target.x)
    //   drone.motor_msg.twist.linear.x = 0.1;
    // else
    //   drone.motor_msg.twist.linear.x = -0.1;
    // if(drone.subOdom.pose.pose.position.y - drone.mean_start_nor < drone.target.y)
    //   drone.motor_msg.twist.linear.y = 0.1;
    // else
    //   drone.motor_msg.twist.linear.y = -0.1;  
    if(ros::Time::now().toSec() - drone.time > 40) {
      drone.finished = true;
    }
    ROS_INFO("Orig: %f  Modified: %f", drone.current_gps.altitude, drone.current_gps.altitude - drone.mean_start_gps);
  }
  if(drone.state == 2 && drone.finished != true) {
    if(drone.current_gps.altitude - drone.mean_start_gps > 0.05){
      drone.motor_msg.twist.linear.x = 0.0;
      drone.motor_msg.twist.linear.z = 0.15;
      drone.motor_msg.twist.angular.z = 0.0;
      drone.move_drone.publish(drone.motor_msg);  
    } else {
      drone.finished = true;
    }
    if(ros::Time::now().toSec() - drone.time > 90) {
      drone.finished = true;
    }
  }
  if(drone.state == 3 && drone.finished != true) {
    drone.motor_msg.twist.linear.x = 1.0;
    drone.motor_msg.twist.linear.z = 0.0;
    drone.motor_msg.twist.angular.z = 0.0;
    drone.move_drone.publish(drone.motor_msg);
    if(ros::Time::now().toSec() - drone.time > 60) {
      drone.finished = true;
    }
  }
}


void Drone_Mission::magneticCallback(const sensor_msgs::MagneticField::ConstPtr& msg) {
  current_magnetic = *msg;
}

void Drone_Mission::imuCallback(const sensor_msgs::ImuConstPtr& msg) {
  sensor_msgs::Imu imu = *msg;
  tf::Quaternion q(imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
  // ROS_INFO("%s: Roll: %f, Pitch: %f, Yaw: %f", name.c_str(), roll, pitch, yaw);
  // test.x = yaw;
  // tester.publish(test);
}


void Drone_Mission::odomCallback(const nav_msgs::OdometryConstPtr& msg) {
  subOdom = *msg;
}


void Drone_Mission::odomLocalCallback(const nav_msgs::OdometryConstPtr& msg) {
  curr_odom_ = *msg;
  curr_position_.x = curr_odom_.pose.pose.position.x;
  curr_position_.y = curr_odom_.pose.pose.position.y;
  curr_position_.z = curr_odom_.pose.pose.position.z;
  curr_position_.yaw = get_yaw_from_quat_msg(curr_odom_.pose.pose.orientation);
}


void drone1_gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
  drone1.current_gps = *msg;
  // utm_drone1.publish(current_drone1_utm);
  static int count_1 = 0;
  static double start_alt_1 = 0;
  static double start_magx = 0;
  if(ros::Time::now().toSec() - drone1.time < 10) {
    count_1 += 1;
    start_alt_1 += drone1.current_gps.altitude;
    drone1.mean_start_gps = start_alt_1/count_1;
    ROS_INFO("Mean drone1 altitude %f %f %d", drone1.mean_start_gps, start_alt_1, count_1);
  }
  // alt_output1.x = drone1.current_gps.altitude;
  // alt_output1.y = drone1.current_gps.altitude - drone1.mean_start_gps;
  // altitude1.publish(alt_output1);
  switch(drone1.state) {
    case 0: 
      break;
    case 1:
      if(drone1.finished == false) {
        step(drone1);
      } else {
        drone1.state = 3;
        drone1.direction = 0.3;
        drone1.finished = false;
        ROS_INFO("Done Hovering");
      }
      break;
    case 2:
      if(drone1.finished == false) {
        step(drone1);
      } else {
        drone1.state = 0;
        drone1.finished = false;
        ROS_INFO("Done Landing");
      }
      break;
    case 3:
      if(drone1.finished == false) {
        step(drone1);
      } else {
        drone1.state = 2;
        drone1.finished = false;
        ROS_INFO("Done Moving");
      }
      break;
    default:
      ROS_INFO("Invalid State");
      break;
  }
}