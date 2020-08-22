#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <sstream>
#include <geographic_msgs/GeoPoint.h>
#include <geographic_msgs/GeoPose.h>
#include <cmath>
#include "two_drones/hover_node.h"
#include <geometry_msgs/Vector3.h>
#include <tf/transform_datatypes.h>
#include <airsim_ros_pkgs/VelCmd.h>
#include <math_common.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

Drone_Mission drone1("drone1");
Drone_Mission drone2("drone2");

float globalTime = 0;
float globalStartTime = 0;
bool counter = false;

geometry_msgs::Vector3 waypoint;

int main(int argc, char **argv)
{

// %Tag(INIT)%
  ros::init(argc, argv, "hover_node");
// %EndTag(INIT)%

// %Tag(NODEHANDLE)%
  ros::NodeHandle n;
// %EndTag(NODEHANDLE)%

ros::AsyncSpinner spinner(2);

spinner.start();

drone1.fix = n.subscribe("airsim_node/SimpleFlight/global_gps", 10, &drone1_gps_callback);
drone2.fix = n.subscribe("airsim_node/SimpleFlight2/global_gps", 10, &drone2_gps_callback);

drone1.magnetic = n.subscribe("/airsim_node/SimpleFlight/magnetometer/Magnetometer", 10, &Drone_Mission::magneticCallback, &drone1);
drone2.magnetic = n.subscribe("/airsim_node/SimpleFlight2/magnetometer/Magnetometer", 10, &Drone_Mission::magneticCallback, &drone2);

drone1.imu = n.subscribe("/airsim_node/SimpleFlight/imu/Imu", 10, &Drone_Mission::imuCallback, &drone1);
drone2.imu = n.subscribe("/airsim_node/SimpleFlight2/imu/Imu", 10, &Drone_Mission::imuCallback, &drone2);

drone1.odom = n.subscribe("/airsim_node/SimpleFlight/odom", 10, &Drone_Mission::odomCallback, &drone1);
drone2.odom = n.subscribe("/airsim_node/SimpleFlight2/odom", 10, &Drone_Mission::odomCallback, &drone2);

drone1.odom_local = n.subscribe("/airsim_node/SimpleFlight/odom_local_ned", 10, &Drone_Mission::odomLocalCallback, &drone1);
drone2.odom_local = n.subscribe("/airsim_node/SimpleFlight2/odom_local_ned", 10, &Drone_Mission::odomLocalCallback, &drone2);

// %Tag(PUBLISHER)%
drone1.move_drone = n.advertise<airsim_ros_pkgs::VelCmd>("/airsim_node/SimpleFlight/vel_cmd_body_frame", 1);
drone2.move_drone = n.advertise<airsim_ros_pkgs::VelCmd>("/airsim_node/SimpleFlight2/vel_cmd_body_frame", 1);

drone1.take_off = n.serviceClient<airsim_ros_pkgs::Takeoff>("/airsim_node/SimpleFlight/takeoff");
drone2.take_off = n.serviceClient<airsim_ros_pkgs::Takeoff>("/airsim_node/SimpleFlight2/takeoff");

// drone1.target.x = 477511.763422;
// drone1.target.y = 5523139.13698;

// drone2.target.x = drone1.target.x - 6;
// drone2.target.y = drone1.target.y;

// %EndTag(PUBLISHER)%

// %Tag(LOOP_RATE)%
  ros::Rate loop_rate(10);
// %EndTag(LOOP_RATE)%

drone1.time = drone2.time = ros::Time::now().toSec();

// %Tag(ROS_OK)%
  static int count = 0;
  while (ros::ok())
  {
    globalTime = (ros::Time::now().toSec() - drone1.time);
    if(counter)
    {
      globalStartTime = globalTime;
      counter = false;
      drone1.counter = 1;
      drone2.counter = 1;
      ROS_INFO("Start Time is %f", globalStartTime);
    }
    ROS_INFO("Current state: %d Current time: %f", drone1.state, globalTime);
    if(globalTime > 10 && globalTime < 30 && count < 2) {
      drone1.state = 1;
      drone2.state = 1;
      ++count;
      ROS_INFO("Trying to control now");
    }  

// %Tag(SPINONCE)%
    // ros::spinOnce();
// %EndTag(SPINONCE)%

// %Tag(RATE_SLEEP)%
    loop_rate.sleep();
// %EndTag(RATE_SLEEP)%
  }


  return 0;
}
// %EndTag(FULLTEXT)%


double get_yaw_from_quat_msg(const geometry_msgs::Quaternion& quat_msg)
{
  tf2::Quaternion quat_tf;
  double roll, pitch, yaw;
  tf2::fromMsg(quat_msg, quat_tf);
    tf2::Matrix3x3(quat_tf).getRPY(roll, pitch, yaw);
    return yaw;
}


// void step(Drone_Mission &drone) {
//   // ros::Rate r(0.2);
//     // r.sleep();
//   if(drone.state == 1 && drone.finished != true) {
//     if(drone.name == "drone1") {
//       Pose pose_("drone1/action/pose", true);
//       drone1_maintain_altitude(4, pose_);
//     } else {  
//       Pose pose_("drone2/action/pose", true);
//       drone2_maintain_altitude(4, pose_);
//     }
//     if(ros::Time::now().toSec() > 25) {
//       drone.finished = true;
//     }
//     ROS_INFO("Current state is %d", drone.state);
//   }
//   if(drone.state == 2 && drone.finished != true) {
//     if(drone.current_gps.altitude - drone.mean_start_gps > 0.05){
//       drone.motor_msg.linear.x = 0.0;
//       drone.motor_msg.linear.z = -0.15;
//       drone.motor_msg.angular.z = 0.0;
//       drone.move_drone.publish(drone.motor_msg);  
//     } else {
//       drone.finished = true;
//     }
//     // if(ros::Time::now().toSec() > 90) {
//     //   drone.finished = true;
//     // }
//   }
//   if(drone.state == 3 && drone.finished != true) {
//     // if(drone.current_gps.altitude - drone.mean_start_gps <= 3.95) {
//     //   drone.motor_msg.linear.x = 0.0;
//     //   drone.motor_msg.linear.z = 0.2;
//     //   drone.motor_msg.angular.z = drone.direction;
//     //   drone.move_drone.publish(drone.motor_msg);
//     // } else if(drone.current_gps.altitude - drone.mean_start_gps > 4.05){
//     //   drone.motor_msg.linear.x = 0.0;
//     //   drone.motor_msg.linear.z = -0.2;
//     //   drone.move_drone.publish(drone.motor_msg);  
//     // }
//     drone.motor_msg.linear.x = 0.0;
//     drone.motor_msg.linear.z = 0.0;
//     drone.motor_msg.angular.z = fabs(drone.direction);
//     drone.move_drone.publish(drone.motor_msg);
//     if(drone.yaw > -0.1 && drone.yaw < 0.1) {
//       drone.motor_msg.angular.z = 0.0;
//       ROS_INFO("Yaw value for %s is %f", drone.name.c_str(), drone.yaw);
//       drone.finished = true;
//     }
//     ROS_INFO("Step 3 of drone1");
//   }
//   if(drone.state == 4 && drone.finished != true) {
//     if(drone.current_gps.altitude - drone.mean_start_gps <= 3.95) {
//       drone.motor_msg.linear.z = 0.1;
//       drone.motor_msg.linear.y = 0.0;
//       drone.motor_msg.linear.x = drone.direction;
//       drone.motor_msg.angular.z = 0.0;
//       drone.move_drone.publish(drone.motor_msg);
//     } else if(drone.current_gps.altitude - drone.mean_start_gps > 4.05){
//       drone.motor_msg.linear.x = 0.0;
//       drone.motor_msg.linear.y = 0.0;
//       drone.motor_msg.linear.z = -0.2;
//       drone.motor_msg.angular.z = 0.0;
//       drone.move_drone.publish(drone.motor_msg);  
//     }
//     if(sqrt(pow(drone1.current_utm.easting - drone2.current_utm.easting, 2) + pow(drone1.current_utm.northing - drone2.current_utm.northing, 2)) > 13.8) {
//       drone.finished = true;
//     }
//     ROS_INFO("Easting Difference: %f Northing Difference: %f Distance: %f", drone1.current_utm.easting - drone2.current_utm.easting, 
//     drone1.current_utm.northing - drone2.current_utm.northing, 
//      sqrt(pow(drone1.current_utm.easting - drone2.current_utm.easting, 2) + pow(drone1.current_utm.northing - drone2.current_utm.northing, 2)));  
//   }
//   if(drone.state == 5 && drone.finished != true) {
//     if(drone.name == "drone1"){
//       Pose pose_("/drone1/action/pose", true);
//       drone1_go_to_position(0, -3, 4, pose_);
//     } else {
//       Pose pose_("/drone2/action/pose", true);
//       drone2_go_to_position(0, 3, 4, pose_);
//     }
//     drone.finished = true;
//     // if(ros::Time::now().toSec() > 50) {
//     // }
//     ROS_INFO("State 1 for %s", drone.name.c_str());
//     // ROS_INFO("Orig: %f  Modified: %f", drone.current_gps.altitude, drone.current_gps.altitude - drone.mean_start_gps);
//   }
//   if(drone.state == 6 && drone.finished != true) {
//     if(drone.current_gps.altitude - drone.mean_start_gps <= 3.95) {
//       drone.motor_msg.linear.x = 0.0;
//       drone.motor_msg.linear.y = 0.0;
//       drone.motor_msg.linear.z = 0.3;
//       drone.motor_msg.angular.z = 0.0;
//       drone.move_drone.publish(drone.motor_msg);
//     } else if(drone.current_gps.altitude - drone.mean_start_gps > 4.05){
//       drone.motor_msg.linear.x = 0.0;
//       drone.motor_msg.linear.y = 0.0;
//       drone.motor_msg.linear.z = -0.2;
//       drone.motor_msg.angular.z = 0.0;
//       drone.move_drone.publish(drone.motor_msg);  
//     }
//     if(ros::Time::now().toSec() > 50) {
//       drone.finished = true;
//     }
//   }
//   if(drone.state == 7 && drone.finished != true) {
//     if(drone.theta_yaw_diff > C_PI)
//       drone.theta_yaw_diff = drone.theta_yaw_diff - (2*C_PI);
//     if(fabs(drone.theta_yaw_diff) > 1.56 && fabs(drone.theta_yaw_diff) < 1.58) {
//       drone.rotateToHeading = true;
//       if(drone1.rotateToHeading == true && drone2.rotateToHeading == true) {
//           drone.motor_msg.linear.x = 0.0;
//           drone.motor_msg.linear.y = drone.sign*0.7;
//           drone.motor_msg.linear.z = 0.0;
//           drone.motor_msg.angular.z = 0.0;
//       } else {
//         drone.motor_msg.linear.x = 0.0;
//         drone.motor_msg.linear.y = 0.0;
//         drone.motor_msg.linear.z = 0.0;
//         drone.motor_msg.angular.z = 0.0;
//       }
//     } else {
//       drone.rotateToHeading = false;
//       drone.motor_msg.linear.x = 0.0;
//       drone.motor_msg.linear.y = 0.0;
//       drone.motor_msg.linear.z = 0.0;
//       drone.motor_msg.angular.z = (drone.theta_yaw_diff - (1.57*drone.sign))*0.5;
//       ROS_INFO("%s angle to Target: %f", drone.name.c_str(), drone.theta_yaw_diff*180/C_PI);
//     }
//     drone.move_drone.publish(drone.motor_msg);
//     // ROS_INFO("%s: %f %f", drone.name.c_str(), drone.theta_yaw_diff, fabs(drone.theta - drone.yaw));
//     if(drone.dist_x < 0.2 && drone.dist_y < 0.2) {
//       drone.finished = true;
//     }
//   }
//   if(drone.state == 8 && drone.finished != true) {
//     if(drone.theta < 0.0)
//       drone.sign = 1;
//     else
//       drone.sign = -1;
//     if(fabs(drone.theta_yaw_diff) < 0.05) {
//       drone.motor_msg.linear.x = 1.0;
//       drone.motor_msg.linear.y = 0.0;
//       drone.motor_msg.linear.z = 0.0;
//       drone.motor_msg.angular.z = 0.0;
//     } else {
//       drone.motor_msg.linear.x = 0.0;
//       drone.motor_msg.linear.y = 0.0;
//       drone.motor_msg.linear.z = 0.0;
//       drone.motor_msg.angular.z = (drone.theta_yaw_diff)*0.5;
//     }
//     drone.move_drone.publish(drone.motor_msg);
//     ROS_INFO("%s yaw - theta: %f", drone.name.c_str(), fabs(drone.theta_yaw_diff));
//     // ROS_INFO("%f %f %f %f %f %f %f", drone.subOdom.pose.pose.position.x, drone.target.x, drone.subOdom.pose.pose.position.y, drone.target.y, drone.dist_x, drone.dist_y, drone.theta);
//     if(drone.dist_x < 0.1 && drone.dist_y < 0.1 && drone.dist_x > -0.1 && drone.dist_y > -0.1)
//       drone.finished = true;
//   }
// }


void step(Drone_Mission &drone) {
  // ros::Rate r(0.2);
    // r.sleep();
  if(drone.state == 1 && drone.finished != true) {
    if(drone.current_gps.altitude - drone.mean_start_gps < 5.8) {
      if(drone.current_gps.altitude - drone.mean_start_gps > 5.75)
        drone.motor_msg.twist.linear.z = -0.3;
      else
        drone.motor_msg.twist.linear.z = -0.2;
      drone.motor_msg.twist.angular.z = 0.0;
      drone.move_drone.publish(drone.motor_msg);
    } else {
      drone.motor_msg.twist.linear.z = 0.3;
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
    // ROS_INFO("Orig: %f  Modified: %f", drone.current_gps.altitude, drone.current_gps.altitude - drone.mean_start_gps);
  }
  if(drone.state == 2 && drone.finished != true) {
    if(drone.current_gps.altitude - drone.mean_start_gps > 0.05){
      drone.motor_msg.twist.linear.x = 0.0;
      drone.motor_msg.twist.linear.z = 0.4;
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
  if(drone.state == 4 && drone.finished != true) {
    drone.takeOff(true);
    drone.finished = true;
  }
  if(drone.state == 5 && drone.finished != true) {
    // drone.target.x = 2.0;
    // drone.target.y = 2.0;
    // drone.target.z = -2.0;
    // drone.target.yaw = 0.0;
    drone.compute_control_cmd();
    drone.enforce_dynamic_constraints();
    drone.move_drone.publish(drone.motor_msg);
    drone.check_reached_goal();
    if(drone.reached_goal_){
      drone.timer = ros::Time::now().toSec();
      drone.finished = true;
    }
  }
  if(drone.state == 6 && drone.finished != true) {
    drone.compute_control_cmd();
    drone.enforce_dynamic_constraints();
    drone.move_drone.publish(drone.motor_msg);
    drone.check_reached_goal();
    if(drone.reached_goal_){
      drone.timer = ros::Time::now().toSec();
      drone.finished = true;
    }
  }
  if(drone.state == 7 && drone.finished != true) {
    if(drone.current_gps.altitude - drone.mean_start_gps < 5.8) {
      if(drone.current_gps.altitude - drone.mean_start_gps > 5.75)
        drone.motor_msg.twist.linear.z = -0.3;
      else
        drone.motor_msg.twist.linear.z = -0.2;
      drone.motor_msg.twist.angular.z = 0.0;
      drone.move_drone.publish(drone.motor_msg);
    } else {
      drone.motor_msg.twist.linear.z = 0.3;
      drone.motor_msg.twist.angular.z = 0.0;
      drone.move_drone.publish(drone.motor_msg);  
    }
    if((drone.counter == 1) && ((globalTime - globalStartTime) > 5)) {
      drone.finished = true;
      drone.counter = 0;
      ROS_INFO("Time DIff: %f", (globalTime - globalStartTime));
    }
  }
  if(drone.state == 8 && drone.finished != true) {
    if(drone.current_gps.altitude - drone.mean_start_gps < 5.8) {
      if(drone.current_gps.altitude - drone.mean_start_gps > 5.75)
        drone.motor_msg.twist.linear.z = -0.3;
      else
        drone.motor_msg.twist.linear.z = -0.2;
      drone.motor_msg.twist.angular.z = 0.0;
      drone.move_drone.publish(drone.motor_msg);
    } else {
      drone.motor_msg.twist.linear.z = 0.3;
      drone.motor_msg.twist.angular.z = 0.0;
      drone.move_drone.publish(drone.motor_msg);  
    }
    if((drone.counter == 1) && ((globalTime - globalStartTime) > 5)) {
      drone.finished = true;
      drone.counter = 0;
      ROS_INFO("Time DIff: %f", (globalTime - globalStartTime));
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

bool Drone_Mission::takeOff(bool enable) {
  if (!take_off.waitForExistence(ros::Duration(5.0)))
  {
    ROS_WARN("Take off enable service not found");
    return false;
  }

  takeoff_srv.request.waitOnLastTask = enable;
  return take_off.call(takeoff_srv);
}

void Drone_Mission::compute_control_cmd()
{
    curr_error_.x = target.x - curr_position_.x;
    curr_error_.y = target.y - curr_position_.y;
    curr_error_.z = target.z - curr_position_.z;
    curr_error_.yaw = math_common::angular_dist(curr_position_.yaw, target.yaw);

    double p_term_x = kp_x * curr_error_.x;
    double p_term_y = kp_y * curr_error_.y;
    double p_term_z = kp_z * curr_error_.z;
    double p_term_yaw = kp_yaw * curr_error_.yaw;

    double d_term_x = kd_x * prev_error_.x;
    double d_term_y = kd_y * prev_error_.y;
    double d_term_z = kd_z * prev_error_.z;
    double d_term_yaw = kp_yaw * prev_error_.yaw;

    prev_error_ = curr_error_;

    motor_msg.twist.linear.x = p_term_x + d_term_x;
    motor_msg.twist.linear.y = p_term_y + d_term_y;
    motor_msg.twist.linear.z = p_term_z + d_term_z;
    motor_msg.twist.angular.z = p_term_yaw + d_term_yaw; // todo
}

void Drone_Mission::enforce_dynamic_constraints()
{
    double vel_norm_horz = sqrt((motor_msg.twist.linear.x * motor_msg.twist.linear.x) 
                            + (motor_msg.twist.linear.y * motor_msg.twist.linear.y));

    if (vel_norm_horz > max_vel_horz_abs)
    {
        motor_msg.twist.linear.x = (motor_msg.twist.linear.x / vel_norm_horz) * max_vel_horz_abs; 
        motor_msg.twist.linear.y = (motor_msg.twist.linear.y / vel_norm_horz) * max_vel_horz_abs; 
    }

    if (std::fabs(motor_msg.twist.linear.z) > max_vel_vert_abs)
    {
        // todo just add a sgn funciton in common utils? return double to be safe. 
        // template <typename T> double sgn(T val) { return (T(0) < val) - (val < T(0)); }
        motor_msg.twist.linear.z = (motor_msg.twist.linear.z / std::fabs(motor_msg.twist.linear.z)) * max_vel_vert_abs; 
    }
    // todo yaw limits
    if (std::fabs(motor_msg.twist.linear.z) > max_yaw_rate_degree)
    {
        // todo just add a sgn funciton in common utils? return double to be safe. 
        // template <typename T> double sgn(T val) { return (T(0) < val) - (val < T(0)); }
        motor_msg.twist.linear.z = (motor_msg.twist.linear.z / std::fabs(motor_msg.twist.linear.z)) * max_yaw_rate_degree;
    }

}

void Drone_Mission::check_reached_goal()
{
    diff_xyz = sqrt((target.x - curr_position_.x) * (target.x - curr_position_.x) 
                        + (target.y - curr_position_.y) * (target.y - curr_position_.y)
                        + (target.z - curr_position_.z) * (target.z - curr_position_.z));

    diff_yaw = math_common::angular_dist(target.yaw, curr_position_.yaw);

    // todo save this in degrees somewhere to avoid repeated conversion
    if (diff_xyz < reached_thresh_xyz && diff_yaw < math_common::deg2rad(reached_yaw_degrees))
      reached_goal_ = true; 
}

// void drone1_gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
//   drone1.current_gps = *msg;
//   // geodesy::convert(drone1.current_gps, drone1.gp);
//   // geodesy::fromMsg(drone1.gp, drone1.current_utm);
//   static int count_1 = 0;
//   static double start_alt_1 = 0;
//   static double start_magx = 0;
//   if(ros::Time::now().toSec() < 20) {
//     count_1 += 1;
//     start_alt_1 += drone1.current_gps.altitude;
//     start_magx += drone1.current_magnetic.vector.x;
//     drone1.mean_start_gps = start_alt_1/count_1;
//     drone1.mean_start_mag_x = start_magx/count_1;
//     ROS_INFO("Mean drone1 altitude %f %f %d", drone1.mean_start_gps, start_alt_1, count_1);
//   }
//   alt_output1.x = drone1.current_gps.altitude;
//   alt_output1.y = drone1.current_gps.altitude - drone1.mean_start_gps;
//   altitude1.publish(alt_output1);
//   switch(drone1.state) {
//     case 0: 
//       break;
//     case 1:
//       if(drone1.finished == false) {
//         step(drone1);
//       } else {
//         drone1.state = 8;
//         drone1.direction = 0.3;
//         drone1.finished = false;
//         ROS_INFO("Done Hovering");
//       }
//       break;
//     case 2:
//       if(drone1.finished == false) {
//         step(drone1);
//       } else {
//         drone1.state = 0;
//         drone1.finished = false;
//         ROS_INFO("Done Landing");
//       }
//       break;
//     case 3:
//       if(drone1.finished == false) {
//         step(drone1);
//       } else {
//         drone1.state = 7;
//         drone1.direction = -0.2;
//         drone1.finished = false;
//         ROS_INFO("Done Rotating");
//       }
//       break;
//     case 4:
//       if(drone1.finished == false) {
//         step(drone1);
//       } else {
//         drone1.state = 2;
//         drone1.finished = false;
//         ROS_INFO("Done Distancing");
//       }
//       break;
//     case 5:
//       if(drone1.finished == false) {
//         step(drone1);
//       } else {
//         drone1.state = 3;
//         drone1.target.x = waypoint.x;
//         drone1.target.y = waypoint.y;
//         drone1.finished = false;
//         ROS_INFO("Done Reaching");
//       }
//       break;
//     case 6:
//       if(drone1.finished == false) {
//         step(drone1);
//       } else {
//         drone1.state = 4;
//         drone1.finished =false;
//         ROS_INFO("Done Hovering");
//       }
//       break;
//     case 7:
//       if(drone1.finished == false) {
//         drone1.sign = 1;
//         if(drone2.state == 7)
//           drone1.substate == 1;
//         step(drone1);
//       } else {
//         drone1.state = 2;
//         drone1.finished = false;
//         ROS_INFO("Done Turning to yaw");
//       }
//       break;
//     case 8:
//       if(drone1.finished == false) {
//         drone1.sign = 1;
//         step(drone1);
//       } else {
//         drone1.state = 7;
//         drone1.target.x = waypoint.x;
//         drone1.target.y = waypoint.y;
//         drone1.finished = false;
//         ROS_INFO("Done reaching with gps");
//       }
//       break;
//     default:
//       ROS_INFO("Invalid State");
//       break;
//   }
// }

// void drone2_gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
//   drone2.current_gps = *msg;
//   geodesy::convert(drone2.current_gps, drone2.gp);
//   geodesy::fromMsg(drone2.gp, drone2.current_utm);
//   // utm_drone2.publish(current_drone2_utm);
//   static int count_2 = 0;
//   static double start_alt_2 = 0;
//   static double start_magx = 0;
//   // ROS_INFO("%d", count);
//   if(ros::Time::now().toSec() < 20) {
//     count_2 += 1;
//     start_alt_2 += drone2.current_gps.altitude;
//     start_magx += drone2.current_magnetic.vector.x;
//     drone2.mean_start_gps = start_alt_2/count_2;
//     drone2.mean_start_mag_x = start_magx/count_2;
//     altitude2.publish(alt_output2);
//     ROS_INFO("Mean drone2 altitude %f %f %d", drone2.mean_start_gps, start_alt_2, count_2);
//   }
//   alt_output2.x = drone2.current_gps.altitude;
//   alt_output2.y = drone2.current_gps.altitude - drone2.mean_start_gps;
//   switch(drone2.state) {
//     case 0: 
//       break;
//     case 1:
//       if(drone2.finished == false) {
//         step(drone2);
//       } else {
//         drone2.state = 8;
//         drone2.direction = -0.3;
//         drone2.finished = false;
//         ROS_INFO("Done Hovering");
//       }
//       break;
//     case 2:
//       if(drone2.finished == false) {
//         step(drone2);
//       } else {
//         drone2.state = 0;
//         drone2.finished = false;
//         ROS_INFO("Done Landing");
//       }
//       break;
//     case 3:
//       if(drone2.finished == false) {
//         step(drone2);
//       } else {
//         drone2.state = 7;
//         drone2.direction = -0.2;
//         drone2.finished = false;
//         ROS_INFO("Done Rotating");
//       }
//       break;
//     case 4:
//       if(drone2.finished == false) {
//         step(drone2);
//       } else {
//         drone2.state = 2;
//         drone2.finished = false;
//         ROS_INFO("Done Distancing");
//       }
//       break;
//     case 5:
//       if(drone2.finished == false) {
//         step(drone2);
//       } else {
//         drone2.state = 3;
//         drone2.target.x = waypoint.x;
//         drone2.target.y = waypoint.y;
//         drone2.finished = false;
//         ROS_INFO("Done Reaching");
//       }
//       break;
//     case 6:
//       if(drone2.finished == false) {
//         step(drone2);
//       } else {
//         drone2.state = 4;
//         drone2.finished = false;
//         ROS_INFO("Done Hovering");
//       }
//       break;
//     case 7:
//       if(drone2.finished == false) {
//         drone2.sign = -1;
//         if(drone1.state == 7)
//           drone2.substate == 1;
//         step(drone2);
//       } else {
//         drone2.state = 2;
//         drone2.finished = false;
//         ROS_INFO("Done turning to yaw State: %d", drone2.state);
//       }
//       break;
//     case 8:
//       if(drone2.finished == false) {
//         drone2.sign = -1;
//         step(drone2);
//       } else {
//         drone2.state = 7;
//         drone2.target.x = waypoint.x - 6;
//         drone2.target.y = waypoint.y;
//         drone2.finished = false;
//         ROS_INFO("Done reaching with gps");
//       }
//       break;
//     default:
//       ROS_INFO("Invalid State");
//       break;
//   }
// }

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
    // ROS_INFO("Mean drone1 altitude %f %f %d", drone1.mean_start_gps, start_alt_1, count_1);
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
        drone1.state = 5;
        drone1.target.x = -3;
        drone1.target.y = 6;
        drone1.target.z = -5;
        drone1.reached_goal_ = false;
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
    case 4:
      if(drone1.finished == false) {
        step(drone1);
      } else {
        drone1.state = 0;
        drone1.finished = false;
        ROS_INFO("Done Taking off");
      }
      break;  
    case 5:
      if(drone1.finished == false) {
        step(drone1);
      } else {
        drone1.state = 7;
        counter = true;
        drone1.finished = false;
        ROS_INFO("Done Moving in Local Frame");
      }
      break; 
    case 6:
      if(drone1.finished == false) {
        step(drone1);
      } else {
        drone1.state = 8;
        counter = true;
        drone1.finished = false;
        ROS_INFO("Done Moving in Local Frame");
      }
      break; 
    case 7:
      if(drone1.finished == false) {
        step(drone1);
      } else {
        drone1.state = 6;
        drone1.target.x = 7;
        drone1.target.y = 6;
        drone1.target.z = -5;
        drone1.reached_goal_ = false;
        drone1.finished = false;
        ROS_INFO("Done waiting for 5 seconds");
      }
      break; 
    case 8:
      if(drone1.finished == false) {
        step(drone1);
      } else {
        drone1.state = 2;
        drone1.target.x = 7;
        drone1.target.y = 6;
        drone1.target.z = -5;
        drone1.reached_goal_ = false;
        drone1.finished = false;
        ROS_INFO("Done waiting for 5 seconds");
      }
      break; 
    default:
      ROS_INFO("Invalid State");
      break;
  }
}

void drone2_gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
  drone2.current_gps = *msg;
  // utm_drone2.publish(current_drone2_utm);
  static int count_1 = 0;
  static double start_alt_1 = 0;
  static double start_magx = 0;
  if(ros::Time::now().toSec() - drone2.time < 10) {
    count_1 += 1;
    start_alt_1 += drone2.current_gps.altitude;
    drone2.mean_start_gps = start_alt_1/count_1;
    // ROS_INFO("Mean drone2 altitude %f %f %d", drone2.mean_start_gps, start_alt_1, count_1);
  }
  // alt_output1.x = drone2.current_gps.altitude;
  // alt_output1.y = drone2.current_gps.altitude - drone2.mean_start_gps;
  // altitude1.publish(alt_output1);
  switch(drone2.state) {
    case 0: 
      break;
    case 1:
      if(drone2.finished == false) {
        step(drone2);
      } else {
        drone2.state = 5;
        drone2.target.x = 3;
        drone2.target.y = -6;
        drone2.target.z = -5;
        drone2.target.yaw = 0;
        drone2.reached_goal_ = false;
        drone2.finished = false;
        ROS_INFO("Done Hovering");
      }
      break;
    case 2:
      if(drone2.finished == false) {
        step(drone2);
      } else {
        drone2.state = 0;
        drone2.finished = false;
        ROS_INFO("Done Landing");
      }
      break;
    case 3:
      if(drone2.finished == false) {
        step(drone2);
      } else {
        drone2.state = 2;
        drone2.finished = false;
        ROS_INFO("Done Moving");
      }
      break;
    case 4:
      if(drone2.finished == false) {
        step(drone2);
      } else {
        drone2.state = 0;
        drone2.finished = false;
        ROS_INFO("Done Moving");
      }
      break;
    case 5:
      if(drone2.finished == false) {
        step(drone2);
      } else {
        drone2.state = 7;
        drone2.finished = false;
        ROS_INFO("Done Moving in Local Frame");
      }
      break;
    case 6:
      if(drone2.finished == false) {
        step(drone2);
      } else {
        drone2.state = 8;
        drone2.finished = false;
        drone2.timer = 0;
        ROS_INFO("Done Moving in Local Frame");
      }
      break;
    case 7:
      if(drone2.finished == false) {
        step(drone2);
      } else {
        drone2.state = 6;
        drone2.target.x = 13;
        drone2.target.y = -6;
        drone2.target.z = -5;
        drone2.reached_goal_ = false;
        drone2.finished = false;
        ROS_INFO("Done Waiting for 5 seconds");
      }
      break;
    case 8:
      if(drone2.finished == false) {
        step(drone2);
      } else {
        drone2.state = 2;
        drone2.target.x = 13;
        drone2.target.y = -6;
        drone2.target.z = -5;
        drone2.reached_goal_ = false;
        drone2.finished = false;
        ROS_INFO("Done Waiting for 5 seconds");
      }
      break;
    default:
      ROS_INFO("Invalid State");
      break;
  }
}