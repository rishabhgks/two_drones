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
#include "../include/two_drones/math_common.h"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <dji_sdk/dji_sdk.h>

float globalTime = 0;
float globalStartTime = 0;
bool counter = false;

Drone_Mission *drone1;
Drone_Mission *drone2;
Drone_Mission *drone3;

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

  drone1 = new Drone_Mission("drone1", n);
  drone2 = new Drone_Mission("drone2", n);
  drone3 = new Drone_Mission("drone3", n);

  drone1->fix = n.subscribe("airsim_node/drone1/global_gps", 10, &drone1_gps_callback);
  drone2->fix = n.subscribe("airsim_node/drone2/global_gps", 10, &drone2_gps_callback);
  drone3->fix = n.subscribe("airsim_node/drone3/global_gps", 10, &drone3_gps_callback);

  // %EndTag(PUBLISHER)%

  // %Tag(LOOP_RATE)%
  ros::Rate loop_rate(10);
  // %EndTag(LOOP_RATE)%

  drone1->time = drone2->time = drone3->time = ros::Time::now().toSec();

  // %Tag(ROS_OK)%
    static int count = 0;
    while (ros::ok())
    {
      globalTime = (ros::Time::now().toSec() - drone1->time);
      if(counter)
      {
        globalStartTime = globalTime;
        counter = false;
        drone1->counter = 1;
        drone2->counter = 1;
        ROS_INFO("Start Time is %f", globalStartTime);
      }
      ROS_INFO("Current state: %d Current time: %f", drone1->state, globalTime);
      if(globalTime > 10 && globalTime < 30 && count < 2) {
        drone1->state = 1;
        drone2->state = 1;
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


// void transform_world_to_local(Drone_Mission &drone) 
// {
//   drone.local_point = tfBuffer.transform(drone.world_point, drone.drone_frame);
//   drone.target.x = drone.local_point.point.x;
//   drone.target.y = drone.local_point.point.y;
//   drone.target.z = drone.local_point.point.z;
//   ROS_INFO("Target set as : %f %f %f", drone.target.x, drone.target.y, drone.target.z);
// }


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
  // if(drone.state == 1 && drone.finished != true) {
  //   if(drone.current_gps.altitude - drone.mean_start_gps < 5.8) {
  //     if(drone.current_gps.altitude - drone.mean_start_gps > 5.75)
  //       drone.motor_msg.twist.linear.z = -0.3;
  //     else
  //       drone.motor_msg.twist.linear.z = -0.6;
  //     drone.motor_msg.twist.angular.z = 0.0;
  //     drone.move_drone.publish(drone.motor_msg);
  //   } else {
  //     drone.motor_msg.twist.linear.z = 0.3;
  //     drone.motor_msg.twist.angular.z = 0.0;
  //     drone.move_drone.publish(drone.motor_msg);  
  //   }
  //   if(ros::Time::now().toSec() - drone.time > 40) {
  //     drone.finished = true;
  //   }
  //   // ROS_INFO("Orig: %f  Modified: %f", drone.current_gps.altitude, drone.current_gps.altitude - drone.mean_start_gps);
  // }
  // if(drone.state == 2 && drone.finished != true) {
  //   if(drone.current_gps.altitude - drone.mean_start_gps > 0.05){
  //     drone.motor_msg.twist.linear.x = 0.0;
  //     drone.motor_msg.twist.linear.z = 0.6;
  //     drone.motor_msg.twist.angular.z = 0.0;
  //     drone.move_drone.publish(drone.motor_msg);  
  //   } else {
  //     drone.finished = true;
  //   }
  //   // if(ros::Time::now().toSec() - drone.time > 90) {
  //   //   drone.finished = true;
  //   // }
  // }
  // if(drone.state == 3 && drone.finished != true) {
  //   drone.motor_msg.twist.linear.x = 1.0;
  //   drone.motor_msg.twist.linear.z = 0.0;
  //   drone.motor_msg.twist.angular.z = 0.0;
  //   drone.move_drone.publish(drone.motor_msg);
  //   if(ros::Time::now().toSec() - drone.time > 60) {
  //     drone.finished = true;
  //   }
  // }
  // // if(drone.state == 4 && drone.finished != true) {
  // //   drone.takeOff(true);
  // //   drone.finished = true;
  // // }
  // if(drone.state == 5 && drone.finished != true) {
  //   // drone.target.x = 2.0;
  //   // drone.target.y = 2.0;
  //   // drone.target.z = -2.0;
  //   // drone.target.yaw = 0.0;
  //   drone.compute_control_cmd();
  //   drone.enforce_dynamic_constraints();
  //   drone.move_drone.publish(drone.motor_msg);
  //   drone.check_reached_goal();
  //   if(drone.reached_goal_){
  //     drone.timer = ros::Time::now().toSec();
  //     drone.finished = true;
  //   }
  //   // ROS_INFO("Trying to go to %f %f %f", drone.target.x, drone.target.y, drone.target.z);
  // }
  // if(drone.state == 6 && drone.finished != true) {
  //   drone.compute_control_cmd();
  //   drone.enforce_dynamic_constraints();
  //   drone.move_drone.publish(drone.motor_msg);
  //   drone.check_reached_goal();
  //   if(drone.reached_goal_){
  //     drone.timer = ros::Time::now().toSec();
  //     drone.finished = true;
  //   }
  // }
  // if(drone.state == 7 && drone.finished != true) {
  //   if(drone.current_gps.altitude - drone.mean_start_gps < 5.5) {
  //     if(drone.current_gps.altitude - drone.mean_start_gps > 5.45)
  //       drone.motor_msg.twist.linear.z = -0.3;
  //     else
  //       drone.motor_msg.twist.linear.z = -0.6;
  //     drone.motor_msg.twist.angular.z = 0.0;
  //     drone.move_drone.publish(drone.motor_msg);
  //   } else {
  //     drone.motor_msg.twist.linear.z = 0.3;
  //     drone.motor_msg.twist.angular.z = 0.0;
  //     drone.move_drone.publish(drone.motor_msg);  
  //   }
  //   if((drone.counter == 1) && ((globalTime - globalStartTime) > 5)) {
  //     drone3->state = 9;
  //     if(drone3->counter == 1) {
  //       drone.finished = true;
  //       drone.counter = 0;
  //     }
  //     ROS_INFO("Time DIff: %f", (globalTime - globalStartTime));
  //   }
  // }
  // if(drone.state == 8 && drone.finished != true) {
  //   if(drone.current_gps.altitude - drone.mean_start_gps < 5.5) {
  //     if(drone.current_gps.altitude - drone.mean_start_gps > 5.45)
  //       drone.motor_msg.twist.linear.z = -0.3;
  //     else
  //       drone.motor_msg.twist.linear.z = -0.6;
  //     drone.motor_msg.twist.angular.z = 0.0;
  //     drone.move_drone.publish(drone.motor_msg);
  //   } else {
  //     drone.motor_msg.twist.linear.z = 0.3;
  //     drone.motor_msg.twist.angular.z = 0.0;
  //     drone.move_drone.publish(drone.motor_msg);  
  //   }
  //   if((drone.counter == 1) && ((globalTime - globalStartTime) > 5)) {
  //     drone.finished = true;
  //     drone.counter = 0;
  //     ROS_INFO("Time DIff: %f", (globalTime - globalStartTime));
  //   }
  // }
  // if(drone.state == 9 && !drone.finished) {
  //   if(drone.current_gps.altitude - drone.mean_start_gps < 5.8) {
  //     if(drone.current_gps.altitude - drone.mean_start_gps > 5.75)
  //       drone.motor_msg.twist.linear.z = -0.3;
  //     else
  //       drone.motor_msg.twist.linear.z = -0.8;
  //     drone.motor_msg.twist.angular.z = 0.0;
  //     drone.move_drone.publish(drone.motor_msg);
  //   } else {
  //     drone.motor_msg.twist.linear.z = 0.3;
  //     drone.motor_msg.twist.angular.z = 0.0;
  //     drone.move_drone.publish(drone.motor_msg);  
  //     drone.counter = 1;
  //     ROS_INFO("Drone 3 Reached higher than 4.8 altitude");
  //   }
  // }
}


// void Drone_Mission::magneticCallback(const sensor_msgs::MagneticField::ConstPtr& msg) {
//   current_magnetic = *msg;
// }

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

// bool Drone_Mission::takeOff(bool enable) {
//   if (!take_off.waitForExistence(ros::Duration(5.0)))
//   {
//     ROS_WARN("Take off enable service not found");
//     return false;
//   }

//   takeoff_srv.request.waitOnLastTask = enable;
//   return take_off.call(takeoff_srv);
// }

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

    motor_msg.axes[0] = p_term_x + d_term_x;
    motor_msg.axes[1] = p_term_y + d_term_y;
    motor_msg.axes[2] = p_term_z + d_term_z;
    motor_msg.axes[3] = p_term_yaw + d_term_yaw; // todo
}

void Drone_Mission::enforce_dynamic_constraints()
{
    double vel_norm_horz = sqrt((motor_msg.axes[0] * motor_msg.axes[0]) 
                            + (motor_msg.axes[1] * motor_msg.axes[1]));

    if (vel_norm_horz > max_vel_horz_abs)
    {
        motor_msg.axes[0] = (motor_msg.axes[0] / vel_norm_horz) * max_vel_horz_abs; 
        motor_msg.axes[1] = (motor_msg.axes[1] / vel_norm_horz) * max_vel_horz_abs; 
    }

    if (std::fabs(motor_msg.axes[2]) > max_vel_vert_abs)
    {
        // todo just add a sgn funciton in common utils? return double to be safe. 
        // template <typename T> double sgn(T val) { return (T(0) < val) - (val < T(0)); }
        motor_msg.axes[2] = (motor_msg.axes[2] / std::fabs(motor_msg.axes[2])) * max_vel_vert_abs; 
    }
    // todo yaw limits
    if (std::fabs(motor_msg.axes[2]) > max_yaw_rate_degree)
    {
        // todo just add a sgn funciton in common utils? return double to be safe. 
        // template <typename T> double sgn(T val) { return (T(0) < val) - (val < T(0)); }
        motor_msg.axes[2] = (motor_msg.axes[2] / std::fabs(motor_msg.axes[2])) * max_yaw_rate_degree;
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

void drone1_gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
  drone1->current_gps = *msg;
  // utm_drone1->publish(current_drone1_utm);
  static int count_1 = 0;
  static double start_alt_1 = 0;
  static double start_magx = 0;
  if(ros::Time::now().toSec() - drone1->time < 10) {
    count_1 += 1;
    start_alt_1 += drone1->current_gps.altitude;
    drone1->mean_start_gps_alt = start_alt_1/count_1;
    // ROS_INFO("Mean drone1 altitude %f %f %d", drone1->mean_start_gps, start_alt_1, count_1);
  }
  switch(drone1->state) {
    case 0: 
      break;
    case 1:
      if(drone1->finished == false) {
        step(*drone1);
      } else {
        drone1->state = 5;
        drone1->target.x = -3;
        drone1->target.y = 4.5;
        drone1->target.z = -5;
        // drone1->world_point.point.x = 0;
        // drone1->world_point.point.y = -3;
        // drone1->world_point.point.z = -5;
        // transform_world_to_local(*drone1);
        drone1->reached_goal_ = false;
        drone1->finished = false;
        ROS_INFO("Done Hovering");
      }
      break;
    case 2:
      if(drone1->finished == false) {
        step(*drone1);
      } else {
        drone1->state = 0;
        drone1->finished = false;
        ROS_INFO("Done Landing");
      }
      break;
    case 3:
      if(drone1->finished == false) {
        step(*drone1);
      } else {
        drone1->state = 2;
        drone1->finished = false;
        ROS_INFO("Done Moving");
      }
      break;
    case 4:
      if(drone1->finished == false) {
        step(*drone1);
      } else {
        drone1->state = 0;
        drone1->finished = false;
        ROS_INFO("Done Taking off");
      }
      break;  
    case 5:
      if(drone1->finished == false) {
        step(*drone1);
      } else {
        drone1->state = 7;
        counter = true;
        drone1->finished = false;
        ROS_INFO("Done Moving in Local Frame");
      }
      break; 
    case 6:
      if(drone1->finished == false) {
        step(*drone1);
      } else {
        drone1->state = 8;
        counter = true;
        drone1->finished = false;
        ROS_INFO("Done Moving in Local Frame");
      }
      break; 
    case 7:
      if(drone1->finished == false) {
        step(*drone1);
      } else {
        drone1->state = 6;
        drone1->target.x = 7;
        drone1->target.y = 4.5;
        drone1->target.z = -5;
        drone1->reached_goal_ = false;
        drone1->finished = false;
        ROS_INFO("Done waiting for 5 seconds");
      }
      break; 
    case 8:
      if(drone1->finished == false) {
        step(*drone1);
      } else {
        drone1->state = 2;
        // drone1->target.x = 5;
        // drone1->target.y = 6;
        // drone1->target.z = -5;
        drone1->reached_goal_ = false;
        drone1->finished = false;
        ROS_INFO("Done waiting for 5 seconds");
      }
      break; 
    default:
      ROS_INFO("Invalid State");
      break;
  }
}

void drone2_gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
  drone2->current_gps = *msg;
  // utm_drone2->publish(current_drone2_utm);
  static int count_1 = 0;
  static double start_alt_1 = 0;
  static double start_magx = 0;
  if(ros::Time::now().toSec() - drone2->time < 10) {
    count_1 += 1;
    start_alt_1 += drone2->current_gps.altitude;
    drone2->mean_start_gps_alt = start_alt_1/count_1;
    // ROS_INFO("Mean drone2 altitude %f %f %d", drone2->mean_start_gps, start_alt_1, count_1);
  }
  switch(drone2->state) {
    case 0: 
      break;
    case 1:
      if(drone2->finished == false) {
        step(*drone2);
      } else {
        drone2->state = 5;
        drone2->target.x = 3;
        drone2->target.y = -4.5;
        drone2->target.z = -5;
        drone2->target.yaw = 0;
        drone2->reached_goal_ = false;
        drone2->finished = false;
        ROS_INFO("Done Hovering");
      }
      break;
    case 2:
      if(drone2->finished == false) {
        step(*drone2);
      } else {
        drone2->state = 0;
        drone2->finished = false;
        ROS_INFO("Done Landing");
      }
      break;
    case 3:
      if(drone2->finished == false) {
        step(*drone2);
      } else {
        drone2->state = 2;
        drone2->finished = false;
        ROS_INFO("Done Moving");
      }
      break;
    case 4:
      if(drone2->finished == false) {
        step(*drone2);
      } else {
        drone2->state = 0;
        drone2->finished = false;
        ROS_INFO("Done Moving");
      }
      break;
    case 5:
      if(drone2->finished == false) {
        step(*drone2);
      } else {
        drone2->state = 7;
        drone2->finished = false;
        ROS_INFO("Done Moving in Local Frame");
      }
      break;
    case 6:
      if(drone2->finished == false) {
        step(*drone2);
      } else {
        drone2->state = 8;
        drone2->finished = false;
        drone2->timer = 0;
        ROS_INFO("Done Moving in Local Frame");
      }
      break;
    case 7:
      if(drone2->finished == false) {
        step(*drone2);
      } else {
        drone2->state = 6;
        drone2->target.x = 13;
        drone2->target.y = -4.5;
        drone2->target.z = -5;
        drone2->reached_goal_ = false;
        drone2->finished = false;
        ROS_INFO("Done Waiting for 5 seconds");
      }
      break;
    case 8:
      if(drone2->finished == false) {
        step(*drone2);
      } else {
        drone2->state = 2;
        // drone2->target.x = 8;
        // drone2->target.y = -4;
        // drone2->target.z = -5;
        drone2->reached_goal_ = false;
        drone2->finished = false;
        ROS_INFO("Done Waiting for 5 seconds");
      }
      break;
    default:
      ROS_INFO("Invalid State");
      break;
  }
}

void drone3_gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
  drone3->current_gps = *msg;
  static int count_3 = 0;
  static double start_alt_3 = 0;
  static double start_magx = 0;
  if(ros::Time::now().toSec() - drone3->time < 10) {
    count_3 += 1;
    start_alt_3 += drone3->current_gps.altitude;
    drone3->mean_start_gps_alt = start_alt_3/count_3;
  }
  switch(drone3->state) {
    case 0:
      break;
    case 1:
      drone3->state = 5;
      drone3->target.x = 6;
      drone3->target.y = 0;
      drone3->target.z = -5;
      drone3->reached_goal_ = false;
      drone3->finished = false;
      break;
    case 5:
      if(drone3->finished == false) {
        step(*drone3);
      }
      break;
    case 9:
      if(!drone3->finished) {
        step(*drone3);
      }
      break;
    default:
      ROS_INFO("Invalid State");
      break; 
  }
}