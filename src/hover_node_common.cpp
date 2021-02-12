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

#include "dji_sdk/dji_sdk.h"

Drone_Mission *drone1;

Drone_Mission *drone2;

const float deg2rad = C_PI/180.0;
const float rad2deg = 180.0/C_PI;

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

  drone1 = new Drone_Mission("frl_uas5", n);

  drone2 = new Drone_Mission("frl_uas1", n);

  drone1->time = drone2->time = ros::Time::now().toSec();

  drone1->fix = n.subscribe("frl_uas5/dji_sdk/gps_position", 10, &drone1_gps_callback);

  drone2->fix = n.subscribe("frl_uas1/dji_sdk/gps_position", 10, &drone2_gps_callback);

  // %EndTag(PUBLISHER)%

  // %Tag(LOOP_RATE)%
  ros::Rate loop_rate(10);
  // %EndTag(LOOP_RATE)%

  drone1->obtain_control_result = drone1->obtain_control();
  
  drone2->obtain_control_result = drone2->obtain_control();

  if(drone1->is_M100())
  {
    ROS_INFO("M100 taking off!");
    drone1->takeoff_result = drone1->M100monitoredTakeoff();
    // drone1->finished = drone1->takeoff_land(6);
  }
  else
  {
    ROS_INFO("A3/N3 taking off!");
    drone1->takeoff_result = drone1->monitoredTakeoff();
  }

    if(drone2->is_M100())
  {
    ROS_INFO("M100 taking off!");
    drone2->takeoff_result = drone2->M100monitoredTakeoff();
    // drone2->finished = drone2->takeoff_land(6);
  }
  else
  {
    ROS_INFO("A3/N3 taking off!");
    drone2->takeoff_result = drone2->monitoredTakeoff();
  }

  // %Tag(ROS_OK)%
  static int count = 0;
  while (ros::ok())
  {
    ROS_INFO("Current state: %d %d | Current Time: %f", drone1->state, drone2->state, (ros::Time::now().toSec() - drone1->time));
    if((ros::Time::now().toSec() - drone1->time) > 20 && (ros::Time::now().toSec() - drone1->time) < 30 && count < 2) {
      drone1->state = drone2->state = 1;
      drone1->setTarget(drone1->mean_start_gps_x, drone1->mean_start_gps_y, 4.0, 0.0);
      drone2->setTarget(drone2->mean_start_gps_x, drone2->mean_start_gps_y, 4.0, 0.0);
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


void step(Drone_Mission &drone) {
  if(drone.finished != true) {
    drone.compute_control_cmd();
    drone.enforce_dynamic_constraints();
    drone.move_drone.publish(drone.motor_msg);
    drone.check_reached_goal();
    if(drone.reached_goal_){
      drone.finished = true;
    }
    // ROS_INFO("Trying to go to %f %f %f", drone.target.x, drone.target.y, drone.target.z);
  }
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
  curr_odom_ = *msg;
  curr_position_.x = curr_odom_.pose.pose.position.x;
  curr_position_.y = curr_odom_.pose.pose.position.y;
  curr_position_.z = curr_odom_.pose.pose.position.z;
  curr_position_.yaw = get_yaw_from_quat_msg(curr_odom_.pose.pose.orientation);
  
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
  drone2->current_gps = *msg;
  // utm_drone2->publish(current_drone1_utm);
  static int count_2 = 0;
  static double start_alt_2 = 0;
  static double start_x_2 = 0;
  static double start_y_2 = 0;
  // static double start_magx = 0;
  if((ros::Time::now().toSec() - drone2->time) < 20) {
    count_2 += 1;
    start_alt_2 += drone2->current_gps.altitude;
    start_x_2 += drone2->curr_odom_.pose.pose.position.x;
    start_y_2 += drone2->curr_odom_.pose.pose.position.y;
    drone2->mean_start_gps_alt = start_alt_2/count_2;
    drone2->mean_start_gps_x = start_x_2/count_2;
    drone2->mean_start_gps_y = start_y_2/count_2;
    // ROS_INFO("Mean drone1 altitude %f %f %d", drone2->mean_start_gps, start_alt_2, count_2);
  }
  switch(drone2->state) {
    case 0: 
      break;
    case 1:
      if(drone2->finished == false) {
        step(*drone1);
      } else {
        drone2->state = 3;
        drone2->setTarget(drone2->mean_start_gps_x - 2, drone2->mean_start_gps_y, 4.0, 0.0); // drone2->target.x = -3;
        // drone2->target.y = 4.5;
        // drone2->target.z = -5;
        drone2->reached_goal_ = false;
        drone2->finished = false;
        ROS_INFO("Done with route 1");
      }
      break;
    case 2:
      if(drone2->finished == false) {
        step(*drone1);
      } else {
        drone2->finished = drone2->takeoff_land(6); //Landing
        drone2->state = 0;
        ROS_INFO("Done Landing");
      }
      break;
    case 3:
      if(drone2->finished == false) {
        step(*drone1);
      } else {
        drone2->state = 4;
        drone2->setTarget(drone2->mean_start_gps_x - 2, drone2->mean_start_gps_y - 2, 4.0, 0.0);
        drone2->reached_goal_ = false;
        drone2->finished = false;
        ROS_INFO("Done with route 2");
      }
      break;
    case 4:
      if(drone2->finished == false) {
        step(*drone1);
      } else {
        drone2->state = 5;
        drone2->setTarget(drone2->mean_start_gps_x, drone2->mean_start_gps_y - 2, 4.0, 0.0);
        drone2->reached_goal_ = false;
        drone2->finished = false;
        ROS_INFO("Done with route 3");
      }
      break;
    case 5:
      if(drone2->finished == false) {
        step(*drone1);
      } else {
        drone2->state = 2;
        drone2->setTarget(drone2->mean_start_gps_x, drone2->mean_start_gps_y, 4.0, 0.0);
        drone2->reached_goal_ = false;
        drone2->finished = false;
        ROS_INFO("Done with route 4");
      }
      break;
    default:
      ROS_INFO("Invalid State");
      break;
  }
}

void drone2_gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
  drone1->current_gps = *msg;
  // utm_drone1->publish(current_drone1_utm);
  static int count_1 = 0;
  static double start_alt_1 = 0;
  static double start_x_1 = 0;
  static double start_y_1 = 0;
  // static double start_magx = 0;
  if((ros::Time::now().toSec() - drone1->time) < 20) {
    count_1 += 1;
    start_alt_1 += drone1->current_gps.altitude;
    start_x_1 += drone1->curr_odom_.pose.pose.position.x;
    start_y_1 += drone1->curr_odom_.pose.pose.position.y;
    drone1->mean_start_gps_alt = start_alt_1/count_1;
    drone1->mean_start_gps_x = start_x_1/count_1;
    drone1->mean_start_gps_y = start_y_1/count_1;
    // ROS_INFO("Mean drone1 altitude %f %f %d", drone1->mean_start_gps, start_alt_1, count_1);
  }
  switch(drone1->state) {
    case 0: 
      break;
    case 1:
      if(drone1->finished == false) {
        step(*drone1);
      } else {
        drone1->state = 3;
        drone1->setTarget(drone1->mean_start_gps_x - 2, drone1->mean_start_gps_y, 4.0, 0.0); // drone1->target.x = -3;
        // drone1->target.y = 4.5;
        // drone1->target.z = -5;
        drone1->reached_goal_ = false;
        drone1->finished = false;
        ROS_INFO("Done with route 1");
      }
      break;
    case 2:
      if(drone1->finished == false) {
        step(*drone1);
      } else {
        drone1->finished = drone1->takeoff_land(6); //Landing
        drone1->state = 0;
        ROS_INFO("Done Landing");
      }
      break;
    case 3:
      if(drone1->finished == false) {
        step(*drone1);
      } else {
        drone1->state = 4;
        drone1->setTarget(drone1->mean_start_gps_x - 2, drone1->mean_start_gps_y - 2, 4.0, 0.0);
        drone1->reached_goal_ = false;
        drone1->finished = false;
        ROS_INFO("Done with route 2");
      }
      break;
    case 4:
      if(drone1->finished == false) {
        step(*drone1);
      } else {
        drone1->state = 5;
        drone1->setTarget(drone1->mean_start_gps_x, drone1->mean_start_gps_y - 2, 4.0, 0.0);
        drone1->reached_goal_ = false;
        drone1->finished = false;
        ROS_INFO("Done with route 3");
      }
      break;
    case 5:
      if(drone1->finished == false) {
        step(*drone1);
      } else {
        drone1->state = 2;
        drone1->setTarget(drone1->mean_start_gps_x, drone1->mean_start_gps_y, 4.0, 0.0);
        drone1->reached_goal_ = false;
        drone1->finished = false;
        ROS_INFO("Done with route 4");
      }
      break;
    default:
      ROS_INFO("Invalid State");
      break;
  }
}

bool Drone_Mission::obtain_control()
{
  dji_sdk::SDKControlAuthority authority;
  authority.request.control_enable=1;
  sdk_ctrl_authority_service.call(authority);

  if(!authority.response.result)
  {
    ROS_ERROR("obtain control failed!");
    return false;
  }

  return true;
}

bool Drone_Mission::is_M100()
{
  dji_sdk::QueryDroneVersion query;
  query_version_service.call(query);

  if(query.response.version == DJISDK::DroneFirmwareVersion::M100_31)
  {
    return true;
  }

  return false;
}

bool Drone_Mission::monitoredTakeoff()
{
  ros::Time start_time = ros::Time::now();

  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF)) {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1.1: Spin the motor
  while (flight_status != DJISDK::FlightStatus::STATUS_ON_GROUND &&
         display_mode != DJISDK::DisplayMode::MODE_ENGINE_START &&
         ros::Time::now() - start_time < ros::Duration(5)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(ros::Time::now() - start_time > ros::Duration(5)) {
    ROS_ERROR("Takeoff failed. Motors are not spinnning.");
    return false;
  }
  else {
    start_time = ros::Time::now();
    ROS_INFO("Motor Spinning ...");
    ros::spinOnce();
  }


  // Step 1.2: Get in to the air
  while (flight_status != DJISDK::FlightStatus::STATUS_IN_AIR &&
          (display_mode != DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode != DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
          ros::Time::now() - start_time < ros::Duration(20)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(ros::Time::now() - start_time > ros::Duration(20)) {
    ROS_ERROR("Takeoff failed. Aircraft is still on the ground, but the motors are spinning.");
    return false;
  }
  else {
    start_time = ros::Time::now();
    ROS_INFO("Ascending...");
    ros::spinOnce();
  }

  // Final check: Finished takeoff
  while ( (display_mode == DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode == DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
          ros::Time::now() - start_time < ros::Duration(20)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if ( display_mode != DJISDK::DisplayMode::MODE_P_GPS || display_mode != DJISDK::DisplayMode::MODE_ATTITUDE)
  {
    ROS_INFO("Successful takeoff!");
    start_time = ros::Time::now();
  }
  else
  {
    ROS_ERROR("Takeoff finished, but the aircraft is in an unexpected mode. Please connect DJI GO.");
    return false;
  }

  return true;
}

bool Drone_Mission::M100monitoredTakeoff()
{
  ros::Time start_time = ros::Time::now();

  float home_altitude = current_gps.altitude;
  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF))
  {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1: If M100 is not in the air after 10 seconds, fail.
  while (ros::Time::now() - start_time < ros::Duration(10))
  {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR ||
      current_gps.altitude - home_altitude < 1.0)
  {
    ROS_ERROR("Takeoff failed.");
    return false;
  }
  else
  {
    start_time = ros::Time::now();
    ROS_INFO("Successful takeoff!");
    ros::spinOnce();
  }

  return true;
}

void Drone_Mission::flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  flight_status = msg->data;
}

void Drone_Mission::display_mode_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  display_mode = msg->data;
}

bool Drone_Mission::takeoff_land(int task)
{
  dji_sdk::DroneTaskControl droneTaskControl;

  droneTaskControl.request.task = task;

  drone_task_service.call(droneTaskControl);

  if(!droneTaskControl.response.result)
  {
    ROS_ERROR("takeoff_land fail");
    return false;
  }

  return true;
}

void Drone_Mission::setTarget(float x, float y, float z, float yaw)
{
  target.x = x;
  target.y = y;
  target.z = z;
  target.yaw = yaw;
}


void Drone_Mission::gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
  current_gps = *msg;
  // utm_publish(current_drone1_utm);
  static int count_1 = 0;
  static double start_alt_1 = 0;
  static double start_x_1 = 0;
  static double start_y_1 = 0;
  // static double start_magx = 0;
  if((ros::Time::now().toSec() - time) < 20) {
    count_1 += 1;
    start_alt_1 += current_gps.altitude;
    start_x_1 += curr_odom_.pose.pose.position.x;
    start_y_1 += curr_odom_.pose.pose.position.y;
    mean_start_gps_alt = start_alt_1/count_1;
    mean_start_gps_x = start_x_1/count_1;
    mean_start_gps_y = start_y_1/count_1;
    // ROS_INFO("Mean drone1 altitude %f %f %d", mean_start_gps, start_alt_1, count_1);
  }
  switch(state) {
    case 0: 
      break;
    case 1:
      if(finished == false) {
        step(*drone1);
      } else {
        state = 3;
        setTarget(mean_start_gps_x - 2, mean_start_gps_y, 4.0, 0.0); // target.x = -3;
        // target.y = 4.5;
        // target.z = -5;
        reached_goal_ = false;
        finished = false;
        ROS_INFO("Done with route 1");
      }
      break;
    case 2:
      if(finished == false) {
        step(*drone1);
      } else {
        finished = takeoff_land(6); //Landing
        state = 0;
        ROS_INFO("Done Landing");
      }
      break;
    case 3:
      if(finished == false) {
        step(*drone1);
      } else {
        state = 4;
        setTarget(mean_start_gps_x - 2, mean_start_gps_y - 2, 4.0, 0.0);
        reached_goal_ = false;
        finished = false;
        ROS_INFO("Done with route 2");
      }
      break;
    case 4:
      if(finished == false) {
        step(*drone1);
      } else {
        state = 5;
        setTarget(mean_start_gps_x, mean_start_gps_y - 2, 4.0, 0.0);
        reached_goal_ = false;
        finished = false;
        ROS_INFO("Done with route 3");
      }
      break;
    case 5:
      if(finished == false) {
        step(*drone1);
      } else {
        state = 2;
        setTarget(mean_start_gps_x, mean_start_gps_y, 4.0, 0.0);
        reached_goal_ = false;
        finished = false;
        ROS_INFO("Done with route 4");
      }
      break;
    default:
      ROS_INFO("Invalid State");
      break;
  }
}

