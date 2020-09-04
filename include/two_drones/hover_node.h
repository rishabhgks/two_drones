
#ifndef HOVER_NODE_H
#define HOVER_NODE_H

// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/QuaternionStamped.h>
// #include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <std_msgs/UInt8.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <airsim_ros_pkgs/VelCmd.h>
#include <airsim_ros_pkgs/Takeoff.h>
#include <geometry_msgs/PointStamped.h>
// #include <hector_uav_msgs/EnableMotors.h>


#include <tf/tf.h>

#define C_EARTH (double)6378137.0
#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))


struct XYZYaw
{
    double x;
    double y;
    double z;
    double yaw;
};


class Drone_Mission{
public:
    std::string name;
    std::string VEHICLE_NAME;
    std::string drone_frame;

    int state;
    int substate;
    int sign;
    int counter;
    
    bool rotateToHeading;
    bool finished;
    bool reached_goal_;
    
    float mean_start_gps;
    float Y_POS;
    float X_POS;
    float theta_yaw_diff;
    float mean_start_eas;
    float mean_start_nor;
    float direction;
    float time;
    float timer;
    float register_time;
    float kp_x, kp_y, kp_z, kp_yaw, kd_x, kd_y, kd_z, kd_yaw, reached_thresh_xyz, reached_yaw_degrees;
    
    double max_vel_horz_abs; // meters/sec
    double max_vel_vert_abs;
    double max_yaw_rate_degree;
    double diff_xyz;
    double diff_yaw;
    double roll, pitch, yaw, dist_x, dist_y, theta;
    
    XYZYaw target;
    XYZYaw prev_error_;
    XYZYaw curr_position_;
    XYZYaw curr_error_;
    
    nav_msgs::Odometry subOdom;
    nav_msgs::Odometry curr_odom_;
    
    geometry_msgs::Vector3 test;
    geometry_msgs::PointStamped world_point;
    geometry_msgs::PointStamped local_point;
    
    sensor_msgs::NavSatFix current_gps;
    sensor_msgs::MagneticField current_magnetic;
    
    airsim_ros_pkgs::VelCmd motor_msg;
    airsim_ros_pkgs::VelCmd motor_msg2;
    
    ros::NodeHandle nh;

    ros::Subscriber fix;
    ros::Subscriber magnetic;
    ros::Subscriber imu;
    ros::Subscriber odom;
    ros::Subscriber odom_local;

    ros::Publisher move_drone;
    ros::Publisher move_drone2;
    ros::Publisher tester;

    // void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);

    Drone_Mission() {}

    Drone_Mission(std::string n, const ros::NodeHandle nh_) : name(n), nh(nh_), state(0), substate(0), finished(false), mean_start_gps(0.0), Y_POS(1.0), max_vel_horz_abs(1.0), max_vel_vert_abs(0.5), max_yaw_rate_degree(10.0), counter(0)
    {
        // *nh = nh_;
        target.x = 0.0;
        target.y = 0.0;
        target.z = 0.0;
        kp_x = 0.30;
        kp_y = 0.30;
        kp_z = 0.30;
        kp_yaw = 0.30;
        kd_x = 0.05;
        kd_y = 0.05;
        kd_z = 0.05;
        kd_yaw = 0.05;
        reached_thresh_xyz = 0.1;
        reached_yaw_degrees = 5.0;
        VEHICLE_NAME = name;
        prev_error_.x = 0.0;
        prev_error_.y = 0.0;
        prev_error_.z = 0.0;
        prev_error_.yaw = 0.0;
        diff_xyz = 1000;
        diff_yaw = 1000;
        reached_goal_ = false;
        magnetic = nh.subscribe("/airsim_node/" + name + "/magnetometer/Magnetometer", 10, &Drone_Mission::magneticCallback, this);
        imu = nh.subscribe("/airsim_node/" + name + "/imu/Imu", 10, &Drone_Mission::imuCallback, this);
        odom = nh.subscribe("/airsim_node/" + name + "/odom", 10, &Drone_Mission::odomCallback, this);
        odom_local = nh.subscribe("/airsim_node/" + name + "/odom_local_ned", 10, &Drone_Mission::odomLocalCallback, this);
        move_drone = nh.advertise<airsim_ros_pkgs::VelCmd>("/airsim_node/" + name + "/vel_cmd_body_frame", 1);
        drone_frame = name + "/odom_local_ned";
        world_point.header.frame_id = name;
        world_point.header.stamp = ros::Time(0);
    }

    void imuCallback(const sensor_msgs::ImuConstPtr& msg);
    void odomCallback(const nav_msgs::OdometryConstPtr& msg);
    void odomLocalCallback(const nav_msgs::OdometryConstPtr& msg);
    void magneticCallback(const sensor_msgs::MagneticField::ConstPtr& msg);
    bool takeOff(bool enable);
    void compute_control_cmd();
    void enforce_dynamic_constraints();
    void check_reached_goal();
};

// void drone1_imuCallback(const sensor_msgs::ImuConstPtr& msg);

// void drone2_imuCallback(const sensor_msgs::ImuConstPtr& msg);

void step(Drone_Mission &drone);
void transform_world_to_local(Drone_Mission &drone);
void drone1_gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);
void drone2_gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);
void drone3_gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);
double get_yaw_from_quat_msg(const geometry_msgs::Quaternion& quat_msg);

#endif // HOVER_NODE_H