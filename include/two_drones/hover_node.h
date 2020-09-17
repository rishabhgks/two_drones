
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
#include <geometry_msgs/PointStamped.h>

#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/QueryDroneVersion.h>
#include <dji_sdk/SetLocalPosRef.h>
#include <dji_sdk/dji_sdk.h>

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
    
    sensor_msgs::Joy motor_msg;
    
    ros::NodeHandle nh;

    ros::Subscriber fix;
    ros::Subscriber magnetic;
    ros::Subscriber imu;
    ros::Subscriber odom;
    ros::Subscriber odom_local;

    ros::Publisher move_drone;
    ros::Publisher tester;

    sensor_msgs::NavSatFix start_gps_location;
    geometry_msgs::Point start_local_position;

    ros::ServiceClient sdk_ctrl_authority_service; 
    ros::ServiceClient drone_task_service;         
    ros::ServiceClient query_version_service;      
    ros::ServiceClient set_local_pos_reference;

    uint8_t flight_status;
    uint8_t display_mode; 

    uint8_t flag; 
    bool obtain_control_result;
    bool takeoff_result;

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
        // magnetic = nh.subscribe("/airsim_node/" + name + "/magnetometer/Magnetometer", 10, &Drone_Mission::magneticCallback, this);
        imu = nh.subscribe("/frl_uas5/dji_sdk/imu", 10, &Drone_Mission::imuCallback, this);
        odom = nh.subscribe(name + "/geonav_odom", 10, &Drone_Mission::odomCallback, this);
        move_drone = nh.advertise<sensor_msgs::Joy>("/frl_uas5/dji_sdk/flight_control_setpoint_ENUvelocity_yawrate", 1);
        drone_frame = name + "/odom_local_ned";

        // ros::Subscriber attitudeSub = nh.subscribe(name + "/dji_sdk/attitude", 10, &attitude_callback);
        // ros::Subscriber gpsSub      = nh.subscribe(name + "/dji_sdk/gps_position", 10, &gps_callback);
        // ros::Subscriber flightStatusSub = nh.subscribe(name + "/dji_sdk/flight_status", 10, &flight_status_callback);
        // ros::Subscriber displayModeSub = nh.subscribe(name + "/dji_sdk/display_mode", 10, &display_mode_callback);
        // ros::Subscriber localPosition = nh.subscribe(name + "/dji_sdk/local_position", 10, &local_position_callback);

        ros::ServiceClient sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> (name + "/dji_sdk/sdk_control_authority");
        ros::ServiceClient drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>(name + "/dji_sdk/drone_task_control");
        ros::ServiceClient query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>(name + "/dji_sdk/query_drone_version");
        ros::ServiceClient set_local_pos_reference    = nh.serviceClient<dji_sdk::SetLocalPosRef> (name + "/dji_sdk/set_local_pos_ref");

        flight_status = 255;
        display_mode  = 255;

        flag = (DJISDK::VERTICAL_VELOCITY   |
                DJISDK::HORIZONTAL_VELOCITY |
                DJISDK::YAW_RATE            |
                DJISDK::HORIZONTAL_GROUND   |
                DJISDK::STABLE_ENABLE);
        motor_msg.axes.push_back(0);
        motor_msg.axes.push_back(0);
        motor_msg.axes.push_back(0);
        motor_msg.axes.push_back(0);
        motor_msg.axes.push_back(flag);  

        world_point.header.frame_id = name;
        world_point.header.stamp = ros::Time(0);
    }

    void imuCallback(const sensor_msgs::ImuConstPtr& msg);
    void odomCallback(const nav_msgs::OdometryConstPtr& msg);
    void odomLocalCallback(const nav_msgs::OdometryConstPtr& msg);
    // void magneticCallback(const sensor_msgs::MagneticField::ConstPtr& msg);
    void compute_control_cmd();
    void enforce_dynamic_constraints();
    void check_reached_goal();
    bool is_M100();
    bool monitoredTakeoff();
    bool M100monitoredTakeoff();
    bool obtain_control();
    void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg);
    void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg);
    bool takeoff_land(int task);
    void setTarget(float x, float y, float z, float yaw);
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