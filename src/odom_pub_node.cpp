#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

sensor_msgs::NavSatFix gps_msg;

sensor_msgs::Imu imu_msg;

void gps_callback(const sensor_msgs::NavSatFixConstPtr& msg);

void imu_callback(const sensor_msgs::ImuConstPtr& msg);

int main(int argc, char **argv)
{

// %Tag(INIT)%
  ros::init(argc, argv, "odom_pub_node");
// %EndTag(INIT)%

// %Tag(NODEHANDLE)%
  ros::NodeHandle n;
// %EndTag(NODEHANDLE)%

// %Tag(PUBLISHER)%
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("nav_odom", 1000);
// %EndTag(PUBLISHER)%

// %Tag(SUBSCRIBER)%
  ros::Subscriber gps = n.subscribe<sensor_msgs::NavSatFix>("dji_sdk/gps_position", 10, gps_callback);

  ros::Subscriber imu = n.subscribe<sensor_msgs::Imu>("dji_sdk/imu", 10, imu_callback);
// %EndTag(SUBSCRIBER)%

  nav_msgs::Odometry odom_msg;

// %Tag(LOOP_RATE)%
  ros::Rate loop_rate(10);
// %EndTag(LOOP_RATE)%

// %Tag(ROS_OK)%
  int count = 0;
  while (ros::ok())
  {
// %EndTag(ROS_OK)%
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
// %Tag(FILL_MESSAGE)%
    odom_msg.header.stamp = ros::Time(0);
    odom_msg.pose.pose.position.x = gps_msg.longitude;
    odom_msg.pose.pose.position.y = gps_msg.latitude;
    odom_msg.pose.pose.position.z = gps_msg.altitude;
    odom_msg.pose.pose.orientation.x = imu_msg.orientation.x;
    odom_msg.pose.pose.orientation.y = imu_msg.orientation.y;
    odom_msg.pose.pose.orientation.z = imu_msg.orientation.z;
    odom_msg.pose.pose.orientation.w = imu_msg.orientation.w;
// %EndTag(FILL_MESSAGE)%

// %Tag(ROSCONSOLE)%
// %EndTag(ROSCONSOLE)%

// %Tag(PUBLISH)%
    odom_pub.publish(odom_msg);
// %EndTag(PUBLISH)%

// %Tag(SPINONCE)%
    ros::spinOnce();
// %EndTag(SPINONCE)%

// %Tag(RATE_SLEEP)%
    loop_rate.sleep();
// %EndTag(RATE_SLEEP)%
    ++count;
  }


  return 0;
}
// %EndTag(FULLTEXT)%

void gps_callback(const sensor_msgs::NavSatFixConstPtr& msg)
{
    gps_msg = *msg;
}

void imu_callback(const sensor_msgs::ImuConstPtr& msg)
{
    imu_msg = *msg;
}