#ifndef DEMO_HPP
#define DEMO_HPP

#include <ardrone_autonomy/CamSelect.h>
#include <ardrone_autonomy/Navdata.h>
#include <ardrone_velocity_ekf/pose_controller.hpp>
#include <geometry_msgs/Twist.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sstream>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Test types
#define WITHOUT_CONTROL 0
#define VEL_CONTROL 1
#define POSE_CONTROL 2

// Path types
#define STRAIGHTLINE 0
#define SQUARE 1
#define STATIC 2
#define LEFTLINE 3
#define ROOM_EXIT 4
#define FINAL_DEMO 5

bool scan_result, detected;

class Demo {
public:
  Demo();
  ros::NodeHandle nh;

  ros::Publisher vel_pub;
  ros::Publisher takeoff_pub;
  ros::Publisher land_pub;
  ros::Publisher reset_pub;
  ros::Publisher poseref_pub;

  ardrone_autonomy::CamSelect toggle_srv;
  ros::ServiceClient togglecam_client;

  void land(void);
  void takeoff(void);
  void hover(void);
  void load_vel(double linX, double linY, double linZ, double angZ);
  void load_pose(double Xpos, double Ypos, double Zpos);
  void test();
  void finish(void);
  void init(void);

  int test_type;

  bool toggle_succeed, scan_request, scan_result, detected;

private:
  int path_type;
  geometry_msgs::Pose targetpose;

  double speed, hovertime, sleeptime;
};
#endif // DEMO_HPP
