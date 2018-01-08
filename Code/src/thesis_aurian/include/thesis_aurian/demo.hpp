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

#include <cv.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <math.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <string>
#include <vector>

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
#define CAM_DEMO 6

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

  // COMPUTER VISION
  void calib_imageCb(const sensor_msgs::ImageConstPtr &msg);
  // void refdoorsCb(const thesis_aurian::refdoors msg);
  void image_processor(const cv::Mat my_img);
  void create_binmasks();

  image_transport::Publisher filtdoor_image_pub, realbindoor_image_pub,
      matchdoor_image_pub;

  cv::Mat converted_img;

private:
  int path_type;
  geometry_msgs::Pose targetpose;

  double speed, hovertime, sleeptime;

  // COMPUTER VISION
  ros::Subscriber calib_image_sub, refdoor_sub;

  cv_bridge::CvImage img_bridge;

  sensor_msgs::ImagePtr filtdoor_msg, realbindoor_msg, matchdoor_msg;

  std_msgs::Header header;

  sensor_msgs::Image current_img, processed_img;

  sensor_msgs::ImagePtr ros_img;

  ros::ServiceClient scan_img_srv;

  cv::Mat redfilt_upp_img, redfilt_low_img, redfilt_dark_img1,
      redfilt_dark_img2, redfilt_lu_img, redfilt_du_img, redfilt_ldu_img,
      redfilt_img, hsv_img, gray_img, redfilt_sub, redfilt_final, hough_img,
      gradBGR, gradBGR_filt, gradGRAY_filt, gradBGR_filt2, grad_canny,
      gradBGR_canny, nofilt_img, RealBinaryDoor, BackgroundGRAY, doorComp,
      InvRealBinaryDoor, spaceComp, testComp, hsv_filt, mergedDoorComp,
      doorCompHSV, mergedDoorComp_temp, HSVDoorComp, hsv_BGRfilt, redfilt_sub2,
      M, RefBinaryDoor, MyDoor, MyOpenSpace;

  char k;

  double rho, theta, minLength, maxLineGap, door_ratio, door_thickness_ratio,
      scale_factor, res, thickness_error, door_tol, space_tol, matchDoor_perc,
      matchSpace_perc, angle, scale;

  int threshold, deg, thickness, xx1, yy1, xx2, yy2, pixel_incr, img_width,
      img_height, height_zoom, door_thickness, xshifts, yshifts, init_xshift,
      my_index, my_J_index, yy1_init, yy2_init, i_angle;

  int refDoorCount, realDoorCount, matchDoorCount;
  int refSpaceCount, realSpaceCount, matchSpaceCount;

  int xx1rec1, yy1rec1, xx2rec1, yy2rec1;
  int xx1rec2, yy1rec2, xx2rec2, yy2rec2;
  int xx1rec3, yy1rec3, xx2rec3, yy2rec3;

  int counter;

  bool keypressed;
};
#endif // DEMO_HPP
