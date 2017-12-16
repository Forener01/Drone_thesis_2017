#ifndef IMAGE_FLOW_HPP
#define IMAGE_FLOW_HPP

#include <ardrone_autonomy/Navdata.h>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>

#include <cv.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <math.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <string>
#include <vector>

class ImageFlow {
public:
  ImageFlow();

  // void imageCb(const sensor_msgs::Image &img_msg);
  void calib_imageCb(const sensor_msgs::ImageConstPtr &msg);
  void image_processor(const cv::Mat my_img);

private:
  ros::Subscriber calib_image_sub;

  image_transport::Publisher processed_image_pub;

  sensor_msgs::Image current_img, processed_img;
  sensor_msgs::ImagePtr ros_img;

  ros::ServiceClient scan_img_srv;

  cv::Mat converted_img, redfilt_upp_img, redfilt_low_img, redfilt_dark_img1,
      redfilt_dark_img2, redfilt_lu_img, redfilt_du_img, redfilt_ldu_img,
      redfilt_img, hsv_img, gray_img, redfilt_sub, redfilt_final, hough_img,
      gradBGR, gradBGR_filt, gradGRAY_filt, gradBGR_filt2, grad_canny,
      gradBGR_canny, nofilt_img, RealBinaryDoor, BackgroundGRAY, doorComp,
      InvRealBinaryDoor, spaceComp, testComp, hsv_filt, mergedDoorComp,
      doorCompHSV, mergedDoorComp_temp, HSVDoorComp, hsv_BGRfilt, redfilt_sub2;

  char k;

  double rho, theta, minLength, maxLineGap, door_ratio, door_thickness_ratio,
      scale_factor, res, thickness_error, door_tol, space_tol, matchDoor_perc,
      matchSpace_perc;

  int threshold, deg, thickness, xx1, yy1, xx2, yy2, pixel_incr, img_width,
      img_height, height_zoom, door_thickness, xshifts, yshifts, init_xshift,
      my_index, my_J_index, yy1_init, yy2_init;

  int refDoorCount, realDoorCount, matchDoorCount;
  int refSpaceCount, realSpaceCount, matchSpaceCount;

  int xx1rec1, yy1rec1, xx2rec1, yy2rec1;
  int xx1rec2, yy1rec2, xx2rec2, yy2rec2;
  int xx1rec3, yy1rec3, xx2rec3, yy2rec3;

  bool keypressed, detected, scan_result;
};

#endif /* IMAGE_FLOW_HPP */