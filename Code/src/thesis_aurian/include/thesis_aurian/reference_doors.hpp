#ifndef REFERENCE_DOORS_HPP
#define REFERENCE_DOORS_HPP

#include <thesis_aurian/demo.hpp>

#include <ros/package.h>
#include <ros/ros.h>
#include <sstream>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Computer vision
#include <cv.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <math.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

// message
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <thesis_aurian/refdoors.h>

class Reference_doors {
public:
  Reference_doors();
  ros::NodeHandle nh;
  ros::Publisher refdoor_pub;

  void create();

  thesis_aurian::refdoors refdoors_msg;

private:
  cv::Mat M, Doors;

  char k;

  double door_ratio, door_thickness_ratio, scale_factor, thickness_error, angle,
      scale;

  int xx1, yy1, xx2, yy2, img_width, img_height, height_zoom, door_thickness,
      xshifts, yshifts, init_xshift, my_index, my_J_index, yy1_init, yy2_init,
      i_angle;

  int xx1rec1, yy1rec1, xx2rec1, yy2rec1;
  int xx1rec2, yy1rec2, xx2rec2, yy2rec2;
  int xx1rec3, yy1rec3, xx2rec3, yy2rec3;

  int counter;

  bool keypressed;
};
#endif // REFERENCE_DOORS_HPP
