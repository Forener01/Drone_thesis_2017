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

void create();

std::vector<cv::Mat> AllDoors;
std::vector<cv::Mat> AllSpaces;

thesis_aurian::refdoors refdoors_msg;

cv::Mat M, Doors;

char k;

double scale_factor, thickness_error, angle, scale;

int xx1, yy1, xx2, yy2, height_zoom, door_thickness, xshifts, yshifts,
    init_xshift, my_index, my_J_index, yy1_init, yy2_init, i_angle;

int xx1rec1, yy1rec1, xx2rec1, yy2rec1;
int xx1rec2, yy1rec2, xx2rec2, yy2rec2;
int xx1rec3, yy1rec3, xx2rec3, yy2rec3;

// Image parameters
int img_width = 640;
int img_height = 360;
double door_ratio = 0.425;           // --> W/H = 93.2/201 = real world
                                     // --> W/H = 104/253 = cam world
double door_thickness_ratio = 0.015; // --> T/H = 3/201

// }

#endif // REFERENCE_DOORS_HPP
