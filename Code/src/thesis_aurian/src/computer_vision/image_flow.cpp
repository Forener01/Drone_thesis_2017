
#include <thesis_aurian/image_flow.hpp>

ImageFlow::ImageFlow() {
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  // Subscribers
  // image_sub =
  //     nh.subscribe("ardrone/front/image_raw", 1, &ImageFlow::imageCb, this);
  calib_image_sub = nh.subscribe("ardrone/front/image_rect_color", 1,
                                 &ImageFlow::calib_imageCb, this);

  // Publishers
  processed_image_pub = it.advertise("processed_image", 1);

  // Image parameters
  img_width = 640;
  img_height = 360;
  door_ratio = 0.425;           // --> W/H = 93.2/201 = real world
                                // --> W/H = 104/253 = cam world
  door_thickness_ratio = 0.015; // --> T/H = 3/201
  keypressed = false;

  // Hough Transform parameters
  deg = 1;
  rho = 1;
  theta = CV_PI / 180 * deg;
  threshold = 71;
  minLength = 32;
  maxLineGap = 11;
  thickness = 1;
}

// void ImageFlow::imageCb(const sensor_msgs::Image &img_msg) {
//   ROS_DEBUG("ImageFlow::imageCb");
//   current_img = img_msg;
// }

void ImageFlow::calib_imageCb(const sensor_msgs::ImageConstPtr &msg) {
  // Conversion from ROS image message to OpenCV Mat image format
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(
      msg, sensor_msgs::image_encodings::BGR8); // 8UC3 = BGR8 format
  converted_img = cv_ptr->image;
  image_processor(converted_img);
}

void ImageFlow::image_processor(const cv::Mat bgr_img) {

  /**********************************
   ********* COLOR FILTERING ********
   **********************************/
  // Applying red-color filtering
  // door_a115: BGR [17, 21, 104];
  // door_lab: BGR [29, 30, 121];
  // BGR 24 18 49 + 27 21 52 + 26 18 48 + 32 23 57 + 13 16 29
  // HSV 339 62 22 + 351 60 17 + 344 52 25 + 328 54 11 + 346 64 18
  // Shelve
  // BGR 34 45 66 + 13 19 35
  // HSV 20 52 34 + 17 58 12

  cv::cvtColor(bgr_img, hsv_img, cv::COLOR_BGR2HSV);
  cv::cvtColor(bgr_img, gray_img, cv::COLOR_BGR2GRAY);
  /* OLD
    cv::inRange(hsv_img, cv::Scalar(155, 95, 35), cv::Scalar(174, 255, 102),
                redfilt_upp_img);
    cv::inRange(hsv_img, cv::Scalar(0, 95, 35), cv::Scalar(5, 255, 102),
                redfilt_low_img);
    cv::inRange(hsv_img, cv::Scalar(0, 0, 0), cv::Scalar(179, 255, 39),
                redfilt_dark_img);
  */

  cv::inRange(hsv_img, cv::Scalar(150, 60, 35), cv::Scalar(174, 255, 102),
              redfilt_upp_img);
  // cv::inRange(hsv_img, cv::Scalar(0, 85, 35), cv::Scalar(5, 255, 102),
  //             redfilt_low_img);
  cv::inRange(hsv_img, cv::Scalar(0, 0, 0), cv::Scalar(25, 255, 45),
              redfilt_dark_img1);
  // cv::inRange(hsv_img, cv::Scalar(8, 135, 0), cv::Scalar(179, 255, 45),
  //             redfilt_dark_img2);

  // Combine the above two images
  // cv::addWeighted(redfilt_low_img, 1.0, redfilt_upp_img, 1.0, 0.0,
  //                 redfilt_lu_img);
  cv::addWeighted(redfilt_dark_img1, 1.0, redfilt_upp_img, 1.0, 0.0,
                  redfilt_du_img);
  // cv::addWeighted(redfilt_du_img, 1.0, redfilt_low_img, 1.0, 0.0,
  //                 redfilt_ldu_img);

  /* to add
    158;84;51
    6.4;56;41
    170;97;46
    172;64;51
    4.48;89;56 */

  /* to withdraw
    6-8;120;43
    83;33;38
    89.5;30.6;17.85
  */

  /* expe total manual:
  H: 0-7 + 170-179
  S: 107-255  ?90
  V: 12-128 ?0-230
  */

  cv::inRange(hsv_img, cv::Scalar(0, 0, 0), cv::Scalar(0, 0, 255), redfilt_sub);
  cv::inRange(hsv_img, cv::Scalar(0, 75, 30), cv::Scalar(15, 145, 50),
              redfilt_sub2);
  redfilt_final = redfilt_du_img - redfilt_sub - redfilt_sub2;
  /* to withdraw outside (hall)
  60 11 7 -> 29 28 17
  0 0 3 -> 0 0 7.65
  0 0 4 -> 0 0 10.2
  0 0 5 -> 0 0 12.75
  0 0 9 -> 0 0 22.95
  */
  // ros_img =
  //     cv_bridge::CvImage(std_msgs::Header(), "HSV8",
  //     redfilt_img).toImageMsg();
  //
  // processed_img = *ros_img;
  // processed_img.width = 640;  /// 2;
  // processed_img.height = 360; /// 2;
  // processed_img.step = 640 * 3;
  // cv::namedWindow("Low-Up Red-filtered image", cv::WINDOW_AUTOSIZE);
  // cv::imshow("Low-Up Red-filtered image", redfilt_lu_img);

  // cv::namedWindow("Upper Red-filtered image", cv::WINDOW_AUTOSIZE);
  // cv::imshow("Upper Red-filtered image", redfilt_upp_img);

  // cv::namedWindow("Lower Red-filtered image", cv::WINDOW_AUTOSIZE);
  // cv::imshow("Lower Red-filtered image", redfilt_low_img);

  // cv::namedWindow("Dark1 Red-filtered image", cv::WINDOW_AUTOSIZE);
  // cv::imshow("Dark1 Red-filtered image", redfilt_dark_img1);

  // cv::namedWindow("Dark2 Red-filtered image", cv::WINDOW_AUTOSIZE);
  // cv::imshow("Dark2 Red-filtered image", redfilt_dark_img2);

  // cv::namedWindow("Dark-Up Red-filtered image", cv::WINDOW_AUTOSIZE);
  // cv::imshow("Dark-Up Red-filtered image", redfilt_du_img);

  // cv::namedWindow("Low-Dark-Up Red-filtered image", cv::WINDOW_AUTOSIZE);
  // cv::imshow("Low-Dark-Up Red-filtered image", redfilt_ldu_img);

  // cv::namedWindow("Sub Red-filtered image", cv::WINDOW_AUTOSIZE);
  // cv::imshow("Sub Red-filtered image", redfilt_sub);

  // cv::namedWindow("Final Red-filtered image", cv::WINDOW_AUTOSIZE);
  // cv::imshow("Final Red-filtered image", redfilt_final);

  // processed_image_pub.publish(processed_img);

  /*********************************
   ********* SOBEL OPERATOR ********
   *********************************/
  int ksize = 3;
  int scale = 1;
  int delta = 0;
  int ddepth_filt = CV_64F;
  cv::Mat grad, grad_filt;
  cv::Mat grad_x_filt, grad_x, grad_y_filt, grad_y;
  cv::Mat abs_grad_x, abs_grad_x_filt, abs_grad_y, abs_grad_y_filt;

  // With color filter
  cv::Sobel(redfilt_final, grad_x_filt, ddepth_filt, 1, 0, ksize, scale, delta,
            cv::BORDER_DEFAULT);
  cv::Sobel(redfilt_final, grad_y_filt, ddepth_filt, 0, 1, ksize, scale, delta,
            cv::BORDER_DEFAULT);
  // converting back to CV_8U
  cv::convertScaleAbs(grad_x_filt, abs_grad_x_filt);
  cv::convertScaleAbs(grad_y_filt, abs_grad_y_filt);
  cv::addWeighted(abs_grad_x_filt, 0.5, abs_grad_y_filt, 0.5, 0, grad_filt);
  // cv::imshow("Sobel - Color-filtered", grad_filt);

  // Without color filter
  int ddepth = CV_8U;
  cv::Sobel(gray_img, grad_x, ddepth, 1, 0, ksize, scale, delta,
            cv::BORDER_DEFAULT);
  cv::Sobel(gray_img, grad_y, ddepth, 0, 1, ksize, scale, delta,
            cv::BORDER_DEFAULT);
  // converting back to CV_8U
  cv::convertScaleAbs(grad_x, abs_grad_x);
  cv::convertScaleAbs(grad_y, abs_grad_y);
  cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);
  // cv::imshow("Sobel - Direct", grad);

  using namespace cv;
  using namespace std;

  /**********************************
   ********* HOUGH TRANSFORM ********
   **********************************/

  if (keypressed) {
    if (k == 'N' || k == 'n') {
      door_ratio += 0.005;
      ROS_INFO("door_ratio = %f", door_ratio);
    } else if (k == 'B' || k == 'b') {
      door_ratio -= 0.005;
      ROS_INFO("door_ratio = %f", door_ratio);
    }

    if (k == 'D' || k == 'd') {
      space_tol += 0.05;
      //   deg += 5;
      //   ROS_INFO("deg = %i", deg);
    } else if (k == 'S' || k == 's') {
      space_tol -= 0.05;
      //   deg -= 5;
      //   ROS_INFO("deg = %i", deg);
    }
    //
    // if (k == 'T' || k == 't') {
    //   threshold += 5;
    //   ROS_INFO("threshold = %i", threshold);
    // } else if (k == 'Y' || k == 'y') {
    //   threshold -= 5;
    //   ROS_INFO("threshold = %i", threshold);
    // }
    //
    if (k == 'L' || k == 'l') {
      thickness += 1;
      ROS_INFO("thickness = %i", thickness);
      //   minLength += 2;
      //   ROS_INFO("minLength = %f", minLength);
    } else if (k == 'K' || k == 'k') {
      thickness -= 1;
      ROS_INFO("thickness = %i", thickness);
      // minLength -= 2;
      //       ROS_INFO("minLength = %f", minLength);
    }
    //
    if (k == 'T' || k == 't') {
      door_tol += 0.02;
      //   maxLineGap += 1;
      //   ROS_INFO("maxLineGap = %f", maxLineGap);
    } else if (k == 'R' || k == 'r') {
      door_tol -= 0.02;
      //   maxLineGap -= 1;
      //   ROS_INFO("maxLineGap = %f", maxLineGap);
    }

    if (k == 'E' || k == 'e') {
      thickness_error += 0.5;
      //   thickness += 1;
      //   ROS_INFO("thickness = %i", thickness);
    } else if (k == 'Z' || k == 'z') {
      thickness_error -= 0.5;
      //   thickness -= 1;
      //   ROS_INFO("thickness = %i", thickness);
    }

    //
    // if (k == 'A' || k == 'a') {
    //   ROS_INFO("rho = %f", rho);
    //   ROS_INFO("deg = %i", deg);
    //   ROS_INFO("threshold = %i", threshold);
    //   ROS_INFO("minLength = %f", minLength);
    //   ROS_INFO("maxLineGap = %f", maxLineGap);
    //   ROS_INFO("thickness = %i", thickness);
    // }

    // // Calibration of the reference door rectangle
    // pixel_incr = 1;
    // if (k == 'X' || k == 'x') {
    //   xx1 += pixel_incr;
    //   ROS_INFO("x1 rectangle coord = %i", xx1);
    // } else if (k == 'W' || k == 'w') {
    //   xx1 -= pixel_incr;
    //   ROS_INFO("x1 rectangle coord = %i", xx1);
    // }
    //
    // if (k == 'B' || k == 'b') {
    //   yy1 += pixel_incr;
    //   ROS_INFO("y1 rectangle coord = %i", yy1);
    // } else if (k == 'V' || k == 'v') {
    //   yy1 -= pixel_incr;
    //   ROS_INFO("y1 rectangle coord = %i", yy1);
    // }
    // if (k == ',' || k == '?') {
    //   xx2 += pixel_incr;
    //   ROS_INFO("x2 rectangle coord = %i", xx2);
    // } else if (k == 'N' || k == 'n') {
    //   xx2 -= pixel_incr;
    //   ROS_INFO("x2 rectangle coord = %i", xx2);
    // }
    //
    // if (k == ':' || k == '/') {
    //   yy2 += pixel_incr;
    //   ROS_INFO("y2 rectangle coord = %i", yy2);
    // } else if (k == ';' || k == '.') {
    //   yy2 -= pixel_incr;
    //   ROS_INFO("y2 rectangle coord = %i", yy2);
    // }

    // Default parameters
  } else {
    // // Hough Transform parameters
    // deg = 1;
    // rho = 1;
    // theta = CV_PI / 180 * deg;
    // threshold = 1;
    // minLength = 1;
    // maxLineGap = 1;
    // thickness = 1;
    // Matching parameters
    thickness_error = 1.0;
    door_tol = 0.36;
    space_tol = 0.92;
  }

  // Set1: threshold = 101; minLength = 66; maxLineGap = 12; thickness rho deg =
  // 1
  // Set2: threshold = 71; minLength = 32; maxLineGap = 11; thickness = 4; rho
  // deg = 1

  // WITHOUT color-filtering
  cvtColor(grad, gradBGR, CV_GRAY2BGR);
  vector<Vec4i> lines;
  HoughLinesP(grad, lines, rho, theta, threshold, minLength, maxLineGap);
  for (size_t i = 0; i < lines.size(); i++) {
    Vec4i l = lines[i];
    line(gradBGR, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255),
         thickness, CV_AA);
  }
  // imshow("Hough Transform", gradBGR);

  // WITH color-filtering
  cvtColor(grad_filt, gradBGR_filt, CV_GRAY2BGR);
  // Mat Blank(grad_filt.rows, grad_filt.cols, CV_8UC3, Scalar(0, 0, 0));
  vector<Vec4i> lines_filt;
  HoughLinesP(grad_filt, lines_filt, rho, theta, threshold, minLength,
              maxLineGap);
  for (size_t i = 0; i < lines_filt.size(); i++) {
    Vec4i l = lines_filt[i];
    line(gradBGR_filt, Point(l[0], l[1]), Point(l[2], l[3]),
         Scalar(255, 255, 255), thickness, CV_AA);
    // line(Blank, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255),
    //      thickness, CV_AA);
  }

  // imshow("Hough Transform with color-filtering", gradBGR_filt);

  // Canny WITHOUT color-filtering
  Canny(gray_img, grad_canny, 50, 200, 3);
  cvtColor(grad_canny, gradBGR_canny, CV_GRAY2BGR);
  Mat Black2(grad_canny.rows, grad_canny.cols, CV_8UC3, Scalar(0, 0, 0));
  vector<Vec4i> lines_canny;
  HoughLinesP(grad_canny, lines_canny, rho, theta, threshold, minLength,
              maxLineGap);

  for (size_t i = 0; i < lines_canny.size(); i++) {
    Vec4i l = lines_canny[i];
    line(gradBGR_canny, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255),
         thickness, CV_AA);
    line(Black2, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255),
         thickness, CV_AA);
  }

  // imshow("Hough Transform with Canny", Black2);

  // Conversion into binary image
  cvtColor(gradBGR_filt, gradGRAY_filt, COLOR_BGR2GRAY);
  cv::threshold(gradGRAY_filt, RealBinaryDoor, 100, 255, cv::THRESH_BINARY);
  imshow("RealBinaryDoor", RealBinaryDoor);

  /**********************************
   ********* MATCHING STEP **********
   **********************************/
  // Creation of the reference image
  /* Linetype:
      8 (or omitted) - 8-connected line.
      4 - 4-connected line.
      CV_AA - antialiased line.
  */

  scale_factor = 0.714; // Out: 0.714; // 1 = door takes the full height
  height_zoom = scale_factor * img_height;
  door_thickness = door_thickness_ratio * height_zoom * thickness_error;
  init_xshift = 140 - (door_ratio * height_zoom / 2 + 3 * door_thickness);
  xshifts = 2 * init_xshift;
  ROS_INFO("xshifts %i", xshifts);
  yshifts = 5;

  xx1 = img_width / 2 - door_ratio * height_zoom / 2 - door_thickness / 2 - 1 +
        init_xshift;
  yy1 = img_height + door_thickness / 2;
  yy1_init = yy1;
  xx2 = img_width / 2 + door_ratio * height_zoom / 2 + door_thickness / 2 - 1 +
        init_xshift;
  yy2 = yy1 - height_zoom - door_thickness / 2;
  yy2_init = yy2;

  // Coordinates for lateral space
  xx1rec1 = xx2 + door_thickness / 2;
  xx2rec1 = xx1rec1 + 2 * door_thickness;
  yy1rec1 = yy1 - door_thickness / 2;
  yy2rec1 = yy2 - 5 * door_thickness / 2;

  // Coordinates for superior space
  xx1rec2 = xx1 + door_thickness / 2;
  xx2rec2 = xx2 + door_thickness / 2;
  yy1rec2 = yy2 - door_thickness / 2;
  yy2rec2 = yy1rec2 - 2 * door_thickness;

  // Coordinates for interior space
  xx1rec3 = xx1 + door_thickness / 2;
  xx2rec3 = xx2 - door_thickness / 2;
  yy1rec3 = yy1 + door_thickness / 2;
  yy2rec3 = yy1rec3 - 0.1866 * height_zoom;
  // Mat Background(grad_filt.rows, grad_filt.cols, CV_8UC3, Scalar(0,
  // 0,
  // 0));
  // rectangle(Background, Point(xx1, yy1), Point(xx2, yy2), Scalar(255,
  // 255,
  // 255),
  //           door_thickness, 8);
  //
  // Conversion into binary door reference image
  // cvtColor(Background, BackgroundGRAY, COLOR_BGR2GRAY);
  // cv::threshold(BackgroundGRAY, RefBinaryDoor, 100, 255,
  // cv::THRESH_BINARY);

  Mat RefBinaryDoor(RealBinaryDoor.rows, RealBinaryDoor.cols, CV_8UC1,
                    Scalar(0));
  rectangle(RefBinaryDoor, Point(xx1, yy1), Point(xx2, yy2), Scalar(255),
            door_thickness, 8);
  line(RefBinaryDoor, Point(xx1, yy1), Point(xx2, yy1), Scalar(0),
       door_thickness, 8);

  refDoorCount = sum(RefBinaryDoor)[0] / 255;
  realDoorCount = sum(RealBinaryDoor)[0] / 255;
  multiply(RealBinaryDoor / 255, RefBinaryDoor / 255, doorComp);
  matchDoorCount = sum(doorComp)[0];
  matchDoor_perc = (double)matchDoorCount / (double)refDoorCount;

  Mat RefSpace(RealBinaryDoor.rows, RealBinaryDoor.cols, CV_8UC1, Scalar(0));
  rectangle(RefSpace, Point(xx1rec1, yy1rec1), Point(xx2rec1, yy2rec1),
            Scalar(255), CV_FILLED, 8);
  rectangle(RefSpace, Point(xx1rec2, yy1rec2), Point(xx2rec2, yy2rec2),
            Scalar(255), CV_FILLED, 8);
  rectangle(RefSpace, Point(xx1rec3, yy1rec3), Point(xx2rec3, yy2rec3),
            Scalar(255), CV_FILLED, 8);

  cv::threshold(RealBinaryDoor, InvRealBinaryDoor, 100, 255, THRESH_BINARY_INV);

  refSpaceCount = sum(RefSpace)[0] / 255;
  realSpaceCount = sum(InvRealBinaryDoor)[0] / 255;
  multiply(InvRealBinaryDoor / 255, RefSpace / 255, spaceComp);
  matchSpaceCount = sum(spaceComp)[0];
  matchSpace_perc = (double)matchSpaceCount / (double)refSpaceCount;

  // ROS_INFO("before: RefDoor count %i", refDoorCount);
  // ROS_INFO("before: RealDoor count %i", realDoorCount);
  // ROS_INFO("before: MatchDoor Count %i", matchDoorCount);
  ROS_INFO("------------------------------------");
  ROS_INFO("before: Door perc %f", matchDoor_perc);
  ROS_INFO("before: Space perc %f", matchSpace_perc);
  ROS_INFO("------------------------------------");
  // ROS_INFO("------------------------------------");
  // ROS_INFO("before: RefSpace count %i", refSpaceCount);
  // ROS_INFO("before: RealSpace count %i", realSpaceCount);
  // ROS_INFO("before: MatchSpace Count %i", matchSpaceCount);
  // ROS_INFO("------------------------------------");
  // ROS_INFO("before: MatchSpace perc %f", matchSpace_perc);

  //  Matching decision
  for (my_index = 0; my_index < xshifts; my_index++) {
    xx1 -= 1;
    xx2 -= 1;
    yy1 = yy1_init;
    yy2 = yy2_init;
    // Coordinates for lateral space
    xx1rec1 = xx2 + door_thickness / 2;
    xx2rec1 = xx1rec1 + 2 * door_thickness;
    yy1rec1 = yy1 - door_thickness / 2;
    yy2rec1 = yy2 - 5 * door_thickness / 2;

    // Coordinates for superior space
    xx1rec2 = xx1 + door_thickness / 2;
    xx2rec2 = xx2 + door_thickness / 2;
    yy1rec2 = yy2 - door_thickness / 2;
    yy2rec2 = yy1rec2 - 2 * door_thickness;

    // Coordinates for interior space
    xx1rec3 = xx1 + door_thickness / 2;
    xx2rec3 = xx2 - door_thickness / 2;
    yy1rec3 = yy1 + door_thickness / 2;
    yy2rec3 = yy1rec3 - 0.1866 * height_zoom;

    for (my_J_index = 0; my_J_index < yshifts; my_J_index++) {
      yy1 -= 1;
      yy2 -= 1;

      // Door comparison
      RefBinaryDoor.operator=(Scalar(0)); // Draw a new shifted reference door
      rectangle(RefBinaryDoor, Point(xx1, yy1), Point(xx2, yy2), Scalar(255),
                door_thickness, 8);
      line(RefBinaryDoor, Point(xx1, yy1), Point(xx2, yy1), Scalar(0),
           door_thickness, 8);

      refDoorCount = sum(RefBinaryDoor)[0] / 255;
      realDoorCount = sum(RealBinaryDoor)[0] / 255;
      multiply(RealBinaryDoor / 255, RefBinaryDoor / 255, doorComp);
      matchDoorCount = sum(doorComp)[0];
      matchDoor_perc = (double)matchDoorCount / (double)refDoorCount;

      // Open space comparison
      RefSpace.operator=(Scalar(0));
      rectangle(RefSpace, Point(xx1rec1, yy1rec1), Point(xx2rec1, yy2rec1),
                Scalar(255), CV_FILLED, 8);
      rectangle(RefSpace, Point(xx1rec2, yy1rec2), Point(xx2rec2, yy2rec2),
                Scalar(255), CV_FILLED, 8);
      rectangle(RefSpace, Point(xx1rec3, yy1rec3), Point(xx2rec3, yy2rec3),
                Scalar(255), CV_FILLED, 8);

      refSpaceCount = sum(RefSpace)[0] / 255;
      realSpaceCount = sum(InvRealBinaryDoor)[0] / 255;
      multiply(InvRealBinaryDoor / 255, RefSpace / 255, spaceComp);
      matchSpaceCount = sum(spaceComp)[0];
      matchSpace_perc = (double)matchSpaceCount / (double)refSpaceCount;

      if ((matchDoor_perc >= door_tol) && (matchSpace_perc >= space_tol)) {
        // ROS_INFO("------------------------------------");
        // ROS_INFO("DETECTED: Matchperc %f", match_perc);
        // ROS_INFO("------------------------------------");
        //   detection_pub.publish(detected);
        break;
      }
    }

    if ((matchDoor_perc >= door_tol) && (matchSpace_perc >= space_tol)) {
      ROS_INFO("------------------------------------");
      ROS_INFO("DETECTED: Door perc %f", matchDoor_perc);
      ROS_INFO("DETECTED: Space perc %f", matchSpace_perc);
      ROS_INFO("------------------------------------");
      //   detection_pub.publish(detected);
      break;
    }
  }

  // ROS_INFO("after: yy1 %i", yy1);
  // ROS_INFO("after: xx1 %i", xx1);
  ROS_INFO("Index %i", my_index);
  // ROS_INFO("after: RefDoor count %i", refDoorCount);
  // ROS_INFO("after: RealDoor count %i", realDoorCount);
  // ROS_INFO("after: MatchDoor Count %i", matchDoorCount);
  ROS_INFO("------------------------------------");
  ROS_INFO("after: Door perc %f", matchDoor_perc);
  ROS_INFO("after: Space perc %f", matchSpace_perc);
  ROS_INFO("------------------------------------");
  // ROS_INFO("------------------------------------");
  // ROS_INFO("after: RefSpace count %i", refSpaceCount);
  // ROS_INFO("after: RealSpace count %i", realSpaceCount);
  // ROS_INFO("after: MatchSpace Count %i", matchSpaceCount);
  // ROS_INFO("------------------------------------");
  // ROS_INFO("after: MatchSpace perc %f", matchSpace_perc);

  ROS_INFO("------------------------------------");
  ROS_INFO("Thickness_error: %f", thickness_error);
  ROS_INFO("Door_tol: %f", door_tol);
  ROS_INFO("Space_tol: %f", space_tol);
  //
  imshow("Match door", doorComp * 255);
  imshow("Match space", spaceComp * 255);
  imshow("Ref space", RefSpace);
  // // imshow("Reference door rectangle", Background);
  // // imshow("Reference GRAY door rectangle", BackgroundGRAY);
  imshow("Reference BIN door rectangle", RefBinaryDoor);

  // Sampling HSV values for the door

  k = waitKey(0);
  keypressed = true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "image_flow");
  ImageFlow imageflownode;
  ros::Rate loop_rate(1);

  ROS_INFO_STREAM("image_flow node started!");

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
