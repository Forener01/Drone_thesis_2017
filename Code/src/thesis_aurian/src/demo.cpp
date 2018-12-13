#include <ardrone_velocity_ekf/pose_controller.hpp>
#include <thesis_aurian/demo.hpp> 

Demo::Demo() {
  ros::NodeHandle nh;

  ros::param::get("~test_type", test_type);
  ros::param::get("~path_type", path_type);
  ros::param::get("~the_speed", speed);
  ros::param::get("~the_hovertime", hovertime);
  ros::param::get("~the_sleeptime", sleeptime);

  // Subscribers
  // calib_image_sub = nh.subscribe("ardrone/front/image_rect_color", 1,
  //                                &Demo::calib_imageCb, this);

  // refdoor_sub = nh.subscribe("reference_doors", 1, &Demo::refdoorsCb, this);

  // Publishers
  takeoff_pub = nh.advertise<std_msgs::Empty>("ardrone/takeoff", 1);
  land_pub = nh.advertise<std_msgs::Empty>("ardrone/land", 1);
  reset_pub = nh.advertise<std_msgs::Empty>("ardrone/reset", 1);

  poseref_pub = nh.advertise<geometry_msgs::Pose>("cmd_pose_ref", 1);
  ROS_INFO("Demo node publishing to cmd_pose_ref topic !");

  if (test_type == WITHOUT_CONTROL) {
    vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",
                                                 1); // without controller
    // vel_pub2 = nh.advertise<geometry_msgs::Twist>("cmd_PID_topic", 1000);
    ROS_INFO("Vel_pub connected to topic cmd_vel");
  }

  else {
    vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_ref",
                                                 1); // with EKF Controller
    ROS_INFO("Demo node publishing to cmd_vel_ref topic !");
  }

  // else if (test_type == POSE_CONTROL) {
  //   poseref_pub = nh.advertise<geometry_msgs::Pose>("cmd_pose_ref", 1);
  //   ROS_INFO("Demo node publishing to cmd_pose_ref topic !");
  // }
  //
  // else {
  //   ROS_WARN("No test_type given !");
  // }

  // COMPUTER VISION
  image_transport::ImageTransport it(nh);
  filtdoor_image_pub = it.advertise("filtdoor_image", 1);
  realbindoor_image_pub = it.advertise("realbindoor_image", 1);
  matchdoor_image_pub = it.advertise("matchdoor_image", 1);

  // Client
  togglecam_client =
      nh.serviceClient<ardrone_autonomy::CamSelect>("ardrone/setcamchannel");
  toggle_srv.request.channel = 1;
  ROS_INFO("BOTTOM CAM active !");
  // Other parameters
  detected = false;
  scan_result = false;
  scan_request = false;

  // Image parameters
  // img_width = 640;
  // img_height = 360;
  // door_ratio = 0.425;           // --> W/H = 93.2/201 = real world
  //                               // --> W/H = 104/253 = cam world
  // door_thickness_ratio = 0.015; // --> T/H = 3/201
  keypressed = false;

  // Hough Transform parameters
  deg = 1;
  rho = 1;
  theta = CV_PI / 180 * deg;
  threshold = 71;
  minLength = 32;
  maxLineGap = 11;
  thickness = 1;

  counter = 0;
}

void Demo::calib_imageCb(const sensor_msgs::ImageConstPtr &msg) {
  // Conversion from ROS image message to OpenCV Mat image format
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(
      msg, sensor_msgs::image_encodings::BGR8); // 8UC3 = BGR8 format
  converted_img = cv_ptr->image;
  // image_processor(converted_img);
  if (scan_request) {
    image_processor(converted_img);
  }
}

// void Demo::refdoorsCb(const thesis_aurian::refdoors msg) {
//   // std::vector<int>::const_iterator RefBinaryDoor = msg.data;
//
//   imshow("RefBinaryDoor", mySharedmatrix);
// }

void Demo::image_processor(const cv::Mat bgr_img) {
  // if (scan_request) {
  ROS_INFO("IMAGE PROCESSOR STARTS !");
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

  // Set1: threshold = 101; minLength = 66; maxLineGap = 12; thickness rho deg
  // =
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

  /**********************************
   ********* MATCHING STEP **********
   **********************************/
  // Creation of the reference image
  /* Linetype:
      8 (or omitted) - 8-connected line.
      4 - 4-connected line.
      CV_AA - antialiased line.
  */

  scale_factor = 1; // Out: 0.714; // 1 = door takes the full height
  height_zoom = scale_factor * img_height;
  door_thickness = door_thickness_ratio * height_zoom * thickness_error;
  init_xshift = 140 - (door_ratio * height_zoom / 2 + 3 * door_thickness);
  xshifts = 2 * init_xshift;
  // ROS_INFO("xshifts %i", xshifts);
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

  // Rotation
  scale = 1.0;

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
  // ROS_INFO("------------------------------------");
  // ROS_INFO("before: Door perc %f", matchDoor_perc);
  // ROS_INFO("before: Space perc %f", matchSpace_perc);
  // ROS_INFO("------------------------------------");
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
      //   for (i_angle = -5; i_angle < 6; i_angle++) {
      //     angle = (double)i_angle;
      //     M = getRotationMatrix2D(Point(320, 180), angle, 1.0);

      // Door comparison
      RefBinaryDoor.operator=(Scalar(0)); // Draw a new shifted reference door
      rectangle(RefBinaryDoor, Point(xx1, yy1), Point(xx2, yy2), Scalar(255),
                door_thickness, 8);
      line(RefBinaryDoor, Point(xx1, yy1), Point(xx2, yy1), Scalar(0),
           door_thickness, 8);
      // warpAffine(RefBinaryDoor, RefBinaryDoor, M, RealBinaryDoor.size());

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
      // warpAffine(RefSpace, RefSpace, M, RealBinaryDoor.size());

      refSpaceCount = sum(RefSpace)[0] / 255;
      realSpaceCount = sum(InvRealBinaryDoor)[0] / 255;
      multiply(InvRealBinaryDoor / 255, RefSpace / 255, spaceComp);
      matchSpaceCount = sum(spaceComp)[0];
      matchSpace_perc = (double)matchSpaceCount / (double)refSpaceCount;

      if ((matchDoor_perc >= door_tol) && (matchSpace_perc >= space_tol)) {
        ROS_INFO("------------------------------------");
        ROS_INFO("DETECTED: Door perc %f", matchDoor_perc);
        ROS_INFO("DETECTED: Space perc %f", matchSpace_perc);
        ROS_INFO("------------------------------------");
        detected = true;
        break;
      }
    }

    //   if (detected) {
    //     break;
    //   }
    // }

    if (detected) {
      break;
    }
  }

  if (!detected) {
    ROS_INFO("------------------------------------");
    ROS_INFO("NOT detected: Door perc %f", matchDoor_perc);
    ROS_INFO("NOT detected: Space perc %f", matchSpace_perc);
    ROS_INFO("------------------------------------");
  }

  detected = false;
  header.seq = counter;            // user defined counter
  header.stamp = ros::Time::now(); // time

  filtdoor_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", redfilt_final)
                     .toImageMsg();

  realbindoor_msg =
      cv_bridge::CvImage(std_msgs::Header(), "mono8", RealBinaryDoor)
          .toImageMsg();

  matchdoor_msg =
      cv_bridge::CvImage(std_msgs::Header(), "mono8", doorComp * 255)
          .toImageMsg();

  filtdoor_image_pub.publish(filtdoor_msg);

  realbindoor_image_pub.publish(realbindoor_msg);

  matchdoor_image_pub.publish(matchdoor_msg);

  scan_result = true;
  counter++;
  // ROS_INFO("after: yy1 %i", yy1);
  // ROS_INFO("after: xx1 %i", xx1);
  // ROS_INFO("Index %i", my_index);
  // ROS_INFO("after: RefDoor count %i", refDoorCount);
  // ROS_INFO("after: RealDoor count %i", realDoorCount);
  // ROS_INFO("after: MatchDoor Count %i", matchDoorCount);
  // ROS_INFO("------------------------------------");
  // ROS_INFO("after: Door perc %f", matchDoor_perc);
  // ROS_INFO("after: Space perc %f", matchSpace_perc);
  // ROS_INFO("------------------------------------");
  // ROS_INFO("------------------------------------");
  // ROS_INFO("after: RefSpace count %i", refSpaceCount);
  // ROS_INFO("after: RealSpace count %i", realSpaceCount);
  // ROS_INFO("after: MatchSpace Count %i", matchSpaceCount);
  // ROS_INFO("------------------------------------");
  // ROS_INFO("after: MatchSpace perc %f", matchSpace_perc);

  // ROS_INFO("------------------------------------");
  // ROS_INFO("Thickness_error: %f", thickness_error);
  // ROS_INFO("Door_tol: %f", door_tol);
  // ROS_INFO("Space_tol: %f", space_tol);

  // imshow("RealBinaryDoor", RealBinaryDoor);
  // imshow("Match door", doorComp * 255);
  // imshow("Match space", spaceComp * 255);
  // imshow("Ref space", RefSpace);
  // // imshow("Reference door rectangle", Background);
  // // imshow("Reference GRAY door rectangle", BackgroundGRAY);
  // imshow("Reference BIN door rectangle", RefBinaryDoor);

  // k = waitKey(0);
  // keypressed = true;
  ROS_INFO("IMAGE PROCESSOR STOPS !");
  ROS_INFO("------------------------------------");
  ROS_INFO("------------------------------------");
  ROS_INFO("------------------------------------");

  /** IMPLEMENTATION ONE SHOT */
  refDoorCount = sum(RefBinaryDoor)[0] / 255;
  realDoorCount = sum(RealBinaryDoor)[0] / 255;
  multiply(RealBinaryDoor / 255, RefBinaryDoor / 255, doorComp);
  matchDoorCount = sum(doorComp)[0];
  matchDoor_perc = (double)matchDoorCount / (double)refDoorCount;
}
// }

void Demo::land(void) { land_pub.publish(std_msgs::Empty()); }

void Demo::takeoff(void) { takeoff_pub.publish(std_msgs::Empty()); }

// This function sets the hover mode.
void Demo::hover(void) {
  geometry_msgs::Twist cmd;

  cmd.linear.x = 0.0;
  cmd.linear.y = 0.0;
  cmd.linear.z = 0.0;
  cmd.angular.z = 0.0;

  cmd.angular.x = 0.0;
  cmd.angular.y = 0.0;

  vel_pub.publish(cmd);
}

// This function sets new velocities values and publishes them to navdata.
void Demo::load_vel(double linX, double linY, double linZ, double angZ) {
  geometry_msgs::Twist cmd;

  cmd.linear.x = linX;
  cmd.linear.y = linY;
  cmd.linear.z = linZ;
  cmd.angular.z = angZ;

  cmd.angular.x = 0.0;
  cmd.angular.y = 0.0;

  vel_pub.publish(cmd);
}

void Demo::load_pose(double Xpos, double Ypos, double Zpos) {
  targetpose.position.x = Xpos;
  targetpose.position.y = Ypos;
  targetpose.position.z = Zpos;

  poseref_pub.publish(targetpose);
  // ros::spinOnce();
}

void Demo::test() {
  if (test_type == WITHOUT_CONTROL) {
    if (path_type == STRAIGHTLINE) {
      ROS_INFO("The drone starts the path-planning without "
               "control following a straight line during %f sec at %f m/s",
               sleeptime, speed);
      load_vel(speed, 0.0, 0.0, 0.0);
      ros::Duration(sleeptime).sleep();
      ROS_INFO_STREAM_ONCE("End of the line");
    }

    else if (path_type == LEFTLINE) {
      ROS_INFO("The drone starts the path-planning without "
               "control following a left line during %f sec at %f m/s",
               sleeptime, speed);
      load_vel(0.0, speed, 0.0, 0.0);
      ros::Duration(sleeptime).sleep();
      ROS_INFO_STREAM_ONCE("End of the line");
    }

    else if (path_type == SQUARE) {
      // Moving to the left #1
      ROS_INFO_STREAM_ONCE(
          "The drone starts the path-planning without control !");
      load_vel(0.0, speed, 0.0, 0.0);
      ros::Duration(sleeptime).sleep();
      ROS_INFO_STREAM_ONCE("Corner #1");
      // Stabilizing #1
      hover();
      ros::Duration(hovertime).sleep();

      // Moving backward #2
      load_vel(-speed, 0.0, 0.0, 0.0);
      ros::Duration(sleeptime).sleep();
      ROS_INFO_STREAM_ONCE("Corner #2");
      // Stabilizing #2
      hover();
      ros::Duration(hovertime).sleep();

      // Moving to the right #3
      load_vel(0.0, -speed, 0.0, 0.0);
      ros::Duration(sleeptime).sleep();
      ROS_INFO_STREAM_ONCE("Corner #3");
      // Stabilizing #3
      hover();
      ros::Duration(hovertime).sleep();

      // Moving forward #4
      load_vel(speed, 0.0, 0.0, 0.0);
      ros::Duration(sleeptime).sleep();
      ROS_INFO_STREAM_ONCE("Corner #4");
      // Stabilizing #4
      hover();
      ros::Duration(hovertime).sleep();
    }

    else if (path_type == STATIC) {
      ROS_INFO_STREAM_ONCE("The drone starts hovering without control !");
      hover();
      ros::Duration(sleeptime).sleep();
    }
  }

  else if (test_type == VEL_CONTROL) {
    if (path_type == STRAIGHTLINE) {
      ROS_INFO("The drone starts the path-planning with velocity "
               "control following a straight line during %f sec at %f m/s",
               sleeptime, speed);
      load_vel(speed, 0.0, 0.0, 0.0);
      ros::Duration(sleeptime).sleep();
      ROS_INFO_STREAM_ONCE("End of the line");
    }

    else if (path_type == LEFTLINE) {
      ROS_INFO("The drone starts the path-planning with velocity "
               "control following a left line during %f sec at %f m/s",
               sleeptime, speed);

      load_vel(0.0, speed, 0.0, 0.0);
      ros::Duration(sleeptime).sleep();
      hover();
      ROS_INFO("Scan #1");
      ros::Duration(hovertime).sleep();
      //   // correction yaw
      //   load_vel(0.0, 0.0, 0.0, -0.02);
      //   ros::Duration(1.0).sleep();

      load_vel(0.0, speed, 0.0, 0.0);
      ros::Duration(sleeptime).sleep();
      hover();
      ROS_INFO("Scan #2");
      ros::Duration(4.0).sleep();
      // correction yaw
      load_vel(0.0, 0.0, 0.0, -0.02);
      ros::Duration(1.0).sleep();

      load_vel(0.0, speed, 0.0, 0.0);
      ros::Duration(sleeptime).sleep();
      hover();
      ROS_INFO("Scan #3");
      ros::Duration(hovertime).sleep();
      //   // correction yaw
      //   load_vel(0.0, 0.0, 0.0, -0.02);
      //   ros::Duration(1.0).sleep();

      load_vel(0.0, speed, 0.0, 0.0);
      ros::Duration(sleeptime).sleep();
      hover();
      ROS_INFO("Scan #4");
      ros::Duration(hovertime).sleep();
      //   // correction yaw
      //   load_vel(0.0, 0.0, 0.0, -0.02);
      //   ros::Duration(1.0).sleep();
      //
      //   load_vel(0.0, speed, 0.0, 0.0);
      //   ros::Duration(sleeptime).sleep();
      //   hover();
      //   ROS_INFO("Scan #5");
      //   ros::Duration(hovertime).sleep();
      //
      //   load_vel(0.0, speed, 0.0, 0.0);
      //   ros::Duration(sleeptime).sleep();
      //   hover();
      //   ROS_INFO("Scan #6");
      //   ros::Duration(hovertime).sleep();
      //
      //   load_vel(0.0, speed, 0.0, 0.0);
      //   ros::Duration(sleeptime).sleep();
      //   hover();
      //   ROS_INFO("Scan #7");
      //   ros::Duration(hovertime).sleep();
      //
      //   load_vel(0.0, speed, 0.0, 0.0);
      //   ros::Duration(sleeptime).sleep();
      //   hover();
      //   ROS_INFO("Scan #8");
      //   ros::Duration(hovertime).sleep();
      //
      //   load_vel(0.0, speed, 0.0, 0.0);
      //   ros::Duration(sleeptime).sleep();
      //   hover();
      //   ROS_INFO("Scan #9");
      //   ros::Duration(hovertime).sleep();

      ROS_INFO_STREAM_ONCE("End of the line");

      load_vel(0.7, 0.0, 0.0, 0.0);
      ros::Duration(6.74).sleep();

      finish();
    }

    else if (path_type == SQUARE) {
      // Moving to the left #1
      ROS_INFO_STREAM_ONCE("The drone starts the path-planning with velocity "
                           "control following a square !");
      load_vel(0.0, speed, 0.0, 0.0);
      ros::Duration(sleeptime).sleep();
      ROS_INFO_STREAM_ONCE("Corner #1");
      // Stabilizing #1
      hover();
      ros::Duration(hovertime).sleep();

      // Moving backward #2
      load_vel(-speed, 0.0, 0.0, 0.0);
      ros::Duration(sleeptime).sleep();
      ROS_INFO_STREAM_ONCE("Corner #2");
      // Stabilizing #2
      hover();
      ros::Duration(hovertime).sleep();

      // Moving to the right #3
      load_vel(0.0, -speed, 0.0, 0.0);
      ros::Duration(sleeptime).sleep();
      ROS_INFO_STREAM_ONCE("Corner #3");
      // Stabilizing #3
      hover();
      ros::Duration(hovertime).sleep();

      // Moving forward #4
      load_vel(speed, 0.0, 0.0, 0.0);
      ros::Duration(sleeptime).sleep();
      ROS_INFO_STREAM_ONCE("Corner #4");
      // Stabilizing #4
      hover();
      ros::Duration(hovertime).sleep();
    }

    else if (path_type == STATIC) {
      ROS_INFO_STREAM_ONCE("The drone starts hovering with velocity control !");
      hover();
      ros::Duration(sleeptime).sleep();
    }

    else if (path_type == ROOM_EXIT) {
      ROS_INFO_STREAM_ONCE("The drone scans the room and exits.");
      //   // Corner #1
      //   load_vel(speed, 0.0, 0.0, 0.0);
      //   ros::Duration(8.0).sleep();
      //   ROS_INFO_STREAM_ONCE("Corner #1");
      //   // Stabilizing #1
      //   hover();
      //   ros::Duration(hovertime).sleep();

      //   // Corner #2
      //   load_vel(0.0, 0.25, 0.0, 0.0);
      //   ros::Duration(18.0).sleep();

      //   load_vel(0.0, 0.15, 0.0, 0.0);
      //   ros::Duration(7.5).sleep();
      //   ROS_INFO_STREAM_ONCE("Corner #2");
      //   // Stabilizing #2
      //   hover();
      //   ros::Duration(0.05).sleep();

      //   // Rotate 90° leftward
      //   load_vel(0.0, 0.0, 0.0, 0.325);
      //   ros::Duration(2.75).sleep();
      //   ROS_INFO_STREAM_ONCE("Rotation done !");
      //   hover();
      //   ros::Duration(0.5).sleep();

      // Corner #3
      //   load_vel(0.2, 0.0, 0.0, 0.0);
      //   ros::Duration(7.5).sleep();
      ROS_INFO_STREAM_ONCE("Going through the door");
      //   load_vel(0.15, 0.0, 0.0, 0.0);
      //   ros::Duration(3.0).sleep();
      //
      //   load_vel(0.25, 0.0, 0.0, 0.0);
      //   ros::Duration(3.0).sleep();
      /* distance = 4.72 m, porte à 2.25m
        0->
      */
      // One shot
      load_vel(0.7, 0.0, 0.0, 0.0);
      ros::Duration(6.74).sleep();

      // Trapezoïdal
      //   load_vel(0.2, 0.0, 0.0, 0.0);
      //   ros::Duration(5.54).sleep();
      //   load_vel(0.45, 0.0, 0.0, 0.0);
      //   ros::Duration(5.54).sleep();
      //
      //   load_vel(0.7, 0.0, 0.0, 0.0);
      //   ros::Duration(3.75).sleep();
    }

    else if (path_type == FINAL_DEMO) {
      ROS_INFO_STREAM_ONCE("DEMO STARTED !");
      //   ROS_INFO("------------ NOW ---------------");
      //   ros::Duration(25.0).sleep();
      //   scan_request = true;
      if (toggle_srv.request.channel != 0) {
        toggle_srv.request.channel = 0; // switch to front camera
        toggle_succeed = togglecam_client.call(toggle_srv);
        ROS_INFO("FRONT CAM active");
        ros::Duration(5.0).sleep();
      }

      scan_request = true;

      if (scan_result && !detected) {
        scan_request = false;
        scan_result = false;

        toggle_srv.request.channel = 1; // switch to bottom camera
        toggle_succeed = togglecam_client.call(toggle_srv);
        if (!toggle_succeed) {
          ROS_ERROR_STREAM("Toggling cam FAILED !");
          finish();
        }
        ROS_INFO("BOTTOM CAM active");
        // ROS_INFO("------------ NOW ---------------");
        ros::Duration(2.0).sleep();

        ROS_INFO("MOVING");
        load_vel(0.0, 0.2, 0.0, 0.0);
        ros::Duration(10.0).sleep();
        // load_vel(0.0, 0.0, 0.0, -0.0016);
        // ros::Duration(1.0).sleep();
        hover();
      }

      else if (scan_result && detected) {
        scan_request = false;
        scan_result = false;

        toggle_srv.request.channel = 1; // switch to bottom camera
        toggle_succeed = togglecam_client.call(toggle_srv);
        if (!toggle_succeed) {
          ROS_ERROR_STREAM("Toggling cam FAILED !");
          finish();
        }
        ROS_INFO("BOTTOM CAM active");
        // ros::Duration(5.0).sleep();
        // ROS_INFO("------------ NOW ---------------");
        ros::Duration(2.0).sleep();

        load_vel(0.7, 0.0, 0.0, 0.0);
        ros::Duration(6.74).sleep();
        /*** FINISHING ***/
        finish();
      }

      else {
        hover();
      }
    }

    else if (path_type == CAM_DEMO) {
      //   scan_request = true;
      //   Reference_doors::Reference_doors myrefdoor;
      create_binmasks();
      ROS_INFO("Printing matrices 1!");

      //   ROS_INFO("Printing matrices 2!");
      //   MyOpenSpace = myrefdoor.AllSpaces[41];
      //   ROS_INFO("Printing matrices 3!");

      //   ROS_INFO("Printing matrices 4!");
      //   cv::imshow("OpenSpace DEMO", MyOpenSpace);
      //   ROS_INFO("Printing matrices 5!");
      cv::waitKey(0);
      //   ROS_INFO("Printing matrices 6!");
    }
  }

  else if (test_type == POSE_CONTROL) {

    if (path_type == STRAIGHTLINE) {
      // Going straightforward during 3 meters
      ROS_INFO_STREAM_ONCE("The drone starts the path-planning with pose "
                           "control following a strainght line trajectory !");
      load_pose(3.0, 0.0, 0.0);
      ros::Duration(35.0).sleep();
      ROS_INFO_STREAM_ONCE("Corner #1");
      // Stabilizing #1
      hover();
      ros::Duration(sleeptime).sleep();
    }

    else if (path_type == SQUARE) {
      // Going to Corner #1
      double corner_pos = 0.75;

      ROS_INFO_STREAM_ONCE("The drone starts the path-planning with pose "
                           "control following a square trajectory !");

      load_pose(0.0, corner_pos, 0.0);
      ros::Duration(sleeptime).sleep();
      ROS_INFO_STREAM_ONCE("Corner #1");
      // Stabilizing #1
      hover();
      ros::Duration(hovertime).sleep();

      // Going to Corner #2
      load_pose(-corner_pos, corner_pos, 0.0);
      ros::Duration(sleeptime).sleep();
      ROS_INFO_STREAM_ONCE("Corner #2");
      // Stabilizing #2
      hover();
      ros::Duration(hovertime).sleep();

      // Going to Corner #3
      load_pose(-corner_pos, 0.0, 0.0);
      ros::Duration(sleeptime).sleep();
      ROS_INFO_STREAM_ONCE("Corner #3");
      // Stabilizing #3
      hover();
      ros::Duration(hovertime).sleep();

      // Going to Corner #4
      load_pose(0.0, 0.0, 0.0);
      ros::Duration(sleeptime).sleep();
      ROS_INFO_STREAM_ONCE("Corner #4");
      // Stabilizing #4
      hover();
      ros::Duration(hovertime).sleep();
    }

    else if (path_type == STATIC) {
      ROS_INFO_STREAM_ONCE("The drone starts hovering with pose control !");
      hover();
      ros::Duration(sleeptime).sleep();
    }

    else if (path_type == ROOM_EXIT) {
      ROS_INFO_STREAM_ONCE("The drone scans the room and exits.");
      // Corner #1
      load_pose(2.0, 0.0, 0.0);
      ros::Duration(sleeptime).sleep();
      ROS_INFO_STREAM_ONCE("Corner #1");
      // Stabilizing #1
      hover();
      ros::Duration(hovertime).sleep();

      // Corner #2
      load_pose(2.0, 1.2, 0.0);
      ros::Duration(sleeptime).sleep();
      ROS_INFO_STREAM_ONCE("Corner #2");
      // Stabilizing #2
      hover();
      ros::Duration(hovertime).sleep();

      // Rotate 90° leftward
      load_vel(0.0, 0.0, 0.0, 0.325);
      ros::Duration(2.75).sleep();
      ROS_INFO_STREAM_ONCE("Rotation done !");
      hover();
      ros::Duration(0.5).sleep();

      // Corner #3
      load_pose(-1.0, 1.2, 0.0);
      ros::Duration(sleeptime).sleep();
      ROS_INFO_STREAM_ONCE("Corner #3");
      // Stabilizing #3
      hover();
      ros::Duration(hovertime).sleep();
    }
  }
}

// This function makes the drone taking off and stabilizing during 2 sec.
void Demo::init(void) {
  takeoff();
  ROS_INFO_STREAM_ONCE("The drone is taking off !");
  ros::Duration(7.0).sleep();

  ROS_INFO_STREAM_ONCE("The drone is in hover mode !");
  hover();
  ros::Duration(3.0).sleep();
}

// This function stabilizes the drone at the end of the test and do the
// drone
// landing.
void Demo::finish(void) {
  ROS_INFO_STREAM_ONCE("The drone just finished !");
  hover();
  ros::Duration(2.0).sleep();
  land();
  ROS_INFO_STREAM_ONCE("The drone just landed !");
  ros::Duration(3.0).sleep();
}

void Demo::create_binmasks() {

  using namespace cv;
  using namespace std;

  Mat RefBinaryDoor(360, 640, CV_8UC1, Scalar(0));
  Mat RefSpace(360, 640, CV_8UC1, Scalar(0));

  std::vector<cv::Mat> RefDoorsList;
  std::vector<cv::Mat> RefSpacesList;

  scale_factor = 1; // Out: 0.714; // 1 = door takes the full height
  height_zoom = scale_factor * img_height;
  door_thickness = door_thickness_ratio * height_zoom * thickness_error;
  init_xshift = 140 - (door_ratio * height_zoom / 2 + 3 * door_thickness);
  xshifts = 2 * init_xshift;
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

  // Rotation
  scale = 1.0;

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

      for (i_angle = -5; i_angle < 6; i_angle++) {
        angle = (double)i_angle;
        M = getRotationMatrix2D(Point(320, 180), angle, 1.0);

        // Door creation
        RefBinaryDoor.operator=(Scalar(0)); // Draw a new shifted reference
        rectangle(RefBinaryDoor, Point(xx1, yy1), Point(xx2, yy2), Scalar(255),
                  door_thickness, 8);
        line(RefBinaryDoor, Point(xx1, yy1), Point(xx2, yy1), Scalar(0),
             door_thickness, 8);
        warpAffine(RefBinaryDoor, RefBinaryDoor, M, RefBinaryDoor.size());
        RefDoorsList.push_back(RefBinaryDoor);

        // Open space creation
        RefSpace.operator=(Scalar(0));
        rectangle(RefSpace, Point(xx1rec1, yy1rec1), Point(xx2rec1, yy2rec1),
                  Scalar(255), CV_FILLED, 8);
        rectangle(RefSpace, Point(xx1rec2, yy1rec2), Point(xx2rec2, yy2rec2),
                  Scalar(255), CV_FILLED, 8);
        rectangle(RefSpace, Point(xx1rec3, yy1rec3), Point(xx2rec3, yy2rec3),
                  Scalar(255), CV_FILLED, 8);
        warpAffine(RefSpace, RefSpace, M, RefBinaryDoor.size());
        RefSpacesList.push_back(RefSpace);
      }
    }
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "Demo_node");
  ros::NodeHandle nh;
  Demo mytest;

  ros::Rate loop_rate(100);

  /*** Waiting time to connect to ARDrone camera ***/
  ros::Duration(20.0).sleep();
  // ROS_INFO_STREAM_ONCE("Demo_node started !");

  // /*** INITIALIZATION ***/
  // mytest.init();

  while (ros::ok()) {
    /*** LAUNCHING THE TEST ***/
    mytest.test();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
