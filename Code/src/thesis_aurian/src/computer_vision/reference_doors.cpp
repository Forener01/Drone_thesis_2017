#include <thesis_aurian/reference_doors.hpp>
// 
// Reference_doors() {
//   ros::NodeHandle nh;
// 
//   // thesis_aurian::refdoors refdoors_msg;
//   // Publishers
//   refdoor_pub = nh.advertise<thesis_aurian::refdoors>("reference_doors", 1);

// // Image parameters
// img_width = 640;
// img_height = 360;
// door_ratio = 0.425;           // --> W/H = 93.2/201 = real world
//                               // --> W/H = 104/253 = cam world
// door_thickness_ratio = 0.015; // --> T/H = 3/201
// keypressed = false;
//
// counter = 0;
// // }

void create() {

  using namespace cv;
  using namespace std;

  Mat RefBinaryDoor(360, 640, CV_8UC1, Scalar(0));
  Mat RefSpace(360, 640, CV_8UC1, Scalar(0));

  Mat AllRefDoorSpaces;

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

        // Door comparison
        RefBinaryDoor.operator=(Scalar(0)); // Draw a new shifted reference door
        rectangle(RefBinaryDoor, Point(xx1, yy1), Point(xx2, yy2), Scalar(255),
                  door_thickness, 8);
        line(RefBinaryDoor, Point(xx1, yy1), Point(xx2, yy1), Scalar(0),
             door_thickness, 8);
        warpAffine(RefBinaryDoor, RefBinaryDoor, M, RefBinaryDoor.size());
        AllDoors.push_back(RefBinaryDoor);
        // Open space comparison
        RefSpace.operator=(Scalar(0));
        rectangle(RefSpace, Point(xx1rec1, yy1rec1), Point(xx2rec1, yy2rec1),
                  Scalar(255), CV_FILLED, 8);
        rectangle(RefSpace, Point(xx1rec2, yy1rec2), Point(xx2rec2, yy2rec2),
                  Scalar(255), CV_FILLED, 8);
        rectangle(RefSpace, Point(xx1rec3, yy1rec3), Point(xx2rec3, yy2rec3),
                  Scalar(255), CV_FILLED, 8);
        warpAffine(RefSpace, RefSpace, M, RefBinaryDoor.size());
        AllSpaces.push_back(RefSpace);
      }
    }
  }

  ROS_INFO("Reference matrices created !");
  cv::imshow("Refdoor", AllDoors[12]);
  cv::imshow("OpenSpace", AllSpaces[41]);
  // ROS_INFO("Reference matrices screened !");
  // AllRefDoorSpaces.push_back(AllDoors);
  // AllRefDoorSpaces.push_back(AllSpaces);

  // refdoors_msg.data = AllRefDoorSpaces;
  // refdoor_pub.publish(refdoors_msg);
  waitKey(0);
}

// int main(int argc, char **argv) {
//   ros::init(argc, argv, "refdoors_node");
//   ros::NodeHandle nh;
//   Reference_doors mydoors;
//   ros::Rate loop_rate(100);
//
//   mydoors.create();
//
//   while (ros::ok()) {
//
//     ros::spinOnce();
//     loop_rate.sleep();
//   }
//
//   return 0;
// }
