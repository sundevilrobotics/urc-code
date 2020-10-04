#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

/*
 * File: img_publisher.cpp
 * Author: Brandon Rice
 * Desc: Testing for detecting the Alvar AR tags probably used in this year's
 *       URC comp. See (http://wiki.ros.org/ar_track_alvar) for more info.
 */


using namespace std;
using namespace cv;

int main(int argc, char** argv)
{

  ros::init(argc, argv, "img_publisher");		// the node name does not have to be the same as the file name

  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image", 1);

  Mat frame, resized;
  VideoCapture cap;
  // open the default camera using default API
  // cap.open(0);
  // OR advance usage: select any API backend
  int deviceID = 1;             // 0 = open default camera
  int apiID = cv::CAP_ANY;      // 0 = autodetect default API
  // open selected camera using selected API


  for(int i = deviceID; i < 10; i++)
  {
    cap.open(i + apiID);
    if(cap.isOpened()) break;
  }
  // check if we succeeded
  if (!cap.isOpened()) {
  cerr << "ERROR! Unable to open camera\n";
  return -1;
  }
  //--- GRAB AND WRITE LOOP
  cout << "Start grabbing" << endl
  << "Press any key to terminate" << endl;
  // cap.set(CV_CAP_PROP_FRAME_WIDTH,1280);
  // cap.set(CV_CAP_PROP_FRAME_HEIGHT,720);

  // cv::Size S = cv::Size((int) cap.get(CV_CAP_PROP_FRAME_WIDTH), (int) cap.get(CV_CAP_PROP_FRAME_HEIGHT));
  // int ex = static_cast<int>(cap.get(CV_CAP_PROP_FOURCC));
  // int fps = cap.get(CV_CAP_PROP_FPS);
  // cout << "fps " << fps << " ex " << ex << endl;
  // VideoWriter video("/home/brandon/catkin_ws/src/urc-code/src/image_processing/img/out.avi", CV_FOURCC('M', 'J', 'P', 'G'), cap.get(CV_CAP_PROP_FPS), S, true);

  while (1)
  {


    // wait for a new frame from camera and store it into 'frame'
    cap.read(frame);
    cv::resize(frame, resized, cv::Size(640, 360));

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();

    ros::Rate loop_rate(5000);
    //while (nh.ok()) {
      pub.publish(msg);
      ros::spinOnce();
      loop_rate.sleep();
      cv::imshow("out", resized);
      char key = (char) cv::waitKey(1);
    //}


    // cvtColor(frame, frame, CV_RGB2GRAY);
    // Mat normalizedImg;
    // //GaussianBlur( frame, frame, Size( 9, 9 ), 0, 0 );
    // cv::normalize(frame,  normalizedImg, 0, 100, cv::NORM_MINMAX);
    // cv::Mat image, imageCopy;
    // normalizedImg.copyTo(imageCopy);
    // std::vector<int> ids;
    // std::vector<std::vector<cv::Point2f> > corners;
    // cv::aruco::detectMarkers(frame, dictionary, corners, ids);
    // // if at least one marker detected
    // if (ids.size() > 0)
    // cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
    // cv::imshow("out", imageCopy);
    // char key = (char) cv::waitKey(1);
    if (key == 27)
    {
      cap.~VideoCapture();
      break;
    }
  }
  // the camera will be deinitialized automatically in VideoCapture destructor
  return 0;
}
