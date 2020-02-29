#include <std_msgs/String.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "img_publisher");		// the node name does not have to be the same as the file name
	ros::NodeHandle nh;


	//ros::Publisher pub = nh.advertise<std_msgs::String>("/channel_1", 10);	// /channel_1 is the topic name, 10 is the queue size

	ros::Rate rate(50);		// publish 50 per second



  // Image Stuff
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image", 1);

  system("roslaunch image_processing usbcam.launch");  //start

  system("rosnode kill usbcamNode");              //stop


  Mat frame;
  VideoCapture cap;
  // open the default camera using default API
  // cap.open(0);
  // OR advance usage: select any API backend
  int deviceID = 0;             // 0 = open default camera
  int apiID = cv::CAP_ANY;      // 0 = autodetect default API
  // open selected camera using selected API
  cap.open(deviceID + apiID);
  // check if we succeeded
  if (!cap.isOpened()) {
  cerr << "ERROR! Unable to open camera\n";
  return -1;
  }





	while (ros::ok()){

    cap.read(frame);

		//std_msgs::String msg;
		//msg.data = "This is the channel 1 news, with your host, Charles Wolfe";

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();

		pub.publish(msg);
		rate.sleep();
	}
}
