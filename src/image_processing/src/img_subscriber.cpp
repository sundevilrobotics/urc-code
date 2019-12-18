#include <ros/ros.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

int message=0;

void printPose(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{

        // ROS_INFO("req.header.frame_id . . . . . .. .  .. ", msg->header.frame_id.c_str());

    //     if(msg != NULL)
    // ROS_INFO("New Frame Found!");
    // message++;


    for(unsigned int i=0;i<msg->markers.size();i++)
    {
      // if(msg->markers[i].id==msg->start_marker_id)
      // {
        ROS_INFO("marker_start: found marker id=%d, at x,y,z=%f,%f,%f", msg->markers[i].id, msg->markers[i].pose.pose.position.x,msg->markers[i].pose.pose.position.y, msg->markers[i].pose.pose.position.z );
        break;
      // }
    }



}



int main(int argc, char **argv)
{
    ROS_INFO("Start");

    ros::init(argc, argv, "pose_subscriber");

    ros::NodeHandle nh;

    ros::Subscriber pose_sub = nh.subscribe("ar_pose_marker", 0, printPose);

    //ros::spin();

    ros::Rate rate(50);
      //while (nh.ok()){
        ROS_INFO("Waiting for data...");


     ROS_INFO_STREAM(message);
ros::spin();


    return 0;

}

//
// void callback_receive_radio_data(const sensor_msgs::ImageConstPtr& msg){
// 	ROS_INFO("New Frame Found!");
//   try
//   {
//    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
//    cv::waitKey(30);
//   }
//   catch (cv_bridge::Exception& e)
//   {
//    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
//   }
// }
//
// int main (int argc, char **argv)
// {
// 	ros::init(argc, argv, "img_subscriber");		// the node name does not have to be the same as the file name
// 	ros::NodeHandle nh;
//
//   cv::namedWindow("view");
//   //cv::startWindowThread();
//   image_transport::ImageTransport it(nh);
//   image_transport::Subscriber sub = it.subscribe("camera/image", 1, callback_receive_radio_data);
//
// 	ros::spin();		// spin keeps the node running until you shut it down
// }
