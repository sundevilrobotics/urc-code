#include <ros/ros.h>
#include <robot_activity/managed_robot_activity.h>

#include <robot_activity_msgs/State.h>
#include <std_srvs/Empty.h>

#include "std_msgs/String.h"
#include <sstream>

namespace my_robot_activity_tutorials
{

    class MyRobotActivity : public robot_activity::ManagedRobotActivity
    {
    public:
        /* Important to inherit the constructor */
        using ManagedRobotActivity::ManagedRobotActivity;
        ~MyRobotActivity()
        {
            ROS_DEBUG_STREAM(getNamespace() << " destructed!");
        }
        // static ros::AsyncSpinner spinner;

    private:
        void onManagedCreate() override
        {
            subscriber_manager.subscribe("/heartbeat", 1,
                                         &MyRobotActivity::mySubscriberCallback, this);

            service_manager.advertiseService("test",
                                             &MyRobotActivity::myServiceCallback, this);

            // auto timer = registerIsolatedTimer(
            //     std::bind(&MyRobotActivity::myTimerCallback, this), 0.5, true);

            subscriber_manager.subscribe("/hello", 1000,
                                         &MyRobotActivity::SubscribeAndPublish, this);
        };
        ros::NodeHandle n;
        ros::Publisher chatter_pub;

        void SubscribeAndPublish(const std_msgs::String::ConstPtr msg)
        {

            chatter_pub = n.advertise<std_msgs::String>("/chatter", 1000);

            std_msgs::String msg1;
            std::stringstream ss;
            ss << "hello world";
            msg1.data = ss.str();
            ROS_INFO_STREAM(" published new info");

            while (std::string(msg->data.c_str()) == "START")
            {
                chatter_pub.publish(msg1);
                ros::spinOnce();
            }
        };
        void onManagedTerminate(){};

        void onManagedConfigure() override { ; };
        void onManagedUnconfigure() override { ; };

        void onManagedStart() override{
            // state = 1;
            // ROS_INFO_STREAM("Starting a publisher for ");

            // ros::NodeHandle n;
            // ros::Publisher chatter_pub = n.advertise<std_msgs::String>("/chatter", 1000);

            // while (ros::ok())
            // {
            //     std_msgs::String msg1;
            //     std::stringstream ss;
            //     ss << "hello world";
            //     msg1.data = ss.str();
            //     chatter_pub.publish(msg1);
            //     ROS_INFO_STREAM(" published new info");
            // }
        };
        void onManagedStop() override { ; };

        void onManagedResume() override { ; };
        void onManagedPause() override{

        };

        void myTimerCallback()
        {
            ROS_INFO_STREAM(getNamespace() << " Timer Counter: " << counter);
            counter++;
            unsigned int seed = time(NULL);
            float r2 = rand_r(&seed) / (RAND_MAX / 0.10);
            ros::Duration(2.05 - r2).sleep();
        };

        void mySubscriberCallback(boost::shared_ptr<robot_activity_msgs::State const> msg)
        {
            ROS_INFO_STREAM(getNamespace() << " "
                                           << msg->node_name << " is in BR" << unsigned(msg->state));
        };

        bool myServiceCallback(
            std_srvs::Empty::Request &request,
            std_srvs::Empty::Response &response)
        {
            ROS_INFO_STREAM(getNamespace() << " Service called, returning true");
            return true;
        };

        int counter = 0;
    };

} // namespace robot_activity_tutorials

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "br");
    using my_robot_activity_tutorials::MyRobotActivity;

    ROS_INFO_STREAM("Hello world");

    // ros::AsyncSpinner spinner(4);
    // spinner.start();

    MyRobotActivity node1(argc, argv, "first");
    node1.init().runAsync();

    MyRobotActivity node2(argc, argv, "second");
    node2.init().runAsync();

    ros::waitForShutdown();
    return 0;
}