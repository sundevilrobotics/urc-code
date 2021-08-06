/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, University of Luxembourg
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of University of Luxembourg nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Maciej Zurad
 *********************************************************************/
#include <ros/ros.h>
#include <robot_activity/managed_robot_activity.h>

#include <robot_activity_msgs/State.h>
#include <std_srvs/Empty.h>
#include <robot_activity/isolated_async_timer.h>

namespace robot_activity
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

    private:
        void onManagedCreate() override
        {
            subscriber_manager.subscribe("/heartbeat", 1,
                                         &MyRobotActivity::mySubscriberCallback, this);

            // service_manager.advertiseService("test",
            //                                  &MyRobotActivity::myServiceCallback, this);

            RobotActivity::registerIsolatedTimer(
                std::bind(&MyRobotActivity::myTimerCallback, this), 1.0, true);
        };
        void onManagedTerminate() override{};

        void onManagedConfigure() override { ; };
        void onManagedUnconfigure() override { ; };

        void onManagedStart() override { ; };
        void onManagedStop() override { ; };

        void onManagedResume() override { ; };
        void onManagedPause() override { ; };

        void myTimerCallback()
        {
            ROS_INFO_STREAM(getNamespace() << " Timer Counter: " << counter);
            counter++;
            // unsigned int seed = time(NULL);
            // float r2 = rand_r(&seed) / (RAND_MAX / 0.10);
            // ros::Duration(2.1 - r2).sleep();
        };

        void mySubscriberCallback(boost::shared_ptr<robot_activity_msgs::State const> msg){
            // ROS_INFO_STREAM(getNamespace() << " "
            //                 << msg->node_name << " is in " << unsigned(msg->state));
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
    using robot_activity::MyRobotActivity;

    MyRobotActivity node1(argc, argv, "process");
    node1.init().runAsync();

    //   MyRobotActivity node2(argc, argv, "second");
    //   node2.init().runAsync();

    ros::waitForShutdown();
    return 0;
}