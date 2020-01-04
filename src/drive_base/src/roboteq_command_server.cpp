#include "ros/ros.h"
#include "drive_base/RoboteqCommand.h"
#include "RoboteqDevice.h"
#include "ErrorCodes.h"

RoboteqDevice device;

bool sendCommand(drive_base::RoboteqCommand::Request &request, drive_base::RoboteqCommand::Response &response)
{
    int result = 0;
    switch(request.commandtype)
    {
        case '~':
            response.response = device.GetConfig(request.item, request.index, result);
        case '!':
            response.response = device.SetCommand(request.item, request.index, request.value);
        case '?':
            response.response = device.GetValue(request.item, request.index, result);
        case '^':
            response.response = device.SetConfig(request.item, request.index, request.value);
        
    }
    response.result = result;
    return true;
}

int main(int argc, char** argv)
{

    // Initialize ROS node
    ros::init(argc, argv, "roboteq_command_server");
    ros::NodeHandle handle;
    ros::ServiceServer server = handle.advertiseService("roboteq_command", sendCommand);

    // Connect to Roboteq Device
    int status = device.Connect("/dev/ttyACM0");
    if(status != RQ_SUCCESS)
    {
        ROS_ERROR("Error Connecting to Roboteq Device: Code %d",status);
        return 1;
    }
    
    // Handle functionality
    ros::spin();
}