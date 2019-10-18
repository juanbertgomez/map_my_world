#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

ros::Publisher motor_command_publisher;

bool handle_drive_request(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Request& res) 
{
    ROS_INFO("DriveToTarget received - j1:%1.2f, j2%1.2f", (float)req.linear_x, (float)req.angular_z);

    geometry_msgs::Twist motor_command;

    //Set velocity open message arrival
    motor_command.linear.x = req.linear_x;
    motor_command.angular.z = req.angular_z;

    //Publish drive commands
    motor_command_publisher.publish(motor_command);

    // Return a response message
    // res.msg_feedback = "DriveToTarget processed - j1:%1.2f, j2%1.2f", (float)req.linear_x, (float)req.angular_z;
    // ROS_INFO_STREAM(res.msg_feedback);

    return true;
}

int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // TODO: Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function
    ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);
    ROS_INFO("Ready to move the bot!");

    //TODO: Handle ROS communication events
    ros::spin();

    return 0;
}