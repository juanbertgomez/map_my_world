#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;
ros::Publisher motor_command_publisher;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    ROS_INFO("DriveToTarget received from process_image_callback- j1:%1.2f, j2%1.2f", (float)lin_x, (float)ang_z);
    // TODO: Request a service and pass the velocities to it to drive the robot
    geometry_msgs::Twist motor_command;

    motor_command.linear_x = lin_x;
    motor_command.angular_z = ang_z;

    //Publish drive commands
    motor_command_publisher.publish(motor_command);
    
    // Return a response message
    // res.msg_feedback = "DriveToTarget processed from process_image_callback");
    // ROS_INFO_STREAM(res.msg_feedback);

    // if (!client.call(srv))
    //     ROS_ERROR("Failed to call service command_robot");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    ROS_INFO("process_image_callback");
    
    int white_pixel = 255;
    enum Direction { left, forward, rigth, stop };
    Direction drive_direction = stop;
    int image_length = img.step;

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    for (int i=0; i < image_length; i++) {
        bool white_detetected  = img.data[i] == white_pixel;
        
        if (white_detetected) {
            if(image_length / i < 7/2)
            {
                drive_direction = left;
                break;
            } else if (image_length/ i < 7/5)
            {
                drive_direction = forward;    
                break;
            } else {
                drive_direction = rigth;
                break;
            }
        }
        
    switch(drive_direction)
        {
            case stop  : drive_robot(0, 0);   break;
            case left  : drive_robot(0.5, -0.5);   break;
            case forward : drive_robot(0.5, 0); break;
            case rigth : drive_robot(0.5, 0.5);  break;
        }
    }
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);
    ROS_INFO("Ready to see things!");

    //Advertise motor commands
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    
    // Handle ROS communication events
    ros::spin();

    return 0;
}