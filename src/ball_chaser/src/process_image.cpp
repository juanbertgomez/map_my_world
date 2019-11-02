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
    ROS_INFO("DriveToTarget received from process_image_callback- lin_x:%1.2f, ang_z: %1.2f", (float)lin_x, (float)ang_z);
    // TODO: Request a service and pass the velocities to it to drive the robot
    geometry_msgs::Twist motor_command;

    motor_command.linear.x = lin_x;
    motor_command.angular.z = ang_z;

    //Publish drive commands
    motor_command_publisher.publish(motor_command);
    
}

void direction_controller(float white_color_side)
{
    if(white_color_side < 2/(double)7)
    {
        ROS_INFO("Movel left");        
        drive_robot(0.5, 0.5);
    } else if (white_color_side < 5/(double)7)
    {
        ROS_INFO("Movel forward");
        drive_robot(0.5, 0);
    } else {
        ROS_INFO("Movel right");
        drive_robot(0.5, -0.5);
    }
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;
    int image_step = img.step;
    int image_width = img.width;
    int image_heigth = img.height;
    int rgb_channels = 3;
    bool ball_found = false;

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    for (int i = 0; i < image_step && !ball_found; i+=rgb_channels) {
        for(int row = 0; row < image_heigth && !ball_found; row++) {
            int index = row * image_step + i;

            bool red_channel_white = img.data[index] == white_pixel;
            bool green_channel_white = img.data[index+1] == white_pixel;
            bool blue_channel_white = img.data[index+2] == white_pixel;

            if(red_channel_white && green_channel_white && blue_channel_white) {
                ball_found = true;
                float white_color_side = i / (double)image_step;
                ROS_INFO("I value - %d, white side - %1.2f", (int)i, (float)white_color_side);
                direction_controller(white_color_side);
            } 
        }
    }

    if(!ball_found) 
    {
        ROS_INFO("STOP!!");
        drive_robot(0, 0);
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