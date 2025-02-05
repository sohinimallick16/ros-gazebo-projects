#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // Create a request and response object for the service
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    // Call the service and pass the requested velocities
    if (!client.call(srv)) {
        ROS_ERROR("Failed to call service /ball_chaser/command_robot");
    }
    
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    int white_pixel = 255;
    bool ball_found = false;
    int ball_position = -1;  // We'll store the column index of the ball here

    // The image data is in a single array. Each pixel is represented by 3 consecutive bytes: R, G, B.
    // img.step = number of bytes in a single row
    // img.width = number of pixels in width
    // img.height = number of pixels in height

    for (int row = 0; row < img.height; row++) {
        for (int col = 0; col < img.width; col++) {
            int pixel_index = (row * img.step) + (col * 3);

            // Check if the pixel is white
            if (img.data[pixel_index] == white_pixel &&
                img.data[pixel_index + 1] == white_pixel &&
                img.data[pixel_index + 2] == white_pixel) {
                ball_found = true;
                ball_position = col;  
                break; 
            }
        }
        if (ball_found) {
            break;
        }
    }

    // If no white ball found, stop the robot
    if (!ball_found) {
        drive_robot(0.0, 0.0);
        return;
    }

    // Determine in which third of the image the ball falls
    int left_boundary = img.width / 3;
    int right_boundary = 2 * img.width / 3;

    if (ball_position < left_boundary) {
        // Ball is on the left side
        drive_robot(0.0, 0.5);  // Turn left
    } else if (ball_position < right_boundary) {
        // Ball is in the center
        drive_robot(0.5, 0.0);  // Move forward
    } else {
        // Ball is on the right side
        drive_robot(0.0, -0.5); // Turn right
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

    // Handle ROS communication events
    ros::spin();

    return 0;
}