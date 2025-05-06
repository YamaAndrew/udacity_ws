#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;
    
    if (!client.call(srv))
    	ROS_ERROR("Failed to call service drive");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;

    // Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    
    int left_bias = 0;
    int mid_bias = 0;
    int right_bias = 0;
    
    for (int i = 0; i < img.data.size(); i += 3) {
	int r = i;
	int g = i + 1;
	int b = i + 2;
	if (img.data[r] == white_pixel && img.data[g] == white_pixel && img.data[b] == white_pixel) {
	    int pixel_index = i / 3;
	    int col = pixel_index % img.width;
	
	    if (col < img.width / 3) {
	        left_bias++;
	    } else if (col > 2 * img.width / 3) {
	        right_bias++;
	    } else {
		mid_bias++;
            }
	}
    }
    
    int total_bias = left_bias + mid_bias + right_bias;
    if (total_bias == 0) {
    	drive_robot(0, 0);
    } else {
    	float z = (float)(left_bias - right_bias) / total_bias;
    	// do z *= 1.0; for sharper turns
    	float x = (float)mid_bias / total_bias;
    	// do x *= 1.0; for faster movement
    	drive_robot(x, z);
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
