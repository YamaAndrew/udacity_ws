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
    int total_bias = 0;
    // int left_border = img.step / img.width - 1;
    // int right_border = img.step - left_border;
    
    for (int i = 0; i < img.data.size(); i += 3) {
	int r = i;
	int g = i + 1;
	int b = i + 2;
	if (img.data[r] == white_pixel && img.data[g] == white_pixel && img.data[b] == white_pixel) {
	
	    int pixel_index = i / 3;
	    int col = pixel_index % img.width;
	
	    if (col < static_cast<double>(img.width) / 3) {
	        left_bias++;
	        //ROS_INFO("LEFT bias incremented");
	    } else if (col > static_cast<double>(img.width) / 3 * 2) {
	        right_bias++;
	        //ROS_INFO("RIGHT bias incremented");
	    } else {
		mid_bias++;
		//ROS_INFO("MID bias incremented");
            }
	}
    }
    
    total_bias = left_bias + mid_bias + right_bias;
    ROS_INFO_STREAM("total_bias: " << total_bias << ", left_bias: " << left_bias << ", mid_bias: " << mid_bias << ", right_bias: " << right_bias);
    if (total_bias == 0) {
    	drive_robot(0.0, 0.0);
    } else {
    	float z = (float)(left_bias - right_bias) / total_bias;
	// z *= 1.0;
    	float x = (float)mid_bias / total_bias;
	// x *= 1.0; 
	
	// Limit max speeds
        //if (z > 0.5) z = 0.5;
        //if (z < -0.5) z = -0.5;
        if (x > 0.2) x = 0.2;

        // Reduce forward speed if the ball is not centered
        if (mid_bias < total_bias * 0.3) {
	    x *= 0.3;
        }
    	drive_robot(x, z);
    	ROS_INFO_STREAM("x: " << x << ", z: " << z);
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
