#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

class ProcessImage final
{
private:
    int ball_pos_past;
    // Define a global client that can request services
    ros::ServiceClient client;
    ros::Subscriber sub_sub1;


public:
    ProcessImage()
    : ball_pos_past{0}
    {
        ros::NodeHandle n;

        // Define a client service capable of requesting services from command_robot
        client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

        // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
        sub_sub1 = n.subscribe("/camera/rgb/image_raw", 10, &ProcessImage::process_image_callback, this);
    }

    // This function calls the command_robot service to drive the robot in the specified direction
    void drive_robot(float lin_x, float ang_z)
    {
        // TODO: Request a service and pass the velocities to it to drive the robot
        ROS_INFO_STREAM("Drive the robot to the ball or stop.");

        // Request velocities
        ball_chaser::DriveToTarget srv;
        srv.request.linear_x  = lin_x;
        srv.request.angular_z = ang_z;

        // Call the drive_bot service and pass the requested velocities
        if( client.call(srv) == false )
        {
            ROS_ERROR("Failed to call service drive_bot");
        }
    }

	// This callback function continuously executes and reads the image data
    void process_image_callback(const sensor_msgs::Image img)
    {
		// TODO: Loop through each pixel in the image and check if there's a bright white one
        // Then, identify if this pixel falls in the left, mid, or right side of the image
        // Depending on the white ball position, call the drive_bot function and pass velocities to it
        // Request a stop when there's no white ball seen by the camera
        
		const unsigned int white_pixel_rgb   {255*3};
        const unsigned int left_area_border  {img.width/3};
        const unsigned int right_area_border {(img.width/3)*2};

        unsigned int left_num   {0};
		unsigned int middle_num {0};
        unsigned int right_num  {0};

        unsigned int ball_pos {0};
        
		
		// 1 - Count the white pixels where they are [left_count, middle_count, right_count]
        for( unsigned int row = 0; row < img.height; row++ ) 
		{
            for( unsigned int col = 0; col < img.width; col++ )
			{
				unsigned int index = row * img.step + col * 3;

				unsigned int r_pixel = img.data[index];
				unsigned int g_pixel = img.data[index + 1];
				unsigned int b_pixel = img.data[index + 2];
    
				if( (r_pixel + g_pixel + b_pixel) == white_pixel_rgb )
				{
					if( col <= left_area_border )
					{
						left_num++;
					}
					else if( col >= right_area_border )
					{
						right_num++;
					}
					else
					{
						middle_num++;
					}
				}
			}
        }
        
		
		// 2 - Ball pose [none, left, middle, right]
        if( left_num > middle_num && left_num > right_num )
        {
            ball_pos = 1;
			// White ball is left
        }
        else if( middle_num > left_num && middle_num > right_num )
        {
            ball_pos = 2;
			// White ball is in the middle
        }
        else if( right_num > left_num && right_num > middle_num )
        {
            ball_pos = 3;
			// White ball is right
        }
        else
        {
            ball_pos = 0;
			// White ball is not in the image
        }


        // 3 - Drive robot command [none, left, forward, right]
        switch(ball_pos)
        {
            case 0: // No ball > no drive
                if( ball_pos_past != ball_pos )
                {
                    drive_robot(0.0, 0.0);
                    ball_pos_past = ball_pos;
                    ROS_INFO("pos = %d with numbers = [ l: %d, m: %d, r: %d ]", ball_pos, left_num, middle_num, right_num);
                }
                break;

            case 1: // drive left
                if( ball_pos_past != ball_pos )
                {
                    drive_robot(0.5, 1.0);
                    ball_pos_past = ball_pos;
                    ROS_INFO("pos = %d with numbers = [ l: %d, m: %d, r: %d ]", ball_pos, left_num, middle_num, right_num);
                }
                break;

            case 2: // drive foreward
                if( ball_pos_past != ball_pos )
                {
                    drive_robot(0.5, 0.0);
                    ball_pos_past = ball_pos;
                    ROS_INFO("pos = %d with numbers = [ l: %d, m: %d, r: %d ]", ball_pos, left_num, middle_num, right_num);
                }
                break;

            case 3: // drive right
                if( ball_pos_past != ball_pos )
                {
                    drive_robot(0.5, -1.0);
                    ball_pos_past = ball_pos;
                    ROS_INFO("pos = %d with numbers = [ l: %d, m: %d, r: %d ]", ball_pos, left_num, middle_num, right_num);
                }
                break;

            default:;
        }
    }

};


int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");

    // Process image service
    ProcessImage process_image;

    // Handle ROS communication events
    ros::spin();

    return 0;
}