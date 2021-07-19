#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
//TODO: Include the ball_chaser "DriveToTarget" header file
#include "ball_chaser/DriveToTarget.h"

class DriveBot final
{
private:
    ros::Publisher pub_motor_command;
    ros::ServiceServer service;

public:
    DriveBot()
    {
        // Create a ROS NodeHandle object
        ros::NodeHandle n;

        // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
        pub_motor_command = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        // TODO: Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function
        service = n.advertiseService("/ball_chaser/command_robot", &DriveBot::handle_drive_request, this);
        
        ROS_INFO("Ready to send drive commands");
    }

    // TODO: Create a handle_drive_request callback function that executes whenever a drive_bot service is requested
    // This function should publish the requested linear x and angular velocities to the robot wheel joints
    // After publishing the requested velocities, a message feedback should be returned with the requested wheel velocities
    bool handle_drive_request(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res)
    {
        ROS_INFO("DriveToTarget received - linear_x: %1.2f, angular_z: %1.2f", (float)req.linear_x, (float)req.angular_z);

        // Create a motor_command object of type geometry_msgs::Twist
        geometry_msgs::Twist motor_command;
        
        // Set wheel velocities, forward [req.linear_x, req.angular_z]
        motor_command.linear.x = req.linear_x;
        motor_command.angular.z = req.angular_z;
            
        // Publish angles to drive the robot
        pub_motor_command.publish(motor_command);

        // Return a response message
        res.msg_feedback = "drive bot set - linear_x: " + std::to_string(req.linear_x) +  " , angular_z: " + std::to_string(req.angular_z);
        
        ROS_INFO_STREAM(res.msg_feedback);

        return true;
    }

};


int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");
    
    // Drive_bot service
    DriveBot drive_bot;

    // TODO: Handle ROS communication events
    ros::spin();

    return 0;
}