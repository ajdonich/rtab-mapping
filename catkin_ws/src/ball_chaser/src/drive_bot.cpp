#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/CommandVel.h"

// ROS::Publisher motor commands;
ros::Publisher motor_command_publisher;
geometry_msgs::Twist prev_command;

// Publish a motor_command to robot wheel joints w/requested linear_x and 
// angular_z velocities, and similarly fill out message feedback response 
bool handle_drive_request(ball_chaser::CommandVel::Request& req,
                          ball_chaser::CommandVel::Response& res) 
{
    geometry_msgs::Twist motor_command;
    motor_command.linear.x = (float)req.linear_x;
    motor_command.angular.z = (float)req.angular_z;
    motor_command_publisher.publish(motor_command);

    res.msg_feedback = ("Published CommandVel " + 
                        std::to_string((float)req.linear_x) + ", " +
                        std::to_string((float)req.angular_z));
    
    if (motor_command.linear.x != prev_command.linear.x ||
        motor_command.angular.z != prev_command.angular.z) {
        //ROS_INFO_STREAM(res.msg_feedback);
        prev_command = motor_command;
    }

    return true;
}

int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");
    ros::NodeHandle n;

    prev_command.linear.x = 0.0;
    prev_command.angular.z = 0.0;

    // Inform ROS master we'll be publishing geometry_msgs::Twist messages  
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Define a drive service with a handle_drive_request callback function
    ros::ServiceServer rcmd_service = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);
    ROS_INFO("Ready to send robot velocity commands");

    ros::spin();
    return 0;
}