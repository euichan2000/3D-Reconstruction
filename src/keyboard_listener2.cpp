#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "keyboard_listener2");
    ros::NodeHandle nh;
    ros::Publisher command_pub = nh.advertise<std_msgs::String>("keyboard_command", 10);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    while (ros::ok())
    {
        std_msgs::String msg;
        // std::cout << "Enter 'a' to start the robot: ";
        std::string input;
        std::cin >> input;

        if (input == "a")
        {
            msg.data = "capture";
            command_pub.publish(msg);
        }

        if (input == "b")
        {
            msg.data = "reconstruction";
            command_pub.publish(msg);
        }

        ros::Duration(1.0).sleep(); // Sleep to prevent CPU overload
    }

    return 0;
}
