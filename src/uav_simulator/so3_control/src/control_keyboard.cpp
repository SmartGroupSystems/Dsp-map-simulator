#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <unistd.h>
#include <termios.h>

int getch() {
    static struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "keyboard_control_node");
    ros::NodeHandle nh;
    ros::Publisher cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 10);

    quadrotor_msgs::PositionCommand cmd_msg;
    cmd_msg.header.stamp = ros::Time::now();
    cmd_msg.position.x = 0;
    cmd_msg.position.y = 0;
    cmd_msg.position.z = 1;
    cmd_msg.yaw = 0;

    cmd_pub.publish(cmd_msg);
    ROS_INFO("\033[1;32m READY TO PUB CONTROL NOW! USE WASD IJKL TO CONTROL THE QUADROTOR.\033[0m");

    char input;
    double step = 0.1;

    while (ros::ok()) {
        input = getch();
        switch(input) {
            case 'w':
                cmd_msg.position.z += step;
                break;
            case 's':
                cmd_msg.position.z -= step;
                break;
            case 'a':
                cmd_msg.yaw += step;
                break;
            case 'd':
                cmd_msg.yaw -= step;
                break;
            case 'i':
                cmd_msg.position.x += step;
                break;
            case 'k':
                cmd_msg.position.x -= step;
                break;
            case 'j':
                cmd_msg.position.y += step;
                break;
            case 'l':
                cmd_msg.position.y -= step;
                break;
            case 3:  // ASCII code for CTRL+C
                ROS_INFO("Ctrl-C pressed. Exiting...");
                return 0;
            default:
                ROS_WARN("INVALID INPUT!");
                break;
        }
        cmd_pub.publish(cmd_msg);
        ros::spinOnce();
    }

    return 0;
}
