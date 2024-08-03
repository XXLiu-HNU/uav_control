#include <ros/ros.h>
#include <std_msgs/String.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

class KeyboardInput
{
public:
    KeyboardInput()
    {
        pub_ = nh_.advertise<std_msgs::String>("keyboard_input", 10);
        tcgetattr(STDIN_FILENO, &initial_settings_);
        termios new_settings = initial_settings_;
        new_settings.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);
        fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
    }

    ~KeyboardInput()
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &initial_settings_);
    }

    void run()
    {
        ros::Rate rate(10);
        while (ros::ok())
        {
            char c;
            if (read(STDIN_FILENO, &c, 1) == 1)
            {
                std_msgs::String msg;
                msg.data = std::string(1, c);
                pub_.publish(msg);
            }
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    termios initial_settings_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "keyboard_input");
    KeyboardInput keyboard_input;
    keyboard_input.run();
    return 0;
}
