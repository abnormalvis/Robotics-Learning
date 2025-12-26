
#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>

#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_B 0x62
#define KEYCODE_Z 0x7A
#define KEYCODE_C 0x63
class SmartCarKeyboardTeleopNode
{
private:
    double walk_vel_;
    double yaw_rate_;

    geometry_msgs::Twist cmdvel_;
    ros::NodeHandle n_;
    ros::Publisher pub_;

public:
    SmartCarKeyboardTeleopNode()
    {
        pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

        ros::NodeHandle n_private("~");
        n_private.param("walk_vel", walk_vel_, 3.0);
        n_private.param("yaw_rate", yaw_rate_, 3.0);
    }

    ~SmartCarKeyboardTeleopNode() {}
    void keyboardLoop();

    void stopRobot()
    {
        cmdvel_.linear.x = 0.0;
        cmdvel_.linear.y = 0.0;
        cmdvel_.angular.z = 0.0;
        pub_.publish(cmdvel_);
    }
};

SmartCarKeyboardTeleopNode *TBK;
int KFD = 0;
struct termios COOKED, RAW;
bool DONE;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tbk", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
    SmartCarKeyboardTeleopNode tbk;
    boost::thread t = boost::thread(boost::bind(&SmartCarKeyboardTeleopNode::keyboardLoop, &tbk));
    ros::spin();
    t.interrupt();
    t.join();
    tbk.stopRobot();
    tcsetattr(KFD, TCSANOW, &COOKED);

    return (0);
}

void SmartCarKeyboardTeleopNode::keyboardLoop()
{
    char c;
    double max_tv = walk_vel_;
    double max_rv = yaw_rate_;
    bool dirty = false;
    int speed_x = 0;
    int speed_y = 0;
    int turn = 0;
    tcgetattr(KFD, &COOKED);
    memcpy(&RAW, &COOKED, sizeof(struct termios));
    RAW.c_lflag &= ~(ICANON | ECHO);
    RAW.c_cc[VEOL] = 1;
    RAW.c_cc[VEOF] = 2;
    tcsetattr(KFD, TCSANOW, &RAW);

    puts("Reading from keyboard");
    puts("Use WASD keys to control the robot");
    puts("Press Shift to move faster");
    struct pollfd ufd;
    ufd.fd = KFD;
    ufd.events = POLLIN;

    for (;;)
    {
        boost::this_thread::interruption_point();
        int num;
        if ((num = poll(&ufd, 1, 250)) < 0)
        {
            perror("poll():");
            return;
        }
        else if (num > 0)
        {
            if (read(KFD, &c, 1) < 0)
            {
                perror("read():");
                return;
            }
        }
        else
        {
            if (dirty)
            {
                stopRobot();
                dirty = false;
            }

            continue;
        }

        switch (c)
        {
        case KEYCODE_W:
            max_tv = walk_vel_;
            speed_x = 1;
            speed_y = 0;
            turn = 0;
            dirty = true;
            break;
        case KEYCODE_S:
            max_tv = walk_vel_;
            speed_x = -1;
            speed_y = 0;
            turn = 0;
            dirty = true;
            break;
        case KEYCODE_A:
            max_tv = walk_vel_;
            speed_x = 0;
            speed_y = -1;
            turn = 0;
            dirty = true;
            break;
        case KEYCODE_D:
            max_tv = walk_vel_;
            speed_x = 0;
            speed_y = 1;
            turn = 0;
            dirty = true;
            break;
        case KEYCODE_Z:
            max_rv = yaw_rate_;
            speed_x = 0;
            speed_y = 0;
            turn = 1;
            dirty = true;
            break;
        case KEYCODE_C:
            max_rv = yaw_rate_;
            speed_x = 0;
            speed_y = 0;
            turn = -1;
            dirty = true;
            break;
        case KEYCODE_B:
            walk_vel_ += 0.1;
            yaw_rate_ += 0.1;
            break;
        default:
            max_tv = walk_vel_;
            max_rv = yaw_rate_;
            speed_x = 0;
            speed_y = 0;
            turn = 0;
            dirty = false;
        }
        cmdvel_.linear.x = speed_x * max_tv;
        cmdvel_.linear.y = speed_y * max_tv;
        cmdvel_.angular.z = turn * max_rv;
        pub_.publish(cmdvel_);
    }
}