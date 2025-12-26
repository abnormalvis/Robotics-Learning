#ifndef MECANUM_CHASSIS_CONTROLLER_HPP
#define MECANUM_CHASSIS_CONTROLLER_HPP
#include <control_msgs/JointControllerState.h>
#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <string.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
namespace mecanum_chassis_controller
{
    class MecanumChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
    public:
        MecanumChassisController() = default;
        ~MecanumChassisController() override = default;
        bool init(hardware_interface::EffortJointInterface *effort_joint_interface, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;
        void update(const ros::Time &time, const ros::Duration &period) override;
        hardware_interface::JointHandle front_left_joint_, front_right_joint_, back_left_joint_, back_right_joint_;
        void getGains(double &p, double &i, double &d, double &i_max, double &i_min);
        void setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min, const bool &antiwindup = false);
        double command_ = 0.0;
        double front_left_command_, front_right_command_, back_right_command_, back_left_command_;
        double wheel_track_, wheel_base_, wheel_radius_;
        double front_left_vel_, front_right_vel_, back_left_vel_, back_right_vel_;
        ros::Publisher odom_pub_;
        tf::TransformBroadcaster odom_broadcaster_;
        double odom_x_, odom_y_, odom_th_, odom_vx_, odom_vy_, odom_vth_;
        ros::Time current_time_, last_time_;
        std::string speed_mode_;
        nav_msgs::Odometry odom_;
        void odom_update(const ros::Time &time, const ros::Duration &period);
        double linear_scale_, angular_scale_;
        geometry_msgs::TransformStamped odom_trans_;
        void effortback(const sensor_msgs::JointState::ConstPtr &msg);
        double joint_torque_[100] = {0.0};
        double joint_Velocity_[100] = {0.0};
        double joint_effort_, k1_, k2_;
        double max_power_;

    private:
        int state_{};
        ros::Time last_change_;
        control_toolbox::Pid pid_front_left_controller_;
        control_toolbox::Pid pid_front_right_controller_;
        control_toolbox::Pid pid_back_left_controller_;
        control_toolbox::Pid pid_back_right_controller_;
        std::unique_ptr<
            realtime_tools::RealtimePublisher<
                control_msgs::JointControllerState>>
            controller_state_publisher_;
        ros::Subscriber sub_command_;
        ros::Subscriber effort_;
        int loop_count_ = 0;
        ros::Subscriber cmd_sub_;
        void cmdCallback(const geometry_msgs::Twist::ConstPtr &msg);
        void setCommandCB(const std_msgs::Float64ConstPtr &state);
    };
}
#endif