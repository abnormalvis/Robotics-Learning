#include <mecanum_chassis_controller.hpp>
#include <memory>
#include <pluginlib/class_list_macros.hpp>

namespace mecanum_chassis_controller
{
    bool MecanumChassisController::init(hardware_interface::EffortJointInterface *effort_joint_interface,
                                        ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
    {
        odom_pub_ = controller_nh.advertise<nav_msgs::Odometry>("/odom", 100);
        effort_ = controller_nh.subscribe("/joint_states", 1000, &MecanumChassisController::effortback, this);
        odom_.header.stamp = ros::Time(ros::Time::now());
        odom_.header.frame_id = "odom";
        odom_pub_.publish(odom_);
        controller_nh.getParam("wheel_track", wheel_track_);
        controller_nh.getParam("wheel_base", wheel_base_);
        controller_nh.getParam("wheel_radius", wheel_radius_);
        controller_nh.getParam("speed_mode", speed_mode_);
        controller_nh.getParam("max_power", max_power_);
        controller_nh.getParam("effort", joint_effort_);

        front_left_joint_ = effort_joint_interface->getHandle("left_front_wheel_joint");
        front_right_joint_ = effort_joint_interface->getHandle("right_front_wheel_joint");
        back_left_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
        back_right_joint_ = effort_joint_interface->getHandle("right_back_wheel_joint");
        pid_back_left_controller_.init(ros::NodeHandle(controller_nh, "pid_back_left"));
        pid_back_right_controller_.init(ros::NodeHandle(controller_nh, "pid_back_right"));
        pid_front_left_controller_.init(ros::NodeHandle(controller_nh, "pid_front_left"));
        pid_front_right_controller_.init(ros::NodeHandle(controller_nh, "pid_front_right"));
        controller_state_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<control_msgs::JointControllerState>>(
            controller_nh, "state", 1);
        cmd_sub_ = root_nh.subscribe("/cmd_vel", 1000, &MecanumChassisController::cmdCallback, this);
        sub_command_ = controller_nh.subscribe<std_msgs::Float64>("command", 1, &MecanumChassisController::setCommandCB, this);

        odom_x_ = -0.003691;
        odom_y_ = -0.001885;
        odom_th_ = 0.0;
        return true;
    }
    void MecanumChassisController::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
    {
        pid_front_right_controller_.getGains(p, i, d, i_max, i_min);
        pid_front_left_controller_.getGains(p, i, d, i_max, i_min);
        pid_back_right_controller_.getGains(p, i, d, i_max, i_min);
        pid_back_left_controller_.getGains(p, i, d, i_max, i_min);
    }
    void MecanumChassisController::setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min, const bool &antiwindup)
    {
        pid_front_right_controller_.setGains(p, i, d, i_max, i_min, antiwindup);
        pid_front_left_controller_.setGains(p, i, d, i_max, i_min, antiwindup);
        pid_back_right_controller_.setGains(p, i, d, i_max, i_min, antiwindup);
        pid_back_left_controller_.setGains(p, i, d, i_max, i_min, antiwindup);
    }
    void MecanumChassisController::update(const ros::Time &time, const ros::Duration &period)
    {
        odom_update(time, period);
        front_left_command_ = front_left_vel_;
        front_right_command_ = front_right_vel_;
        back_left_command_ = back_left_vel_;
        back_right_command_ = back_right_vel_;
        double front_left_error = front_left_command_ - front_left_joint_.getVelocity();
        double front_right_error = front_right_command_ - front_right_joint_.getVelocity();
        double back_right_error = back_right_command_ - back_right_joint_.getVelocity();
        double back_left_error = back_left_command_ - back_left_joint_.getVelocity();
        double front_left_commanded_effort = pid_front_left_controller_.computeCommand(front_left_error, period);
        double front_right_commanded_effort = pid_front_right_controller_.computeCommand(front_right_error, period);
        double back_left_commanded_effort = pid_back_left_controller_.computeCommand(back_left_error, period);
        double back_right_commanded_effort = pid_back_right_controller_.computeCommand(back_right_error, period);
        double now_power = (fabs(front_right_commanded_effort) + fabs(front_left_commanded_effort) +
                            fabs(back_right_commanded_effort) + fabs(back_left_commanded_effort)) *
                               joint_torque_[0] * 4 +
                           k1_ * (joint_torque_[0] * 4) * (joint_torque_[0] * 4) +
                           k2_ * (fabs(front_right_commanded_effort) + fabs(front_left_commanded_effort) + fabs(back_right_commanded_effort) + fabs(back_left_commanded_effort)) *
                               (fabs(front_right_commanded_effort) + fabs(front_left_commanded_effort) +
                                fabs(back_right_commanded_effort) + fabs(back_left_commanded_effort));
        if (now_power > max_power_)
        {
            front_left_joint_.setCommand(max_power_ / (joint_torque_[1] * 4.0));
            front_right_joint_.setCommand(max_power_ / (joint_torque_[3] * 4.0));
            back_left_joint_.setCommand(max_power_ / (joint_torque_[0] * 4.0));
            back_right_joint_.setCommand(max_power_ / (joint_torque_[2] * 4.0));
        }
        else
        {
            front_left_joint_.setCommand(front_left_commanded_effort);
            front_right_joint_.setCommand(front_right_commanded_effort);
            back_left_joint_.setCommand(back_left_commanded_effort);
            back_right_joint_.setCommand(back_right_commanded_effort);
        }
    }
    void MecanumChassisController::setCommandCB(const std_msgs::Float64ConstPtr &state)
    {
        state_ = state->data;
    }
    void MecanumChassisController::cmdCallback(const geometry_msgs::Twist::ConstPtr &msg)
    {
        geometry_msgs::Twist vel;
        vel.linear.x = msg->linear.x;
        vel.linear.y = msg->linear.y;
        vel.angular.z = msg->angular.z;
        if (speed_mode_ == "global")
        {
            try
            {
                tf::TransformListener tf_listener;
                geometry_msgs::Vector3Stamped vel_local, vel_global;
                vel_global.header.frame_id = "odom";
                vel_global.vector.x = vel.linear.x;
                vel_global.vector.y = vel.linear.y;
                vel_global.vector.z = 0.0;
                //          ros::Time now = ros::Time::now();
                tf_listener.waitForTransform("base_link", "odom", ros::Time(), ros::Duration(2.0));
                tf_listener.transformVector("base_link", vel_global, vel_local);
                vel.linear.x = vel_local.vector.x;
                vel.linear.y = vel_local.vector.y;
            }
            catch (tf::TransformException &ex)
            {
                ROS_WARN("TF transform error: %s", ex.what());
                return;
            }
        }
        front_left_vel_ = vel.linear.x - vel.linear.y - vel.angular.z * ((wheel_track_ + wheel_base_) / 2.0);
        front_right_vel_ = vel.linear.x + vel.linear.y + vel.angular.z * ((wheel_track_ + wheel_base_) / 2.0);
        back_right_vel_ = vel.linear.x - vel.linear.y + vel.angular.z * ((wheel_track_ + wheel_base_) / 2.0);
        back_left_vel_ = vel.linear.x + vel.linear.y - vel.angular.z * ((wheel_track_ + wheel_base_) / 2.0);
        front_left_vel_ /= wheel_radius_;
        front_right_vel_ /= wheel_radius_;
        back_left_vel_ /= wheel_radius_;
        back_right_vel_ /= wheel_radius_;
    }
    void MecanumChassisController::odom_update(const ros::Time &time, const ros::Duration &period)
    {
        odom_vx_ = ((back_left_joint_.getVelocity() + back_right_joint_.getVelocity()) * wheel_radius_) / 2.0;
        odom_vy_ = ((back_left_joint_.getVelocity() - front_left_joint_.getVelocity()) * wheel_radius_) / 2.0;
        odom_vth_ = ((front_right_joint_.getVelocity() - back_left_joint_.getVelocity()) * wheel_radius_) / ((wheel_base_ + wheel_track_));
        double dt = period.toSec();
        double delta_x = (odom_vx_ * cos(odom_th_) - odom_vy_ * sin(odom_th_)) * dt;
        double delta_y = (odom_vx_ * sin(odom_th_) + odom_vy_ * cos(odom_th_)) * dt;
        double delta_th = odom_vth_ * dt;
        odom_x_ += delta_x;
        odom_y_ += delta_y;
        odom_th_ += delta_th;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odom_th_);

        odom_trans_.header.stamp = time;
        odom_trans_.header.frame_id = "odom";
        odom_trans_.child_frame_id = "base_link";
        odom_trans_.transform.translation.x = odom_x_;
        odom_trans_.transform.translation.y = odom_y_;
        odom_trans_.transform.translation.z = 0.0;
        odom_trans_.transform.rotation = odom_quat;
        odom_broadcaster_.sendTransform(odom_trans_);
        odom_.header.stamp = time;
        odom_.header.frame_id = "odom";
        odom_.pose.pose.position.x = odom_x_;
        odom_.pose.pose.position.y = odom_y_;
        odom_.pose.pose.position.z = 0.0;
        odom_.pose.pose.orientation = odom_quat;
        odom_.child_frame_id = "base_link";
        odom_.twist.twist.linear.x = odom_vx_;
        odom_.twist.twist.linear.y = odom_vy_;
        odom_.twist.twist.angular.z = odom_vth_;
        odom_pub_.publish(odom_);
    }
    void MecanumChassisController::effortback(const sensor_msgs::JointState::ConstPtr &msg)
    {
        // 检查数组大小，确保至少有4个元素才进行访问
        if (msg->effort.size() < 4 || msg->velocity.size() < 4)
        {
            ROS_WARN_THROTTLE(5.0, "Joint state message has insufficient data (effort: %zu, velocity: %zu). Need at least 4 elements.",
                              msg->effort.size(), msg->velocity.size());
            return;
        }
        
        joint_torque_[0] = msg->effort[0] * wheel_radius_; // left_back
        joint_torque_[1] = msg->effort[1] * wheel_radius_; // left_front
        joint_torque_[2] = msg->effort[2] * wheel_radius_; // right_back
        joint_torque_[3] = msg->effort[3] * wheel_radius_; // right_front
        joint_Velocity_[0] = msg->velocity[0];
        joint_Velocity_[1] = msg->velocity[1];
        joint_Velocity_[2] = msg->velocity[2];
        joint_Velocity_[3] = msg->velocity[3];
    }
    PLUGINLIB_EXPORT_CLASS(mecanum_chassis_controller::MecanumChassisController, controller_interface::ControllerBase)
}