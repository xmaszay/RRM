#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "final_project/manipulator.hpp"

#include <Eigen/Geometry>

using namespace std::chrono_literals;

class LinDemo : public rclcpp::Node
{
public:
    LinDemo() : Node("lin_demo"), index_(0)
    {
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        joint_names_ = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

        Manipulator::JointVector q_t1 = {
            -0.346254, 0.197993, -0.105621, -1.563038, -0.344984, 1.532185
        };

        Eigen::Affine3d pose_t1 = makePose(-0.55, 0.35, 1.20, 0.0, 0.0, 0.0);
        Eigen::Affine3d pose_t2 = makePose(-0.20, 0.35, 1.20, 0.0, 0.0, 0.0);

        try {
            trajectory_ = manipulator_.generateLIN(pose_t1, pose_t2, q_t1, 4.0, 0.01);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "LIN generation failed: %s", e.what());
        }

        timer_ = this->create_wall_timer(
            10ms, std::bind(&LinDemo::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "lin_demo started");
    }

private:
    Eigen::Affine3d makePose(double x, double y, double z, double rx, double ry, double rz)
    {
        return Eigen::Translation3d(Eigen::Vector3d(x, y, z)) *
               Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX()) *
               Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()) *
               Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ());
    }

    void timerCallback()
    {
        if (trajectory_.empty()) {
            return;
        }

        sensor_msgs::msg::JointState msg;
        msg.header.stamp = this->now();
        msg.name = joint_names_;

        if (index_ < trajectory_.size()) {
            msg.position.assign(trajectory_[index_].q.begin(), trajectory_[index_].q.end());
            ++index_;
        } else {
            msg.position.assign(trajectory_.back().q.begin(), trajectory_.back().q.end());
        }

        joint_pub_->publish(msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<std::string> joint_names_;
    Manipulator manipulator_;
    std::vector<Manipulator::Sample> trajectory_;
    size_t index_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LinDemo>());
    rclcpp::shutdown();
    return 0;
}