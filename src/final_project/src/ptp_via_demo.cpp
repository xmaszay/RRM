#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "final_project/manipulator.hpp"

using namespace std::chrono_literals;

class PtpViaDemo : public rclcpp::Node
{
public:
    PtpViaDemo() : Node("ptp_via_demo"), index_(0)
    {
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        joint_names_ = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

        Manipulator::JointVector q_t2 = {
            0.212505, 0.141679, -0.044757, 1.521074, -0.212008, -1.506295
        };

        Manipulator::JointVector q_via = {
            0.135446, -0.144309, 0.451832, 0.460273, -0.308750, -0.437340
        };

        Manipulator::JointVector q_t3 = {
            0.023699, 0.155486, 0.456767, -3.098603, 0.583753, 3.106401
        };

        trajectory_ = manipulator_.generatePTPVia(q_t2, q_via, q_t3, 2.5, 2.5, 0.01);

        timer_ = this->create_wall_timer(
            10ms, std::bind(&PtpViaDemo::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "ptp_via_demo started");
    }

private:
    void timerCallback()
    {
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
    rclcpp::spin(std::make_shared<PtpViaDemo>());
    rclcpp::shutdown();
    return 0;
}